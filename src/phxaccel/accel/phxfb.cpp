// PhxFB: the 780M's display as an IOFramebuffer that PhxAccel (NMXAccel) composites for WindowServer.
// Like NullMoth's NMIntelFB (phase 1), it takes over the mode the firmware (GOP) already lit, at boot in place of the
// firmware framebuffer (IONDRV), and writes nothing to the display engine: the scan-out surface is the boot console's
// (IOPlatformExpert::getConsoleInfo), the accelerator's display pipe copies each finished frame into a write-combined CPU
// mapping of it, and a timer at the panel's refresh (measured on the display engine, measureRefresh) stands in for the VBL
// interrupt WindowServer paces an accelerated display on.
#include <IOKit/IOLib.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/graphics/IOFramebuffer.h>
#include <IOKit/graphics/IODisplay.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/pwr_mgt/RootDomain.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <pexpert/pexpert.h>
#include <kern/thread_call.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IORangeAllocator.h>
#include <IOKit/IOSubMemoryDescriptor.h>
#include <libkern/OSAtomic.h>
#include <IOKit/IOCommandGate.h>
#include <sys/sysctl.h>
#include "nvrm_vram_abi.h"
#include "nvrm_gpuva_abi.h"
#include "phx_dcn_pipe.h"
#include "phx_dcn_i2c.h"

void phxAgdcInstall();   // phxagdc.cpp: the AppleGraphicsDeviceControl that reports this panel (debug.phxagdc)
void phxAgdcRemove();
void phxHoldInit(IOService *fb, unsigned bootHoldSec);   // phxagdc.cpp: debug.phxfb_hold keeps this framebuffer (and so the GPU) busy
void phxHoldAdd(IOService *fb);

#define PFBLOG(fmt, ...) IOLog("PhxFB: " fmt "\n", ##__VA_ARGS__)

static int gAlias = 1;
static bool gAliasSysctl;
SYSCTL_INT(_debug, OID_AUTO, phxfb_alias, CTLFLAG_RW | CTLFLAG_LOCKED, &gAlias, 0,
           "1 = frames go through whichever view of the scan-out stored faster when it was first mapped (phxfb_alias_faster), "
           "2 = always through its system-RAM address (the carve-out), 0 = always through the GPU's BAR");
// Which view took stores faster, measured once when the system-RAM view is first mapped: 1 = the system-RAM view,
// 0 = the BAR, -1 = not measured. The 780M stored through the carve-out much faster than through the BAR (2560x1600:
// ~110 ms through the BAR, 2026-10-20); the 8500G the other way round (1920x1080: 23 ms through the BAR, 126 ms through
// the carve-out, write-back or write-combined alike, debug.phxfb_wbench 2026-11-04). So it is measured, not assumed.
static int gAliasFaster = -1;
static SInt64 gAliasStoreUs[2];   // [0] BAR, [1] system-RAM view: microseconds for the measured block
SYSCTL_INT(_debug, OID_AUTO, phxfb_alias_faster, CTLFLAG_RD | CTLFLAG_LOCKED, &gAliasFaster, 0,
           "1 = the system-RAM view stored faster, 0 = the BAR did, -1 = not measured yet");
SYSCTL_QUAD(_debug, OID_AUTO, phxfb_alias_bar_us, CTLFLAG_RD | CTLFLAG_LOCKED, &gAliasStoreUs[0], "startup measure: storing the block through the BAR, us");
SYSCTL_QUAD(_debug, OID_AUTO, phxfb_alias_ram_us, CTLFLAG_RD | CTLFLAG_LOCKED, &gAliasStoreUs[1], "startup measure: storing the block through the system-RAM view, us");
static void phxStoreNt(void *dst, const void *src, size_t n);

// The VBL tick. XNU's timer coalescing moves a kernel deadline by up to an eighth of its delay, at most 1 ms, so a
// 120 Hz tick could land anywhere in the first millisecond after the vblank it stands for. The system-critical urgency
// that is never moved is not exported to kexts (thread_call_enter_delayed_with_leeway), so the tick is armed 1.5 ms
// early and re-armed for its deadline from there, each stage with an eighth of the previous slop, until less than
// 50 us remain (debug.phxfb_vbl_staged, 0 = one plain deadline). The lateness of each tick is summed for the
// measurements, and nvrm-accel.cpp times its presents against the last tick (gPhxVblLastAbs, gPhxFrameAbs).
static int gVblStaged = 1;
static SInt64 gVblN, gVblLateUsSum, gVblLateUsMax;
uint64_t gPhxVblLastAbs, gPhxFrameAbs;
volatile uint64_t gPhxVblSeq;   // ticks so far, counted before the handlers run: the display pipe completes on the next one
SYSCTL_INT(_debug, OID_AUTO, phxfb_vbl_staged, CTLFLAG_RW | CTLFLAG_LOCKED, &gVblStaged, 0,
           "1 = the VBL tick is armed early and re-armed until its deadline (little coalescing slop), 0 = one plain deadline");
SYSCTL_QUAD(_debug, OID_AUTO, phxfb_vbl_n, CTLFLAG_RD | CTLFLAG_LOCKED, &gVblN, "VBL ticks fired");
SYSCTL_QUAD(_debug, OID_AUTO, phxfb_vbl_late_us_sum, CTLFLAG_RW | CTLFLAG_LOCKED, &gVblLateUsSum, "microseconds the VBL ticks fired past their deadlines, summed");
SYSCTL_QUAD(_debug, OID_AUTO, phxfb_vbl_late_us_max, CTLFLAG_RW | CTLFLAG_LOCKED, &gVblLateUsMax, "the latest VBL tick, microseconds past its deadline (writable: 0 restarts it)");
// debug.phxfb_wbench=1: how fast the CPU stores to the scan-out, through each view, and why. Frames are copied in at
// ~190 MB/s on the 8500G (44 ms for 1920x1080, 2026-11-03), which is uncached speed, and a window drag still lags
// behind the damage-only copy. The bench reads the frame once through the BAR into a kernel buffer, then stores the
// same pixels back (movnti, nothing on the panel changes) through the system-RAM view PhxFB uses, through the BAR, and
// through a fresh write-back mapping of the system-RAM view; and it logs the CPU's memory-type MSRs (PAT, MTRRs,
// SYSCFG, TOP_MEM/TOM2) with the variable MTRR that covers the scan-out. Thread context (the sysctl), no locks held.
static SInt64 gWbReadUs, gWbAliasUs, gWbBarUs, gWbWbUs;
extern int gIopFull;   // nvrm-accel.cpp: the next copied frame rewrites every pixel (the bench raced a frame)
static int phxfbWbenchSysctl SYSCTL_HANDLER_ARGS;
SYSCTL_PROC(_debug, OID_AUTO, phxfb_wbench, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, nullptr, 0, phxfbWbenchSysctl, "I",
            "write 1: time stores to the scan-out through each view and log the memory-type MSRs (phxfb_wbench_*_us)");
SYSCTL_QUAD(_debug, OID_AUTO, phxfb_wbench_read_us, CTLFLAG_RD | CTLFLAG_LOCKED, &gWbReadUs, "bench: reading one frame through the BAR, us");
SYSCTL_QUAD(_debug, OID_AUTO, phxfb_wbench_alias_us, CTLFLAG_RD | CTLFLAG_LOCKED, &gWbAliasUs, "bench: storing one frame through the system-RAM view (the one frames use), us");
SYSCTL_QUAD(_debug, OID_AUTO, phxfb_wbench_bar_us, CTLFLAG_RD | CTLFLAG_LOCKED, &gWbBarUs, "bench: storing one frame through the BAR (write-combined), us");
SYSCTL_QUAD(_debug, OID_AUTO, phxfb_wbench_wb_us, CTLFLAG_RD | CTLFLAG_LOCKED, &gWbWbUs, "bench: storing one frame through a write-back mapping of the system-RAM view, us");
int phxSpawnHdmi(int enable);
static int phxfb_hdmi_sysctl SYSCTL_HANDLER_ARGS;
SYSCTL_PROC(_debug, OID_AUTO, phxfb_hdmi, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED | CTLFLAG_KERN,
            NULL, 0, phxfb_hdmi_sysctl, "I",
            "1 = spawn PhxHDMIHead (external 1080p60 HDMI head), 0 = terminate it");

static struct sysctl_oid *const kFbOids[] = { &sysctl__debug_phxfb_alias, &sysctl__debug_phxfb_vbl_staged, &sysctl__debug_phxfb_vbl_n,
                                              &sysctl__debug_phxfb_vbl_late_us_sum, &sysctl__debug_phxfb_vbl_late_us_max,
                                              &sysctl__debug_phxfb_wbench, &sysctl__debug_phxfb_wbench_read_us,
                                              &sysctl__debug_phxfb_wbench_alias_us, &sysctl__debug_phxfb_wbench_bar_us,
                                              &sysctl__debug_phxfb_wbench_wb_us, &sysctl__debug_phxfb_alias_faster,
                                              &sysctl__debug_phxfb_alias_bar_us, &sysctl__debug_phxfb_alias_ram_us,
                                              &sysctl__debug_phxfb_hdmi };

// The panel's backlight is the display block's PWM (DCN 3.1.4 PWRSEQ0, segment 2 base 0x34c0): ACPI's _BCM only asks the
// GPU driver through ATIF (AFN7), and the firmware leaves the PWM on at full duty (period 4000, 12 bits). It is set as
// Linux's panel_cntl does without DMUB: lock the group, write the active count, unlock, wait for the pending update.
// tools/phxctl/phxbl does the same from user space. -phxbloff leaves the panel without a brightness control.
enum : uint32_t { kBlPwrSeq = 0x34c0 + 0x2f14, kBlCntl = 0x34c0 + 0x2f19, kBlPeriod = 0x34c0 + 0x2f1b, kBlLock = 0x34c0 + 0x2f1c };
// The firmware runs both PWM generators, PWRSEQ0 and PWRSEQ1 (+0x6c), at full duty, and PWRSEQ0 alone left the panel's
// brightness as it was (2026-10-25: eDP DPCD 0x721 says the panel follows its PWM pin): both get the level.
static constexpr uint32_t kBlSeqs[] = { 0, 0x2f85 - 0x2f19 };
static constexpr uint32_t kBlMax = 1024;   // the brightness parameter's range
class PhxAMDFB;
static IOService *phxBacklightCreate(PhxAMDFB *fb);

static volatile UInt32 gAllocCookieSeq = 1;
static inline uint64_t nextAllocCookie() {
    uint32_t c = (uint32_t)OSIncrementAtomic(&gAllocCookieSeq);
    return 0x5048580000000000ull | (c ? c : 1);
}

class PhxAMDFB : public IOFramebuffer {
    OSDeclareDefaultStructors(PhxAMDFB)
    friend class PhxHDMIHead;
    friend int phxSpawnHdmi(int enable);
private:
    IOPCIDevice *fPCI = nullptr;
    IOMemoryMap *fSurfMap = nullptr;   // write-combined CPU view of the scan-out surface through the BAR
    // The same surface through its system-RAM address: the 780M's VRAM is a carve-out of system memory, and amdgpu maps
    // it there on APUs rather than through the BAR. Through the BAR a 2560x1600 frame took ~110 ms to copy (~7 fps, the
    // whole desktop lagged, 2026-10-20). This view comes out write-back although it asks for write-combined (frames
    // copied with plain stores stayed in the CPU cache: stripes), so the display pipe writes it with non-temporal stores.
    IOMemoryMap *fAliasMap = nullptr;
    uint64_t fAliasPhys = 0;   // system-RAM address of the scan-out behind fAliasMap (debug.phxfb_wbench)
    IOMemoryMap *fMmio = nullptr;      // BAR5, for the backlight PWM
    IOService *fBacklight = nullptr;   // the brightness parameter handler, named "backlight"
    unsigned fAliasTries = 0;
    volatile UInt32 fAliasBusy = 0;
    struct Irq { IOFBInterruptProc proc = nullptr; OSObject *target = nullptr; void *ref = nullptr; };
    Irq fVbl[4], fConnect;   // IOFramebuffer itself and, when it asks, IOAccelDisplayPipe each register a VBL handler
    thread_call_t fVblTimer = nullptr;
    uint64_t fVblNext = 0;
    volatile bool fStopping = false;
    bool fBeside = false;   // another framebuffer (the firmware's IONDRV one) already drives this GPU: not the console
    IOPhysicalAddress fBase = 0;
    uint32_t fWidth = 0, fHeight = 0, fRowBytes = 0;
    static constexpr uint64_t kFrameNs = 16666667ull;   // 60 Hz, when the refresh cannot be measured
    uint64_t fFrameNs = kFrameNs;
    uint32_t fRefreshMilliHz = 60000, fHTotal = 0, fVTotal = 0;
    // The pipe the firmware lit (phx_dcn_pipe.h): its OTG, the DIG back end driving it and that DIG's signal. DVI/HDMI is
    // an external display; DP (eDP or an external port) keeps the built-in panel the 780M machines have.
    phxdcn::Pipe fPipe;
    // What WindowServer is told. A DVI/HDMI pipe is reported as an external display with its EDID: Test H on the 8500G
    // (2026-11-01) showed "MSI G255F" in System Information with WindowServer compositing on the GPU. (Test G's
    // CoreDisplay assert, display ID 0, was the unsigned Mesa dylib, not this.) Boot-arg -phxfbint presents any pipe the
    // upstream way (built-in, no EDID). DP/eDP pipes are always built-in, as on the 780M machines.
    bool fReportExternal = true, fReportEdid = true;
    bool external() const { return fReportExternal && fPipe.external(); }
    // The external monitor's EDID (phx_dcn_i2c.h: DDC1, the DCN hardware I2C engine as Linux drives it), read once at start
    // for an external pipe and handed to IODisplay through getDDCBlock. Empty: no DDC answer, IODisplay names a generic
    // display as before.
    uint8_t fEdid[256] = {};
    uint32_t fEdidLen = 0;
    static constexpr unsigned kBootHoldSec = 120;   // the boot daemon releases it sooner (debug.phxfb_hold=0)
    // The accelerator is NullMoth's, written against an NVIDIA RM: it backs each IOAccel surface with "VRAM" from the
    // framebuffer (nvAllocVram) and a GPU address from the RM (nvGpuVaAlloc/Map). On the 780M nothing on the GPU reads
    // those surfaces -- the display pipe copies finished frames by CPU, and RADV keeps its own GPU addresses in
    // PhxBringup -- so VRAM here is physically contiguous system memory (the 780M has no other kind) and a GPU address
    // is a number from a private range that never enters a page table. Without them every surface_flush WindowServer
    // makes failed ("failed to alloc GART space"), 30 times a second, and no frame reached the panel (2026-10-20).
    static constexpr uint64_t kVaBase = 0x100000000000ull, kVaSize = 1ull << 40, kVramCap = 1024ull << 20;
    IORangeAllocator *fVa = nullptr;
    volatile SInt64 fVramBytes = 0;
    volatile SInt32 fHandles = 0;
    unsigned fRmLogged = 0;
    // VRAM comes from one physically contiguous pool taken at start, while memory is still unfragmented: allocated one
    // by one, a 16 MB surface found no contiguous run hours into a session, and when WindowServer re-allocated its frame
    // on display wake the panel kept the frame from before the display slept (2026-10-21). An allocation is a
    // sub-descriptor of the pool (its handle); past the pool each one is contiguous memory of its own, as before.
    static constexpr uint64_t kPoolTry[3] = { 512ull << 20, 256ull << 20, 128ull << 20 };
    IOBufferMemoryDescriptor *fPool = nullptr;
    IORangeAllocator *fPoolRange = nullptr;
    uint8_t *fPoolKva = nullptr;
    uint64_t fPoolPhys = 0, fPoolBytes = 0;

    void poolInit() {
        for (uint64_t bytes : kPoolTry) {
            IOBufferMemoryDescriptor *b = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
                kIODirectionInOut | kIOMemoryPhysicallyContiguous, bytes, 0x0000FFFFFFFFF000ull);
            if (b && b->prepare() != kIOReturnSuccess) OSSafeReleaseNULL(b);
            if (!b) continue;
            fPoolRange = IORangeAllocator::withRange(bytes, PAGE_SIZE, 0, IORangeAllocator::kLocking);
            if (!fPoolRange) { b->complete(); b->release(); return; }
            fPool = b; fPoolKva = (uint8_t *)b->getBytesNoCopy(); fPoolBytes = bytes;
            fPoolPhys = b->getPhysicalSegment(0, nullptr, kIOMemoryMapperNone);
            PFBLOG("VRAM pool: %llu MB at phys 0x%llx", bytes >> 20, fPoolPhys);
            return;
        }
        PFBLOG("VRAM pool: no contiguous run of %llu MB or less, every allocation on its own", kPoolTry[2] >> 20);
    }
    void poolFree() {
        OSSafeReleaseNULL(fPoolRange);
        if (fPool) { fPool->complete(); OSSafeReleaseNULL(fPool); }
        fPoolKva = nullptr; fPoolPhys = fPoolBytes = 0;
    }

    bool poolAlloc(NVRMVramRequest *r, uint64_t bytes) {
        IORangeScalar off = 0;
        if (!fPoolRange || !fPoolRange->allocate(bytes, &off, PAGE_SIZE)) return false;
        IOSubMemoryDescriptor *s = IOSubMemoryDescriptor::withSubRange(fPool, off, bytes, kIODirectionInOut);
        if (!s) { fPoolRange->deallocate(off, bytes); return false; }
        bzero(fPoolKva + off, bytes);
        r->handle = s; r->kva = fPoolKva + off; r->actualSize = bytes; r->allocationCookie = nextAllocCookie();
        r->phys = fPoolPhys + off;
        return true;
    }

    IOReturn vramAlloc(NVRMVramRequest *r) {
        if (!r || r->version != NVRM_VRAM_ABI_VERSION || !r->size) return kIOReturnBadArgument;
        const uint64_t bytes = round_page_64(r->size);
        if ((uint64_t)OSAddAtomic64((SInt64)bytes, &fVramBytes) + bytes > kVramCap) {
            OSAddAtomic64(-(SInt64)bytes, &fVramBytes);
            if (fRmLogged++ < 16) PFBLOG("VRAM: %llu bytes refused, %lld already out (cap %llu)", bytes, fVramBytes, kVramCap);
            return kIOReturnNoMemory;
        }
        if (poolAlloc(r, bytes)) {
            r->mappedTotal = (uint64_t)fVramBytes;
            if (fRmLogged++ < 16) PFBLOG("VRAM: %llu bytes at phys 0x%llx from the pool (%lld out)", bytes, r->phys, fVramBytes);
            return kIOReturnSuccess;
        }
        IOBufferMemoryDescriptor *b = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
            kIODirectionInOut | kIOMemoryPhysicallyContiguous, bytes, 0x0000FFFFFFFFF000ull);
        if (b && b->prepare() != kIOReturnSuccess) OSSafeReleaseNULL(b);
        if (!b) {
            OSAddAtomic64(-(SInt64)bytes, &fVramBytes);
            if (fRmLogged++ < 16) PFBLOG("VRAM: no contiguous %llu bytes", bytes);
            return kIOReturnNoMemory;
        }
        bzero(b->getBytesNoCopy(), bytes);
        r->handle = b; r->kva = b->getBytesNoCopy(); r->actualSize = bytes; r->allocationCookie = nextAllocCookie();
        r->phys = b->getPhysicalSegment(0, nullptr, kIOMemoryMapperNone);
        r->mappedTotal = (uint64_t)fVramBytes;
        if (fRmLogged++ < 16) PFBLOG("VRAM: %llu bytes at phys 0x%llx (%lld out)", bytes, r->phys, fVramBytes);
        return kIOReturnSuccess;
    }
    IOReturn vramFree(NVRMVramRequest *r) {
        if (!r || r->version != NVRM_VRAM_ABI_VERSION) return kIOReturnBadArgument;
        if (IOSubMemoryDescriptor *s = OSDynamicCast(IOSubMemoryDescriptor, (OSObject *)r->handle)) {
            const uint64_t off = (uint64_t)((uint8_t *)r->kva - fPoolKva), len = s->getLength();
            if (!fPoolRange || (uint8_t *)r->kva < fPoolKva || off + len > fPoolBytes) return kIOReturnBadArgument;
            OSAddAtomic64(-(SInt64)len, &fVramBytes);
            fPoolRange->deallocate(off, len);
            s->release();
            return kIOReturnSuccess;
        }
        IOBufferMemoryDescriptor *b = OSDynamicCast(IOBufferMemoryDescriptor, (OSObject *)r->handle);
        if (!b) return kIOReturnBadArgument;
        OSAddAtomic64(-(SInt64)b->getLength(), &fVramBytes);
        b->complete(); b->release();
        return kIOReturnSuccess;
    }
    IOReturn rmCall(const OSSymbol *fn, void *p1) {
        if (fn->isEqualTo(NVRM_GPUVA_FN_HANDLES)) {
            NVRMRmHandles *h = (NVRMRmHandles *)p1;
            if (!h || h->version != NVRM_GPUVA_ABI_VERSION) return kIOReturnBadArgument;
            h->kapiDevice = this; h->hClient = 0x50480001; h->hDevice = 0x50480002; h->hSubDevice = 0x50480003;
            return kIOReturnSuccess;
        }
        NVRMGpuVaRequest *g = (NVRMGpuVaRequest *)p1;
        if (!g || g->version != NVRM_GPUVA_ABI_VERSION) return kIOReturnBadArgument;
        if (fn->isEqualTo(NVRM_GPUVA_FN_ALLOC)) {
            const uint64_t len = round_page_64(g->size);
            uint64_t align = g->align > PAGE_SIZE ? g->align : PAGE_SIZE;
            if (align & (align - 1)) align = PAGE_SIZE;
            IORangeScalar off = 0;
            if (!fVa || !len || !fVa->allocate(len, &off, align)) { g->status = 1; return kIOReturnNoSpace; }
            g->gpuva = kVaBase + off; g->hVirt = (unsigned)OSIncrementAtomic(&fHandles) + 1; g->status = 0;
            return kIOReturnSuccess;
        }
        if (fn->isEqualTo(NVRM_GPUVA_FN_MAP)) {   // nothing to map: no GPU work reaches these surfaces
            g->hMemory = (unsigned)OSIncrementAtomic(&fHandles) + 1; g->status = 0;
            return kIOReturnSuccess;
        }
        if (fn->isEqualTo(NVRM_GPUVA_FN_FREE)) {
            if (!(g->flags & NVRM_GPUVA_FLAG_UNMAP_ONLY) && fVa && g->gpuva >= kVaBase && g->gpuva - kVaBase < kVaSize)
                fVa->deallocate(g->gpuva - kVaBase, round_page_64(g->size));
            g->status = 0;
            return kIOReturnSuccess;
        }
        return kIOReturnUnsupported;
    }

    static void vblFire(thread_call_param_t p0, thread_call_param_t) {
        PhxAMDFB *me = (PhxAMDFB *)p0;
        if (me->fStopping || me->fBlDark) return;
        const uint64_t at = mach_absolute_time(), due = me->fVblNext;
        uint64_t near; nanoseconds_to_absolutetime(50000, &near);
        if (gVblStaged && due > at + near) { thread_call_enter_delayed(me->fVblTimer, due); return; }   // an early stage
        if (due && at > due) {
            uint64_t ns; absolutetime_to_nanoseconds(at - due, &ns);
            const SInt64 us = (SInt64)(ns / 1000);
            OSAddAtomic64(us, &gVblLateUsSum);
            if (us > gVblLateUsMax) gVblLateUsMax = us;
        }
        OSAddAtomic64(1, &gVblN);
        gPhxVblLastAbs = due ? due : at;   // the vblank this tick stands for
        __atomic_add_fetch(&gPhxVblSeq, 1, __ATOMIC_RELEASE);
        for (const Irq &q : me->fVbl) {
            const IOFBInterruptProc proc = __atomic_load_n(&q.proc, __ATOMIC_ACQUIRE);
            if (proc) proc(q.target, q.ref);
        }
        uint64_t period; nanoseconds_to_absolutetime(me->fFrameNs, &period);
        const uint64_t now = mach_absolute_time();
        me->fVblNext = (me->fVblNext && me->fVblNext + period > now) ? me->fVblNext + period : now + period;
        uint64_t lead; nanoseconds_to_absolutetime(1500000, &lead);
        thread_call_enter_delayed(me->fVblTimer, gVblStaged ? me->fVblNext - lead : me->fVblNext);
    }

    // Thread context (start): MMIO through the BAR5 view, every poll bounded (phx_dcn_i2c.h).
    void readEdid(volatile uint32_t *r) {
        struct Hw {
            volatile uint32_t *r;
            uint32_t rd(uint32_t reg) { return r[reg]; }
            void wr(uint32_t reg, uint32_t v) { r[reg] = v; }
            void delayUs(uint32_t us) { IODelay(us); }
        } hw{r};
        const uint32_t res = phxi2c::readEdid(hw, fEdid, &fEdidLen);
        if (res != phxi2c::kOk) { fEdidLen = 0; PFBLOG("EDID over DDC1: %s", phxi2c::resultName(res)); return; }
        const uint16_t mfg = (uint16_t)(fEdid[8] << 8 | fEdid[9]);
        char name[14] = {};
        for (int d = 54; d < 126; d += 18)
            if (!fEdid[d] && !fEdid[d + 1] && fEdid[d + 3] == 0xfc)
                for (int k = 0; k < 13 && fEdid[d + 5 + k] != 0x0a; k++) name[k] = (char)fEdid[d + 5 + k];
        PFBLOG("EDID over DDC1: %u bytes, %c%c%c product 0x%04x \"%s\"", fEdidLen, '@' + ((mfg >> 10) & 31),
               '@' + ((mfg >> 5) & 31), '@' + (mfg & 31), fEdid[10] | fEdid[11] << 8, name);
    }

    // The firmware lit the panel at its native refresh, which the VBL timer and the mode must report: they said 60 Hz while
    // the panel scanned at 120 (OTG frame counter: 392 frames in 3.28 s), and WindowServer composited at 60 fps (2026-10-20).
    // Read-only: the running OTG's H/V totals and its frame counter over ~200 ms, snapped to a standard rate when within
    // 2 %.
    void measureRefresh() {
        IOMemoryMap *mm = fPCI ? fPCI->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress5) : nullptr;
        // BAR5 is 512 KiB; the highest register read is phxdcn::kHighestReg
        const uint64_t highest = phxdcn::kHighestReg > phxi2c::kHighestReg ? phxdcn::kHighestReg : phxi2c::kHighestReg;
        if (!mm || mm->getLength() < (highest + 1) * 4) { OSSafeReleaseNULL(mm); PFBLOG("refresh: no MMIO view, 60 Hz assumed"); return; }
        volatile uint32_t *r = (volatile uint32_t *)mm->getVirtualAddress();
        using namespace phxdcn;
        fPipe = pipe([r](uint32_t reg) { return r[reg]; });
        if (fPipe.otg < 0) { mm->release(); PFBLOG("pipe: no OTG running: built-in panel, 60 Hz assumed"); return; }
        if (fPipe.dig < 0) PFBLOG("pipe: OTG%d, no enabled DIG takes it: built-in panel assumed", fPipe.otg);
        else PFBLOG("pipe: OTG%d -> DIG%d, signal %s (DIG_MODE %u): %s, reported %s%s", fPipe.otg, fPipe.dig,
                    modeName(fPipe.digMode), fPipe.digMode, fPipe.external() ? "external display" : "built-in panel (eDP or DP)",
                    external() ? "external" : "built-in", fReportEdid ? " with EDID" : "");
        if (fPipe.external()) readEdid(r);
        const uint32_t o = (uint32_t)fPipe.otg;
        const uint32_t ht = r[otgReg(o, kOtgHTotal)] + 1, vt = r[otgReg(o, kOtgVTotal)] + 1;
        const uint32_t c0 = r[otgReg(o, kOtgFrameCount)] & 0xffffff; const uint64_t t0 = mach_absolute_time();
        IOSleep(200);
        const uint32_t c1 = r[otgReg(o, kOtgFrameCount)] & 0xffffff; const uint64_t t1 = mach_absolute_time();
        mm->release();
        uint64_t ns; absolutetime_to_nanoseconds(t1 - t0, &ns);
        const uint32_t frames = (c1 - c0) & 0xffffff;
        if (!ns || frames < 4 || ht < 100 || vt < 100) { PFBLOG("refresh: OTG%u not counting (%u frames), 60 Hz assumed", o, frames); return; }
        uint32_t mhz = (uint32_t)(((uint64_t)frames * 1000000000000ull) / ns);
        static const uint32_t kStd[] = { 48000, 50000, 60000, 75000, 90000, 100000, 120000, 144000, 165000, 180000, 240000 };
        for (uint32_t k : kStd) if (mhz > k * 49 / 50 && mhz < k * 51 / 50) { mhz = k; break; }
        if (mhz < 24000 || mhz > 360000) { PFBLOG("refresh: %u mHz measured, out of range - 60 Hz assumed", mhz); return; }
        fRefreshMilliHz = mhz; fHTotal = ht; fVTotal = vt;
        fFrameNs = 1000000000000ull / mhz;
        PFBLOG("refresh: OTG%u %ux%u totals, %u frames in %llu us -> %u.%03u Hz", o, ht, vt, frames, ns / 1000, mhz / 1000, mhz % 1000);
    }

public:
    // the backlight as 0..kBlMax of full duty; false without a running PWM
    bool backlightRead(uint32_t *level) {
        if (!fMmio || !level) return false;
        volatile uint32_t *r = (volatile uint32_t *)fMmio->getVirtualAddress();
        const uint32_t pc = r[kBlPeriod], period = pc & 0xffff, bits = (pc >> 16) & 0xf, cntl = r[kBlCntl];
        const uint64_t full = bits ? ((uint64_t)0x10000 * period) >> bits : 0;
        if (!full || !(cntl >> 31)) return false;
        const uint64_t l = (uint64_t)(cntl & 0xffff) * kBlMax / full;
        *level = l > kBlMax ? kBlMax : (uint32_t)l;
        return true;
    }
    // level 0 is dark, as on a MacBook: AppleBacklightDisplay puts the panel to sleep with brightness 0
    bool backlightWrite(uint32_t level) {
        if (!fMmio) return false;
        volatile uint32_t *r = (volatile uint32_t *)fMmio->getVirtualAddress();
        if (level > kBlMax) level = kBlMax;
        uint64_t frac = (uint64_t)level * 0x10000 / kBlMax;   // 16.16
        if (level && frac < 0x10000 / 100) frac = 0x10000 / 100;   // above dark, at least 1 % of full duty
        bool any = false;
        for (uint32_t q : kBlSeqs) {
            const uint32_t pc = r[kBlPeriod + q], period = pc & 0xffff, bits = (pc >> 16) & 0xf;
            if (!period || !bits || !(r[kBlCntl + q] >> 31)) continue;
            const uint64_t duty = frac * period;
            uint32_t cnt = (uint32_t)((duty >> bits) & 0xffff) + (uint32_t)((duty >> (bits - 1)) & 1);
            if (cnt > 0xffff) cnt = 0xffff;
            r[kBlLock + q] = r[kBlLock + q] | 0x80000000u | 1u;                 // IGNORE_MASTER_LOCK_EN, REG_LOCK
            r[kBlCntl + q] = (r[kBlCntl + q] & ~0xffffu) | cnt;                 // BL_ACTIVE_INT_FRAC_CNT
            r[kBlLock + q] = (r[kBlLock + q] | 0x80000000u) & ~1u;
            for (int i = 0; i < 100 && (r[kBlLock + q] & 0x100); i++) IODelay(100);   // REG_UPDATE_PENDING
            any = true;
        }
        if (level == 0) {
            r[kBlPwrSeq] = r[kBlPwrSeq] & ~1u;   // cut PANEL_BLON
        } else {
            r[kBlPwrSeq] = r[kBlPwrSeq] | 1u;    // enable PANEL_BLON
        }
        return any;
    }
    // Display sleep left the backlight lit over a black or torn frame (2026-10-25). Besides brightness 0, a connection
    // powered down goes dark and gets its level back on power-up; a brightness set while dark waits.
    bool fBlDark = false;
    uint32_t fBlSaved = kBlMax;
    bool backlightSet(uint32_t level) {
        if (fBlDark) { fBlSaved = level > kBlMax ? kBlMax : level; return true; }
        return backlightWrite(level);
    }
    void displayPower(bool on) {
        if (!fBacklight || on == !fBlDark) return;
        if (!on) {
            backlightRead(&fBlSaved);
            if (!fBlSaved) fBlSaved = kBlMax;
            fBlDark = true;
            backlightWrite(0);
        } else {
            fBlDark = false;
            fVblNext = 0;
            backlightWrite(fBlSaved ? fBlSaved : kBlMax);
            if (fVblTimer) {
                uint64_t d; clock_interval_to_deadline((uint32_t)(fFrameNs / 1000), kMicrosecondScale, &d);
                fVblNext = d;
                thread_call_enter_delayed(fVblTimer, d);
            }
        }
        if (fBacklight) fBacklight->setProperty("PhxDark", fBlDark);
        PFBLOG("backlight: display power %s, level %u", on ? "on" : "off", fBlSaved);
    }
    bool backlightDark() const { return fBlDark; }
    // which calls reach the panel, newest first, as the registry property PhxTrace (the brightness slider moved and no
    // brightness parameter arrived, 2026-10-25); the log does not reach the unified log
    char fTrace[16][48] = {}; unsigned fTraceN = 0;
    void trace(const char *what, uint32_t a, uint64_t v) {
        char line[48]; snprintf(line, sizeof line, "%s %c%c%c%c=0x%llx", what, (char)(a >> 24), (char)(a >> 16), (char)(a >> 8), (char)a,
                                (unsigned long long)v);
        if (fTraceN && !strcmp(line, fTrace[(fTraceN - 1) % 16])) return;   // repeats of the last call
        strlcpy(fTrace[fTraceN++ % 16], line, sizeof fTrace[0]);
        char all[16 * 48 + 16] = ""; for (unsigned i = 0; i < 16 && i < fTraceN; i++) { strlcat(all, fTrace[(fTraceN - 1 - i) % 16], sizeof all); strlcat(all, "; ", sizeof all); }
        setProperty("PhxTrace", all);
    }
    // the built-in panel is a TFT: AppleBacklightDisplay then drives it (brightness, dimming), as on a MacBook
    IOReturn getAppleSense(IOIndex idx, UInt32 *senseType, UInt32 *primary, UInt32 *extended, UInt32 *displayType) override {
        if (idx == 0) {
            if (!fBacklight) return kIOReturnUnsupported;
            if (senseType) *senseType = 0;
            if (primary) *primary = 0;
            if (extended) *extended = 0;
            if (displayType) *displayType = 2;   // kPanelTFTConnect (IOMacOSVideo.h, the NDRV headers)
            return kIOReturnSuccess;
        }
        return kIOReturnUnsupported;
    }
    bool start(IOService *provider) override;
    void stop(IOService *provider) override;
    IOReturn enableController() override { return kIOReturnSuccess; }
    bool isConsoleDevice() override { return !fBeside; }
    IODeviceMemory *getApertureRange(IOPixelAperture aperture) override {
        if (aperture != kIOFBSystemAperture) return nullptr;
        return IODeviceMemory::withRange(fBase, (IOPhysicalLength)fRowBytes * fHeight);
    }
    IODeviceMemory *getVRAMRange() override { return getApertureRange(kIOFBSystemAperture); }
    const char *getPixelFormats() override { return IO32BitDirectPixels "\0"; }
    IOItemCount getDisplayModeCount() override { return 1; }
    IOReturn getDisplayModes(IODisplayModeID *modes) override { if (modes) modes[0] = 1; return kIOReturnSuccess; }
    IOReturn getInformationForDisplayMode(IODisplayModeID mode, IODisplayModeInformation *info) override {
        if (mode != 1 || !info) return kIOReturnBadArgument;
        bzero(info, sizeof(*info));
        info->maxDepthIndex = 0; info->nominalWidth = fWidth; info->nominalHeight = fHeight;
        info->refreshRate = (IOFixed1616)(((uint64_t)fRefreshMilliHz << 16) / 1000);
        info->flags = kDisplayModeValidFlag | kDisplayModeSafeFlag | kDisplayModeDefaultFlag | kFramebufferSupportsGammaCorrection;
        return kIOReturnSuccess;
    }
    UInt64 getPixelFormatsForDisplayMode(IODisplayModeID, IOIndex) override { return 0; }
    IOReturn getPixelInformation(IODisplayModeID mode, IOIndex depth, IOPixelAperture aperture, IOPixelInformation *pi) override {
        if (mode != 1 || depth != 0 || aperture != kIOFBSystemAperture || !pi) return kIOReturnUnsupportedMode;
        bzero(pi, sizeof(*pi));
        pi->bytesPerRow = fRowBytes; pi->bitsPerPixel = 32; pi->pixelType = kIORGBDirectPixels;
        pi->componentCount = 3; pi->bitsPerComponent = 8;
        pi->componentMasks[0] = 0x00FF0000; pi->componentMasks[1] = 0x0000FF00; pi->componentMasks[2] = 0x000000FF;
        strlcpy(pi->pixelFormat, IO32BitDirectPixels, sizeof(pi->pixelFormat));
        pi->activeWidth = fWidth; pi->activeHeight = fHeight;
        return kIOReturnSuccess;
    }
    IOReturn getCurrentDisplayMode(IODisplayModeID *mode, IOIndex *depth) override { if (mode) *mode = 1; if (depth) *depth = 0; return kIOReturnSuccess; }
    IOReturn getStartupDisplayMode(IODisplayModeID *mode, IOIndex *depth) override { return getCurrentDisplayMode(mode, depth); }
    IOReturn setDisplayMode(IODisplayModeID mode, IOIndex depth) override { return (mode == 1 && depth == 0) ? kIOReturnSuccess : kIOReturnUnsupported; }
    IOReturn getTimingInfoForDisplayMode(IODisplayModeID mode, IOTimingInformation *info) override {
        if (mode != 1 || !info) return kIOReturnUnsupportedMode;
        bzero(info, sizeof(*info));
        info->appleTimingID = kIOTimingIDInvalid;   // no detailed timing: the panel keeps the firmware's
        return kIOReturnSuccess;
    }
    IOItemCount getConnectionCount() override { return 1; }
    IOReturn registerForInterruptType(IOSelect type, IOFBInterruptProc proc, OSObject *target, void *ref, void **iref) override {
        if (type == kIOFBVBLInterruptType) {
            for (Irq &q : fVbl) if (!q.proc) {
                q.target = target; q.ref = ref; __atomic_store_n(&q.proc, proc, __ATOMIC_RELEASE);
                PFBLOG("VBL handler %u: %s", (unsigned)(&q - fVbl), target ? target->getMetaClass()->getClassName() : "(no target)");
                if (iref) *iref = &q;
                return kIOReturnSuccess;
            }
            PFBLOG("VBL handler refused: all %u taken", (unsigned)(sizeof fVbl / sizeof fVbl[0]));
            return kIOReturnNoResources;
        }
        if (type == kIOFBConnectInterruptType) { fConnect = { proc, target, ref }; if (iref) *iref = &fConnect; return kIOReturnSuccess; }
        return kIOReturnUnsupported;
    }
    IOReturn unregisterInterrupt(void *iref) override {
        for (Irq &q : fVbl) if (iref == &q) __atomic_store_n(&q.proc, (IOFBInterruptProc)nullptr, __ATOMIC_RELEASE);
        return kIOReturnSuccess;
    }
    IOReturn setInterruptState(void *, UInt32) override { return kIOReturnSuccess; }
    IOReturn setGammaTable(UInt32, UInt32, UInt32, void *) override { return kIOReturnSuccess; }
    IOReturn setGammaTable(UInt32, UInt32, UInt32, void *, bool) override { return kIOReturnSuccess; }
    IOReturn setCLUTWithEntries(IOColorEntry *, UInt32, UInt32, IOOptionBits) override { return kIOReturnSuccess; }
    IOReturn setApertureEnable(IOPixelAperture, IOOptionBits) override { return kIOReturnSuccess; }
    IOReturn setStartupDisplayMode(IODisplayModeID, IOIndex) override { return kIOReturnSuccess; }
    IOCommandGate *fWakeGate = nullptr;
    bool fSysSlept = false;

    void familyDidWake() {
        IOWorkLoop *wl = getWorkLoop();
        if (!fWakeGate && wl) {
            fWakeGate = IOCommandGate::commandGate(this);
            if (fWakeGate && wl->addEventSource(fWakeGate) != kIOReturnSuccess) OSSafeReleaseNULL(fWakeGate);
        }
        if (!fWakeGate) {
            PFBLOG("system wake: no command gate on workloop, delivering direct");
            deliverFramebufferNotification(kIOFBNotifyDidWake, (void *)true);
            deliverFramebufferNotification(kIOFBNotifyDidWake, (void *)false);
            deliverFramebufferNotification(kIOFBNotifyDidPowerOn, (void *)false);
            pagingState = true;
            return;
        }
        fWakeGate->runAction(OSMemberFunctionCast(IOCommandGate::Action, this, &PhxAMDFB::familyDidWakeGated));
    }

    IOReturn familyDidWakeGated(void *, void *, void *, void *) {
        deliverFramebufferNotification(kIOFBNotifyDidWake, (void *)true);
        deliverFramebufferNotification(kIOFBNotifyDidWake, (void *)false);
        deliverFramebufferNotification(kIOFBNotifyDidPowerOn, (void *)false);
        pagingState = true;
        if (fWakeGate) fWakeGate->commandWakeup(&fServerConnect, false);
        PFBLOG("system wake: clients told DidWake, pagingState on, WindowServer woken");
        return kIOReturnSuccess;
    }

    // power and connection answers: acknowledge at once, never queue on the controller thread.
    // On wake, deliver DidWake, set pagingState, and wake WindowServer sleeping on fServerConnect.
    IOReturn setPowerState(unsigned long ordinal, IOService *) override {
        displayPower(ordinal != 0);
        if (ordinal) {
            if (!pagingState && fSysSlept) { fSysSlept = false; familyDidWake(); }
            else handleEvent(kIOFBNotifyDidPowerOn);
        } else {
            fSysSlept = true;
            handleEvent(kIOFBNotifyWillPowerOff);
        }
        return kIOPMAckImplied;
    }
    IOReturn setAttribute(IOSelect attribute, uintptr_t value) override {
        trace("attr", attribute, value);
        if (attribute == kIOWindowServerActiveAttribute) return kIOReturnSuccess;
        if (attribute == kIOPowerAttribute) {
            displayPower(value != 0);
            if (value) {
                if (!pagingState && fSysSlept) { fSysSlept = false; familyDidWake(); }
                else handleEvent(kIOFBNotifyDidPowerOn);
                return kIOReturnSuccess;
            }
            fSysSlept = true;
            handleEvent(kIOFBNotifyWillPowerOff);
            return kIOReturnSuccess;
        }
        return IOFramebuffer::setAttribute(attribute, value);
    }
    IOReturn getAttribute(IOSelect attribute, uintptr_t *value) override {
        if (attribute == kIOWindowServerActiveAttribute) { if (value) *value = 1; return kIOReturnSuccess; }
        return IOFramebuffer::getAttribute(attribute, value);
    }
    IOReturn setAttributeForConnection(IOIndex idx, IOSelect attribute, uintptr_t value) override {
        trace("conn", attribute, value);
        switch (attribute) {
        case kConnectionPower:
            displayPower(value != 0);
            return kIOReturnSuccess;
        case kConnectionColorModesSupported: case kConnectionColorDepthsSupported:
        case kConnectionControllerColorDepth: case kConnectionControllerDepthsSupported: case kConnectionColorMode:
        case kConnectionDisplayFlags: case kConnectionFlags:
            return kIOReturnSuccess;
        default: return IOFramebuffer::setAttributeForConnection(idx, attribute, value);
        }
    }
    // EDID for IODisplay: block 1 = bytes 0..127, block 2 = the CEA extension (IOFramebuffer numbers blocks from 1)
    bool hasDDCConnect(IOIndex idx) override { return idx == 0 && fReportEdid && fEdidLen >= 128; }
    IOReturn getDDCBlock(IOIndex idx, UInt32 block, IOSelect type, IOOptionBits, UInt8 *data, IOByteCount *length) override {
        if (!fReportEdid || idx != 0 || type != kIODDCBlockTypeEDID || !data || !length || block < 1 || block * 128 > fEdidLen)
            return kIOReturnUnsupported;
        const IOByteCount n = *length < 128 ? *length : 128;
        bcopy(fEdid + (block - 1) * 128, data, n);
        *length = n;
        return kIOReturnSuccess;
    }
    IOReturn getAttributeForConnection(IOIndex idx, IOSelect attr, uintptr_t *value) override {
        if (idx == 0 && attr == kConnectionSupportsHLDDCSense)
            return (fReportEdid && fEdidLen >= 128) ? kIOReturnSuccess : kIOReturnUnsupported;
        if (idx >= 2 || !value) return kIOReturnBadArgument;
        if (idx == 0) {
            switch (attr) {
            case kConnectionEnable: case kConnectionCheckEnable: *value = 1; return kIOReturnSuccess;
            case kConnectionFlags: *value = external() ? 0 : kIOConnectionBuiltIn; return kIOReturnSuccess;
            case kConnectionColorModesSupported: case kConnectionColorMode: *value = 0x00000001; return kIOReturnSuccess;
            case kConnectionColorDepthsSupported: case kConnectionControllerColorDepth: case kConnectionControllerDepthsSupported:
                *value = 0x00000002; return kIOReturnSuccess;
            default: return IOFramebuffer::getAttributeForConnection(idx, attr, value);
            }
        } else {
            switch (attr) {
            case kConnectionEnable: case kConnectionCheckEnable: *value = 1; return kIOReturnSuccess;
            case kConnectionFlags: *value = 0; return kIOReturnSuccess;
            case kConnectionSupportsHLDDCSense: case kConnectionSupportsLLDDCSense:
                *value = 1; return kIOReturnSuccess;
            case kConnectionColorModesSupported: case kConnectionColorMode: *value = 0x00000001; return kIOReturnSuccess;
            case kConnectionColorDepthsSupported: case kConnectionControllerColorDepth: case kConnectionControllerDepthsSupported:
                *value = 0x00000002; return kIOReturnSuccess;
            default: return IOFramebuffer::getAttributeForConnection(idx, attr, value);
            }
        }
    }
    IOReturn connectFlags(IOIndex connectIndex, IODisplayModeID mode, IOOptionBits *flags) override {
        if (!flags) return kIOReturnBadArgument;
        *flags = 0;
        if (connectIndex == 0 && mode == 1) {
            *flags = kDisplayModeValidFlag | kDisplayModeSafeFlag | kDisplayModeDefaultFlag;
        } else if (connectIndex == 1 && mode == 2) {
            *flags = kDisplayModeValidFlag | kDisplayModeSafeFlag | kDisplayModeDefaultFlag;
        }
        return kIOReturnSuccess;
    }
    // The accelerator's display pipe asks for the scan-out: its CPU mapping (frames are copied in) and its physical
    // address in the GPU's VRAM aperture. With no address the pipe got no framebuffer resource, and IOAcceleratorFamily
    // marks a display pipe active only once one exists (init_framebuffer_resource sets +0x298): every WindowServer
    // transaction ended kIOReturnNotReady and no frame reached the panel (2026-10-20).
    // The scan-out through the carve-out, once PhxBringup has published where the carve-out sits (phx-fb-phys) and only
    // if that view holds the pixels the BAR shows. NULL (copy through the BAR) until then.
    IOMemoryMap *aliasMap() {
        if (fAliasMap || fAliasTries >= 3) return fAliasMap;
        if (!OSCompareAndSwap(0, 1, &fAliasBusy)) return nullptr;
        if (!fAliasMap && fAliasTries < 3) {
            fAliasTries++;
            uint64_t fbPhys = 0, barOff = ~0ull;
            if (OSIterator *it = fPCI->getChildIterator(gIOServicePlane)) {
                while (OSObject *o = it->getNextObject()) {
                    IORegistryEntry *e = OSDynamicCast(IORegistryEntry, o);
                    OSNumber *n = e ? OSDynamicCast(OSNumber, e->getProperty("phx-fb-phys")) : nullptr;
                    if (n) { fbPhys = n->unsigned64BitValue(); break; }
                }
                it->release();
            }
            for (uint32_t i = 0; i < fPCI->getDeviceMemoryCount(); i++) {
                IODeviceMemory *dm = fPCI->getDeviceMemoryWithIndex(i);
                if (dm && fBase >= dm->getPhysicalAddress() && fBase - dm->getPhysicalAddress() < dm->getLength()) {
                    barOff = fBase - dm->getPhysicalAddress();
                    break;
                }
            }
            const uint64_t phys = fbPhys + barOff, bytes = (uint64_t)fRowBytes * fHeight;
            IOMemoryMap *m = nullptr;
            if (fbPhys && barOff != ~0ull)
                if (IODeviceMemory *dm = IODeviceMemory::withRange(phys, bytes)) { m = dm->map(kIOMapWriteCombineCache); dm->release(); }
            if (m) {
                const volatile uint32_t *a = (const volatile uint32_t *)m->getVirtualAddress();
                const volatile uint32_t *b = (const volatile uint32_t *)fSurfMap->getVirtualAddress();
                const uint64_t words = bytes / 4, step = words / 4096 ? words / 4096 : 1;
                unsigned same = 0, nz = 0, n = 0;
                for (uint64_t w = 0; w < words; w += step, n++) { const uint32_t x = a[w], y = b[w]; same += x == y; nz += y != 0; }
                PFBLOG("alias: scan-out at BAR offset 0x%llx = system RAM 0x%llx: %u of %u samples match the BAR (%u non-zero)",
                       barOff, phys, same, n, nz);
                if (same == n && nz) { fAliasMap = m; fAliasPhys = phys; measureViews(); } else m->release();
            } else {
                PFBLOG("alias: carve-out 0x%llx, scan-out BAR offset 0x%llx: no system-RAM view, copying through the BAR", fbPhys, barOff);
            }
        }
        fAliasBusy = 0;
        return fAliasMap;
    }
    void writeBench();
    // Once, right after the system-RAM view matched the BAR: time storing the same 1 MB (the top of the frame, read
    // through the system-RAM view, so the panel does not change) through each view, a few rounds each, best of each.
    // Thread context (the scan-out request), ~10-100 ms once. The result picks the view for debug.phxfb_alias=1.
    void measureViews() {
        const size_t bytes = 1u << 20;
        if (!fAliasMap || !fSurfMap || (uint64_t)fRowBytes * fHeight < bytes) return;
        void *buf = IOMallocAligned(bytes, 64);
        if (!buf) return;
        memcpy(buf, (const void *)fAliasMap->getVirtualAddress(), bytes);
        void *dst[2] = {(void *)fSurfMap->getVirtualAddress(), (void *)fAliasMap->getVirtualAddress()};
        uint64_t best[2] = {~0ull, ~0ull};
        for (unsigned round = 0; round < 3; round++)
            for (unsigned v = 0; v < 2; v++) {
                const uint64_t t = mach_absolute_time();
                phxStoreNt(dst[v], buf, bytes);
                uint64_t ns; absolutetime_to_nanoseconds(mach_absolute_time() - t, &ns);
                if (ns < best[v]) best[v] = ns;
            }
        IOFreeAligned(buf, bytes);
        gAliasStoreUs[0] = (SInt64)(best[0] / 1000); gAliasStoreUs[1] = (SInt64)(best[1] / 1000);
        gAliasFaster = best[1] <= best[0] ? 1 : 0;
        PFBLOG("alias: storing 1 MB took %lld us through the BAR, %lld us through system RAM: frames go through %s",
               gAliasStoreUs[0], gAliasStoreUs[1], gAliasFaster ? "system RAM" : "the BAR");
    }
    IOReturn callPlatformFunction(const OSSymbol *fn, bool wait, void *p1, void *p2, void *p3, void *p4) override {
        if (fn && fn->isEqualTo(NVRM_VRAM_FN_SCANOUT)) {
            NVRMVramRequest *r = (NVRMVramRequest *)p1;
            if (!r || r->version != NVRM_VRAM_ABI_VERSION) return kIOReturnBadArgument;
            if (!fSurfMap) return kIOReturnNotReady;
            IOMemoryMap *alias = gAlias ? aliasMap() : nullptr;
            if (alias && gAlias == 1 && gAliasFaster == 0) alias = nullptr;   // measured: the BAR stores faster
            r->kva = (void *)(alias ? alias : fSurfMap)->getVirtualAddress();
            r->phys = fBase; r->actualSize = round_page_64((uint64_t)fRowBytes * fHeight);
            r->width = fWidth; r->height = fHeight; r->pitch = fRowBytes;
            return kIOReturnSuccess;
        }
        if (fn && fn->isEqualTo(NVRM_VRAM_FN_ALLOC)) {
            return vramAlloc((NVRMVramRequest *)p1);
        }
        if (fn && fn->isEqualTo(NVRM_VRAM_FN_FREE)) {
            return vramFree((NVRMVramRequest *)p1);
        }
        if (fn && (fn->isEqualTo("nvFlipToSurfacePure") || fn->isEqualTo(NVRM_FLIP_FN))) {
            struct NVRMFlipRequest *f = (struct NVRMFlipRequest *)p1;
            if (!f || f->version != NVRM_FLIP_ABI_VERSION) return kIOReturnBadArgument;
            const bool home = (f->flags & 1u) != 0;
            uint64_t phys = (uint64_t)fBase;
            if (!home && f->kapiMemory) {
                IOMemoryDescriptor *md = OSDynamicCast(IOMemoryDescriptor, (OSObject *)f->kapiMemory);
                if (md) phys = (uint64_t)md->getPhysicalSegment(0, nullptr, kIOMemoryMapperNone);
                else return kIOReturnBadArgument;
            }
            if (fMmio) {
                volatile uint32_t *r = (volatile uint32_t *)fMmio->getVirtualAddress();
                const uint32_t hubpOff = 0;
                r[0x34c0 + hubpOff + 0x060b] = (uint32_t)(phys >> 32);
                r[0x34c0 + hubpOff + 0x060a] = (uint32_t)phys;
                if (!home && f->pitch >= 4) {
                    r[0x34c0 + hubpOff + 0x0607] = f->pitch / 4;
                } else {
                    r[0x34c0 + hubpOff + 0x0607] = fRowBytes / 4;
                }
            }
            f->flipResult = 0;
            return kIOReturnSuccess;
        }
        if (fn && (fn->isEqualTo(NVRM_GPUVA_FN_HANDLES) || fn->isEqualTo(NVRM_GPUVA_FN_ALLOC) || fn->isEqualTo(NVRM_GPUVA_FN_MAP) ||
                   fn->isEqualTo(NVRM_GPUVA_FN_FREE))) {
            return rmCall(fn, p1);
        }
        return IOFramebuffer::callPlatformFunction(fn, wait, p1, p2, p3, p4);
    }
};

OSDefineMetaClassAndStructors(PhxAMDFB, IOFramebuffer)

static PhxAMDFB *gWbFb;   // the started framebuffer, for debug.phxfb_wbench

static inline uint64_t phxRdmsr(uint32_t msr) {
    uint32_t lo, hi;
    __asm__ __volatile__("rdmsr" : "=a"(lo), "=d"(hi) : "c"(msr));
    return (uint64_t)hi << 32 | lo;
}

static uint64_t phxUsSince(uint64_t t0) { uint64_t ns; absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns); return ns / 1000; }

// Store n bytes from src to dst with movnti (8 bytes at a time) and fence: the copy loop's store side.
static void phxStoreNt(void *dst, const void *src, size_t n) {
    uint64_t *d = (uint64_t *)dst;
    const uint64_t *s = (const uint64_t *)src;
    for (size_t i = 0; i < n / 8; i++) __asm__ __volatile__("movnti %1, %0" : "=m"(d[i]) : "r"(s[i]));
    __asm__ __volatile__("sfence" ::: "memory");
}

void PhxAMDFB::writeBench() {
    // The memory-type MSRs: architectural on AMD64 (APM vol. 2, 7.7 MTRRs and 7.8 PAT; SYSCFG / TOP_MEM / TOM2 in 7.9)
    const uint64_t pat = phxRdmsr(0x277), cap = phxRdmsr(0xFE), def = phxRdmsr(0x2FF), syscfg = phxRdmsr(0xC0010010);
    const uint64_t top = phxRdmsr(0xC001001A), tom2 = phxRdmsr(0xC001001D);
    PFBLOG("wbench: PAT 0x%016llx MTRRcap 0x%llx MTRRdefType 0x%llx SYSCFG 0x%llx TOP_MEM 0x%llx TOM2 0x%llx", pat, cap, def,
           syscfg, top, tom2);
    const uint64_t phys = fAliasPhys;
    const unsigned vcnt = (unsigned)(cap & 0xFF) > 16 ? 16 : (unsigned)(cap & 0xFF);
    for (unsigned i = 0; i < vcnt; i++) {
        const uint64_t b = phxRdmsr(0x200 + 2 * i), m = phxRdmsr(0x201 + 2 * i);
        if (!(m & 0x800)) continue;   // MTRRphysMask valid bit
        const uint64_t mask = m & 0x000FFFFFFFFFF000ull, base = b & 0x000FFFFFFFFFF000ull;
        const bool covers = phys && ((phys & mask) == (base & mask));
        PFBLOG("wbench: MTRR%u base 0x%llx type %llu mask 0x%llx%s", i, base, b & 0xFF, mask, covers ? "  <- covers the scan-out" : "");
    }
    const size_t bytes = (size_t)fRowBytes * fHeight & ~(size_t)63;
    void *buf = bytes ? IOMallocAligned(bytes, 64) : nullptr;
    if (!buf || !fSurfMap) { PFBLOG("wbench: no %s", buf ? "BAR mapping" : "buffer"); if (buf) IOFreeAligned(buf, bytes); return; }
    uint64_t t = mach_absolute_time();
    const volatile uint64_t *bar = (const volatile uint64_t *)fSurfMap->getVirtualAddress();
    uint64_t *b64 = (uint64_t *)buf;
    for (size_t i = 0; i < bytes / 8; i++) b64[i] = bar[i];
    gWbReadUs = (SInt64)phxUsSince(t);
    IOMemoryMap *alias = fAliasMap;
    if (alias) { t = mach_absolute_time(); phxStoreNt((void *)alias->getVirtualAddress(), buf, bytes); gWbAliasUs = (SInt64)phxUsSince(t); }
    t = mach_absolute_time(); phxStoreNt((void *)fSurfMap->getVirtualAddress(), buf, bytes); gWbBarUs = (SInt64)phxUsSince(t);
    gWbWbUs = -1;
    if (phys)
        if (IODeviceMemory *dm = IODeviceMemory::withRange(phys, bytes)) {
            if (IOMemoryMap *m = dm->map(kIOMapCopybackCache)) {
                t = mach_absolute_time(); phxStoreNt((void *)m->getVirtualAddress(), buf, bytes); gWbWbUs = (SInt64)phxUsSince(t);
                m->release();
            }
            dm->release();
        }
    IOFreeAligned(buf, bytes);
    gIopFull = 1;   // a frame copied while the bench ran may have been overwritten with the older pixels
    PFBLOG("wbench: %llu bytes: read via BAR %lld us; store via system-RAM view %lld us, via BAR %lld us, via write-back view %lld us",
           (unsigned long long)bytes, gWbReadUs, alias ? gWbAliasUs : -1ll, gWbBarUs, gWbWbUs);
}

static int phxfbWbenchSysctl SYSCTL_HANDLER_ARGS {
    int v = 0;
    const int err = sysctl_handle_int(oidp, &v, 0, req);
    if (err || !req->newptr) return err;
    if (v != 1) return EINVAL;
    PhxAMDFB *fb = gWbFb;
    if (!fb) return ENXIO;
    fb->writeBench();
    return 0;
}

IOService *gPhxHDMI = nullptr;

class PhxHDMINub : public IOService {
    OSDeclareDefaultStructors(PhxHDMINub)
public:
    bool attach(IOService *provider) override { return IOService::attach(provider); }
};
OSDefineMetaClassAndStructors(PhxHDMINub, IOService)

class PhxHDMIHead : public IOFramebuffer {
    OSDeclareDefaultStructors(PhxHDMIHead)
public:
    PhxAMDFB *fPrimary = nullptr;
    IOPhysicalAddress fBase = 0;
    IOMemoryMap *fMmio = nullptr;
    IOMemoryMap *fSurfMap = nullptr;
    thread_call_t fVblTimer = nullptr;
    uint64_t fVblNext = 0;
    struct Irq { IOFBInterruptProc proc = nullptr; OSObject *target = nullptr; void *ref = nullptr; };
    Irq fVbl[4] = {};
    Irq fConnect = {};

    static void vblFire(thread_call_param_t p0, thread_call_param_t) {
        PhxHDMIHead *me = (PhxHDMIHead *)p0;
        if (!me) return;
        for (const Irq &q : me->fVbl) {
            const IOFBInterruptProc proc = __atomic_load_n(&q.proc, __ATOMIC_ACQUIRE);
            if (proc) proc(q.target, q.ref);
        }
        uint64_t period; nanoseconds_to_absolutetime(16666666ull, &period);
        const uint64_t now = mach_absolute_time();
        me->fVblNext = (me->fVblNext && me->fVblNext + period > now) ? me->fVblNext + period : now + period;
        thread_call_enter_delayed(me->fVblTimer, me->fVblNext);
    }

    void enableHdmiHw() {
        if (!fMmio) return;
        volatile uint32_t *r = (volatile uint32_t *)fMmio->getVirtualAddress();
        r[0x34c0 + 0x218b] = 0x01000000;
        r[0x34c0 + 0x21b1] = 0x00010100;
        r[0x34c0 + 0x21b2] = 0x00000101;
        r[0x34c0 + 0x21d8] = 0x00000011;
        r[0x34c0 + 0x2094 + 0x100] = 0x00010011;
        const uint64_t phys1 = (uint64_t)fBase + 0x01000000ull;
        r[0x34c0 + 0x05c0 + 0x060b] = (uint32_t)(phys1 >> 32);
        r[0x34c0 + 0x05c0 + 0x060a] = (uint32_t)phys1;
        r[0x34c0 + 0x05c0 + 0x0607] = 1920;
    }

    bool start(IOService *provider) override {
        if (!IOFramebuffer::start(provider)) return false;
        setProperty("IOFBDependentID", 0x50485846ull << 32 | 2, 64);
        setProperty("IOFBDependentIndex", 1ull, 32);
        setProperty("IOFBGammaWidth", 8ull, 32);
        setProperty("IOFBGammaCount", 256ull, 32);
        setProperty("DisplayGammaChannels", 3ull, 32);
        setProperty("DisplayGammaEntryCount", 256ull, 32);
        setProperty("DisplayGammaEntrySize", 1ull, 32);
        setProperty("PhxFBWidth", 1920ull, 32);
        setProperty("PhxFBHeight", 1080ull, 32);
        setProperty("PhxFBRefreshMilliHz", 60000ull, 32);

        if (IODeviceMemory *sm = IODeviceMemory::withRange(fBase + 0x01000000ull, 16ull * 1024ull * 1024ull)) {
            fSurfMap = sm->map(kIOMapWriteCombineCache); sm->release();
        }
        if (!fSurfMap) {
            PFBLOG("PhxHDMIHead: failed to map scanout buffer at 0x%llx", (unsigned long long)(fBase + 0x01000000ull));
            return false;
        }
        uint32_t *pix = (uint32_t *)fSurfMap->getVirtualAddress();
        for (size_t i = 0; i < 1920 * 1080; i++) pix[i] = 0xFF102030;

        fVblTimer = thread_call_allocate(vblFire, this);
        if (fVblTimer) {
            uint64_t d; clock_interval_to_deadline(16666, kMicrosecondScale, &d);
            fVblNext = d; thread_call_enter_delayed(fVblTimer, d);
        }

        enableHdmiHw();
        registerService();
        PFBLOG("PhxHDMIHead: published 1920x1080@60Hz at phys 0x%llx", (unsigned long long)(fBase + 0x01000000ull));
        return true;
    }

    IOReturn powerStateWillChangeTo(IOPMPowerFlags, unsigned long, IOService *) override { return IOPMAckImplied; }
    IOReturn powerStateDidChangeTo(IOPMPowerFlags, unsigned long, IOService *) override { return IOPMAckImplied; }

    void stop(IOService *provider) override {
        if (fVblTimer) { thread_call_cancel(fVblTimer); while (!thread_call_free(fVblTimer)) { thread_call_cancel(fVblTimer); IOSleep(5); } fVblTimer = nullptr; }
        OSSafeReleaseNULL(fSurfMap);
        IOFramebuffer::stop(provider);
    }

    bool isConsoleDevice() override { return false; }
    IODeviceMemory *getApertureRange(IOPixelAperture aperture) override {
        if (aperture != kIOFBSystemAperture) return nullptr;
        return IODeviceMemory::withRange(fBase + 0x01000000ull, 1920 * 4 * 1080);
    }
    IODeviceMemory *getVRAMRange() override { return getApertureRange(kIOFBSystemAperture); }
    const char *getPixelFormats() override { return IO32BitDirectPixels "\0"; }
    IOItemCount getDisplayModeCount() override { return 1; }
    IOReturn getDisplayModes(IODisplayModeID *modes) override { if (modes) modes[0] = 1; return kIOReturnSuccess; }
    IOReturn getInformationForDisplayMode(IODisplayModeID mode, IODisplayModeInformation *info) override {
        if (mode != 1 || !info) return kIOReturnBadArgument;
        bzero(info, sizeof(*info));
        info->maxDepthIndex = 0; info->nominalWidth = 1920; info->nominalHeight = 1080;
        info->refreshRate = 60 << 16;
        info->flags = kDisplayModeValidFlag | kDisplayModeSafeFlag | kDisplayModeDefaultFlag | kFramebufferSupportsGammaCorrection;
        return kIOReturnSuccess;
    }
    UInt64 getPixelFormatsForDisplayMode(IODisplayModeID, IOIndex) override { return 0; }
    IOReturn getPixelInformation(IODisplayModeID mode, IOIndex depth, IOPixelAperture aperture, IOPixelInformation *pi) override {
        if (mode != 1 || depth != 0 || aperture != kIOFBSystemAperture || !pi) return kIOReturnUnsupportedMode;
        bzero(pi, sizeof(*pi));
        pi->bytesPerRow = 1920 * 4; pi->bitsPerPixel = 32; pi->pixelType = kIORGBDirectPixels;
        pi->componentCount = 3; pi->bitsPerComponent = 8;
        pi->componentMasks[0] = 0x00FF0000; pi->componentMasks[1] = 0x0000FF00; pi->componentMasks[2] = 0x000000FF;
        strlcpy(pi->pixelFormat, IO32BitDirectPixels, sizeof(pi->pixelFormat));
        pi->activeWidth = 1920; pi->activeHeight = 1080;
        return kIOReturnSuccess;
    }
    IOReturn getCurrentDisplayMode(IODisplayModeID *mode, IOIndex *depth) override { if (mode) *mode = 1; if (depth) *depth = 0; return kIOReturnSuccess; }
    IOReturn getStartupDisplayMode(IODisplayModeID *mode, IOIndex *depth) override { return getCurrentDisplayMode(mode, depth); }
    IOReturn setDisplayMode(IODisplayModeID mode, IOIndex depth) override { return (mode == 1 && depth == 0) ? kIOReturnSuccess : kIOReturnUnsupported; }
    IOReturn getTimingInfoForDisplayMode(IODisplayModeID mode, IOTimingInformation *info) override {
        if (mode != 1 || !info) return kIOReturnUnsupportedMode;
        bzero(info, sizeof(*info));
        info->appleTimingID = kIOTimingIDInvalid;
        return kIOReturnSuccess;
    }
    IOItemCount getConnectionCount() override { return 1; }
    bool hasDDCConnect(IOIndex) override { return true; }
    IOReturn getDDCBlock(IOIndex idx, UInt32 block, IOSelect select, IOOptionBits options, UInt8 *data, IOByteCount *length) override {
        if (idx != 0 || block != 0 || !data || !length) return kIOReturnBadArgument;
        static const uint8_t kHdmiEdid[128] = {
            0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x1e, 0x6d, 0x55, 0x25, 0x01, 0x01, 0x01, 0x01,
            0x01, 0x20, 0x01, 0x03, 0x80, 0x35, 0x1e, 0x78, 0xea, 0xee, 0x95, 0xa3, 0x54, 0x4c, 0x99, 0x26,
            0x0f, 0x50, 0x54, 0xa5, 0x4b, 0x00, 0x71, 0x4f, 0x81, 0x80, 0x81, 0xc0, 0x01, 0x01, 0x01, 0x01,
            0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c,
            0x45, 0x00, 0x10, 0x09, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x30, 0x4b, 0x1e,
            0x50, 0x0f, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x47,
            0x32, 0x35, 0x35, 0x46, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xff,
            0x00, 0x32, 0x30, 0x32, 0x36, 0x31, 0x31, 0x30, 0x39, 0x30, 0x30, 0x31, 0x0a, 0x20, 0x00, 0x68
        };
        const IOByteCount copyLen = *length < 128 ? *length : 128;
        bcopy(kHdmiEdid, data, copyLen);
        *length = copyLen;
        return kIOReturnSuccess;
    }
    IOReturn getAppleSense(IOIndex idx, UInt32 *senseType, UInt32 *primary, UInt32 *extended, UInt32 *displayType) override {
        if (senseType) *senseType = 0;
        if (primary) *primary = 6;
        if (extended) *extended = 0x22;
        if (displayType) *displayType = 20;
        return kIOReturnSuccess;
    }
    IOReturn getAttributeForConnection(IOIndex idx, IOSelect attr, uintptr_t *value) override {
        if (!value) return kIOReturnBadArgument;
        switch (attr) {
        case kConnectionEnable: case kConnectionCheckEnable: *value = 1; return kIOReturnSuccess;
        case kConnectionFlags: *value = 0; return kIOReturnSuccess;
        case kConnectionSupportsHLDDCSense: case kConnectionSupportsLLDDCSense: *value = 1; return kIOReturnSuccess;
        case kConnectionColorModesSupported: case kConnectionColorMode: *value = 0x00000001; return kIOReturnSuccess;
        case kConnectionColorDepthsSupported: case kConnectionControllerColorDepth: case kConnectionControllerDepthsSupported:
            *value = 0x00000002; return kIOReturnSuccess;
        default: return IOFramebuffer::getAttributeForConnection(idx, attr, value);
        }
    }
    IOReturn setAttributeForConnection(IOIndex idx, IOSelect attr, uintptr_t value) override {
        switch (attr) {
        case kConnectionPower: return kIOReturnSuccess;
        case kConnectionColorModesSupported: case kConnectionColorDepthsSupported:
        case kConnectionControllerColorDepth: case kConnectionControllerDepthsSupported: case kConnectionColorMode:
        case kConnectionDisplayFlags: case kConnectionFlags: return kIOReturnSuccess;
        default: return IOFramebuffer::setAttributeForConnection(idx, attr, value);
        }
    }
    IOReturn connectFlags(IOIndex connectIndex, IODisplayModeID mode, IOOptionBits *flags) override {
        if (!flags) return kIOReturnBadArgument;
        *flags = kDisplayModeValidFlag | kDisplayModeSafeFlag | kDisplayModeDefaultFlag;
        return kIOReturnSuccess;
    }
    IOReturn registerForInterruptType(IOSelect type, IOFBInterruptProc proc, OSObject *target, void *ref, void **iref) override {
        if (type == kIOFBVBLInterruptType) {
            for (Irq &q : fVbl) if (!q.proc) {
                q.target = target; q.ref = ref; __atomic_store_n(&q.proc, proc, __ATOMIC_RELEASE);
                if (iref) *iref = &q;
                return kIOReturnSuccess;
            }
            return kIOReturnNoResources;
        }
        if (type == kIOFBConnectInterruptType) { fConnect = { proc, target, ref }; if (iref) *iref = &fConnect; return kIOReturnSuccess; }
        return kIOReturnUnsupported;
    }
    IOReturn unregisterInterrupt(void *iref) override {
        for (Irq &q : fVbl) if (iref == &q) __atomic_store_n(&q.proc, (IOFBInterruptProc)nullptr, __ATOMIC_RELEASE);
        return kIOReturnSuccess;
    }
    IOReturn setInterruptState(void *, UInt32) override { return kIOReturnSuccess; }
    IOReturn setGammaTable(UInt32, UInt32, UInt32, void *) override { return kIOReturnSuccess; }
    IOReturn setGammaTable(UInt32, UInt32, UInt32, void *, bool) override { return kIOReturnSuccess; }
    IOReturn setCLUTWithEntries(IOColorEntry *, UInt32, UInt32, IOOptionBits) override { return kIOReturnSuccess; }
    IOReturn setApertureEnable(IOPixelAperture, IOOptionBits) override { return kIOReturnSuccess; }
    IOReturn setStartupDisplayMode(IODisplayModeID, IOIndex) override { return kIOReturnSuccess; }
    IOReturn setPowerState(unsigned long, IOService *) override { return kIOPMAckImplied; }
    IOReturn setAttribute(IOSelect attr, uintptr_t val) override {
        if (attr == kIOWindowServerActiveAttribute || attr == kIOPowerAttribute) return kIOReturnSuccess;
        return IOFramebuffer::setAttribute(attr, val);
    }
    IOReturn getAttribute(IOSelect attr, uintptr_t *val) override {
        if (attr == kIOWindowServerActiveAttribute) { if (val) *val = 1; return kIOReturnSuccess; }
        return IOFramebuffer::getAttribute(attr, val);
    }
    IOReturn callPlatformFunction(const OSSymbol *fn, bool wait, void *p1, void *p2, void *p3, void *p4) override {
        if (fn && fn->isEqualTo(NVRM_VRAM_FN_SCANOUT)) {
            NVRMVramRequest *r = (NVRMVramRequest *)p1;
            if (!r || r->version != NVRM_VRAM_ABI_VERSION) return kIOReturnBadArgument;
            if (!fSurfMap) return kIOReturnNotReady;
            r->kva = (void *)fSurfMap->getVirtualAddress();
            r->phys = fBase + 0x01000000ull;
            r->actualSize = round_page_64(1920ull * 4 * 1080);
            r->width = 1920; r->height = 1080; r->pitch = 1920 * 4;
            return kIOReturnSuccess;
        }
        if (fPrimary) return fPrimary->callPlatformFunction(fn, wait, p1, p2, p3, p4);
        return IOFramebuffer::callPlatformFunction(fn, wait, p1, p2, p3, p4);
    }
};
OSDefineMetaClassAndStructors(PhxHDMIHead, IOFramebuffer)

static PhxAMDFB *gPhxPrimaryFB = nullptr;
static PhxHDMIHead *gPhxHDMIHead = nullptr;
static PhxHDMINub *gPhxHDMINub = nullptr;

int phxSpawnHdmi(int enable) {
    if (enable) {
        if (gPhxHDMIHead) return 0;
        if (!gPhxPrimaryFB || !gPhxPrimaryFB->fPCI) {
            PFBLOG("phxSpawnHdmi: primary FB or PCI device not ready");
            return -1;
        }

        gPhxHDMINub = new PhxHDMINub;
        if (!gPhxHDMINub) return -1;
        if (!gPhxHDMINub->init()) {
            gPhxHDMINub->release();
            gPhxHDMINub = nullptr;
            return -1;
        }
        if (!gPhxHDMINub->attach(gPhxPrimaryFB->fPCI)) {
            gPhxHDMINub->release();
            gPhxHDMINub = nullptr;
            return -1;
        }
        gPhxHDMINub->registerService();

        gPhxHDMIHead = new PhxHDMIHead;
        if (!gPhxHDMIHead) {
            gPhxHDMINub->detach(gPhxPrimaryFB->fPCI);
            gPhxHDMINub->release();
            gPhxHDMINub = nullptr;
            return -1;
        }
        if (!gPhxHDMIHead->init()) {
            gPhxHDMIHead->release();
            gPhxHDMIHead = nullptr;
            gPhxHDMINub->detach(gPhxPrimaryFB->fPCI);
            gPhxHDMINub->release();
            gPhxHDMINub = nullptr;
            return -1;
        }

        gPhxHDMIHead->fPrimary = gPhxPrimaryFB;
        gPhxHDMIHead->fBase = gPhxPrimaryFB->fBase;
        gPhxHDMIHead->fMmio = gPhxPrimaryFB->fMmio;
        if (gPhxHDMIHead->fMmio) gPhxHDMIHead->fMmio->retain();

        if (!gPhxHDMIHead->attach(gPhxHDMINub)) {
            gPhxHDMIHead->release();
            gPhxHDMIHead = nullptr;
            gPhxHDMINub->detach(gPhxPrimaryFB->fPCI);
            gPhxHDMINub->release();
            gPhxHDMINub = nullptr;
            return -1;
        }

        gPhxHDMI = gPhxHDMIHead;

        if (!gPhxHDMIHead->start(gPhxHDMINub)) {
            gPhxHDMI = nullptr;
            gPhxHDMIHead->detach(gPhxHDMINub);
            gPhxHDMIHead->release();
            gPhxHDMIHead = nullptr;
            gPhxHDMINub->detach(gPhxPrimaryFB->fPCI);
            gPhxHDMINub->release();
            gPhxHDMINub = nullptr;
            PFBLOG("phxSpawnHdmi: PhxHDMIHead start failed");
            return -1;
        }

        PFBLOG("phxSpawnHdmi: PhxHDMIHead published (1920x1080@60Hz)");
        return 0;
    } else {
        if (!gPhxHDMIHead) return 0;
        PFBLOG("phxSpawnHdmi: stopping PhxHDMIHead");
        gPhxHDMI = nullptr;
        gPhxHDMIHead->terminate(kIOServiceRequired | kIOServiceSynchronous);
        gPhxHDMIHead->release();
        gPhxHDMIHead = nullptr;
        if (gPhxHDMINub) {
            gPhxHDMINub->terminate(kIOServiceRequired | kIOServiceSynchronous);
            gPhxHDMINub->release();
            gPhxHDMINub = nullptr;
        }
        return 0;
    }
}

static int phxfb_hdmi_sysctl SYSCTL_HANDLER_ARGS
{
    int v = (gPhxHDMI != nullptr) ? 1 : 0;
    int err = sysctl_handle_int(oidp, &v, 0, req);
    if (err || !req->newptr) return err;
    if (v != 0 && v != 1) return EINVAL;
    int r = phxSpawnHdmi(v);
    return r ? EIO : 0;
}

// The panel's brightness as a display parameter. IOGraphics looks for parameter handlers under the framebuffer, and
// takes a built-in TFT panel for a backlit one (AppleBacklightDisplay) only while a service named "backlight" exists.
class PhxBacklight : public IODisplayParameterHandler {
    OSDeclareDefaultStructors(PhxBacklight)
    PhxAMDFB *fFB = nullptr;
    uint32_t fLevel = kBlMax;
public:
    static PhxBacklight *withFramebuffer(PhxAMDFB *fb) {
        PhxBacklight *b = new PhxBacklight;
        if (b && !b->init()) OSSafeReleaseNULL(b);
        if (b) { b->fFB = fb; fb->backlightRead(&b->fLevel); }
        return b;
    }
    bool setDisplay(IODisplay *display) override {
        if (!display) return true;
        OSObject *p = display->copyProperty(gIODisplayParametersKey);
        OSDictionary *cur = OSDynamicCast(OSDictionary, p);
        OSDictionary *params = cur ? OSDictionary::withDictionary(cur) : OSDictionary::withCapacity(2);
        OSSafeReleaseNULL(p);
        if (!params) return false;
        fFB->backlightRead(&fLevel);
        IODisplay::addParameter(params, gIODisplayBrightnessKey, 0, kBlMax);
        IODisplay::setParameter(params, gIODisplayBrightnessKey, (SInt32)fLevel);
        // CoreBrightness sets the linear ones, as an Intel MacBook's framebuffer offers them: with "brightness" alone
        // the slider moved and no parameter ever reached the panel (2026-10-25)
        const OSSymbol *linear[] = { gIODisplayLinearBrightnessKey, gIODisplayUsableLinearBrightnessKey };
        for (const OSSymbol *k : linear) { IODisplay::addParameter(params, k, 0, kBlMax); IODisplay::setParameter(params, k, (SInt32)fLevel); }
        display->setProperty(gIODisplayParametersKey, params);
        params->release();
        PFBLOG("backlight: brightness parameter on %s, %u of %u", display->getName(), fLevel, kBlMax);
        return true;
    }
    // the last parameter set, in the registry: PhxAMDFB's log does not reach the unified log
    uint32_t fSets = 0;
    bool doIntegerSet(OSDictionary *, const OSSymbol *paramName, UInt32 value) override {
        setProperty("PhxLastParam", const_cast<OSSymbol *>(paramName)); setProperty("PhxLastValue", value, 32); setProperty("PhxSets", ++fSets, 32);
        fFB->trace(paramName->getCStringNoCopy(), 0x20202020, value);
        if (paramName != gIODisplayBrightnessKey && paramName != gIODisplayLinearBrightnessKey
            && paramName != gIODisplayUsableLinearBrightnessKey) return false;
        if (paramName == gIODisplayBrightnessKey && value == 0) {
            fFB->displayPower(false);
            fLevel = 0;
            setProperty("PhxDark", fFB->backlightDark());
            return true;
        } else if (paramName == gIODisplayBrightnessKey && value > 0 && fFB->backlightDark()) {
            fFB->displayPower(true);
        }
        if (!fFB->backlightSet(value)) return false;
        fLevel = value > kBlMax ? kBlMax : value;
        setProperty("PhxDark", fFB->backlightDark());
        return true;
    }
    bool doDataSet(const OSSymbol *, OSData *) override { return false; }
    IOReturn setProperties(OSObject *properties) override {
        OSDictionary *d = OSDynamicCast(OSDictionary, properties);
        if (OSCollectionIterator *it = d ? OSCollectionIterator::withCollection(d) : nullptr) {
            while (const OSSymbol *k = OSDynamicCast(OSSymbol, it->getNextObject())) fFB->trace(k->getCStringNoCopy(), 0x2a2a2a2a, 0);
            it->release();
        }
        return kIOReturnUnsupported;
    }
    bool doUpdate() override { return true; }
};
OSDefineMetaClassAndStructors(PhxBacklight, IODisplayParameterHandler)

static IOService *phxBacklightCreate(PhxAMDFB *fb)
{
    PhxBacklight *b = PhxBacklight::withFramebuffer(fb);
    if (!b) return nullptr;
    b->setName("backlight");
    if (!b->attach(fb)) { b->release(); return nullptr; }
    b->registerService();
    return b;
}

// One boot per arming: `nvram phxfb-boot=1` lets the next start take the panel, and start removes the variable and
// commits NVRAM at once, so a boot that panics after this point comes back on the firmware framebuffer without hands.
static bool takeBootArming()
{
    IORegistryEntry *opts = IORegistryEntry::fromPath("/options", gIODTPlane);
    if (!opts) return false;
    OSObject *o = opts->copyProperty("phxfb-boot");
    const bool armed = o != nullptr;
    bool stuck = false;
    OSSafeReleaseNULL(o);
    if (armed) {
        opts->removeProperty("phxfb-boot");
        // the forced commit: the throttled one (IONVRAM-SYNCNOW) left the variable in flash once, and the boot after a
        // panic took the panel again (2026-10-20)
        if (OSString *why = OSString::withCString("phxfb")) { opts->setProperty("IONVRAM-FORCESYNCNOW-PROPERTY", why); why->release(); }
        o = opts->copyProperty("phxfb-boot");
        stuck = o != nullptr;
        PFBLOG("NVRAM phxfb-boot taken (%s)", stuck ? "STILL SET: removal refused, not taking the panel" : "removed and committed");
        OSSafeReleaseNULL(o);
    }
    opts->release();
    return armed && !stuck;
}

bool PhxAMDFB::start(IOService *provider)
{
    char arg[8];
    if (PE_parse_boot_argn("-phxfboff", arg, sizeof arg)) { PFBLOG("-phxfboff: leaving the firmware framebuffer"); return false; }
    if (PE_parse_boot_argn("-phxoff", arg, sizeof arg)) { PFBLOG("-phxoff: leaving the firmware framebuffer"); return false; }
    if (!takeBootArming()) { PFBLOG("NVRAM phxfb-boot not set: leaving the firmware framebuffer this boot"); return false; }
    fPCI = OSDynamicCast(IOPCIDevice, provider);
    if (!fPCI) return false;
    if (PE_parse_boot_argn("-phxfbint", arg, sizeof arg)) fReportExternal = fReportEdid = false;
    if (OSIterator *it = fPCI->getChildIterator(gIOServicePlane)) {
        while (OSObject *o = it->getNextObject())
            if (o != this && OSDynamicCast(IOFramebuffer, o)) fBeside = true;
        it->release();
    }
    PE_Video v;
    bzero(&v, sizeof(v));
    IOPlatformExpert *pe = getPlatform();
    if (!pe || pe->getConsoleInfo(&v) != kIOReturnSuccess || !v.v_baseAddr || v.v_depth != 32 || !v.v_width || !v.v_height ||
        v.v_rowBytes < v.v_width * 4 || v.v_width > 16384 || v.v_height > 16384) {
        PFBLOG("no usable boot console (base 0x%lx %lux%lu depth %lu row %lu): leaving the firmware framebuffer",
               v.v_baseAddr, v.v_width, v.v_height, v.v_depth, v.v_rowBytes);
        return false;
    }
    fWidth = (uint32_t)v.v_width; fHeight = (uint32_t)v.v_height; fRowBytes = (uint32_t)v.v_rowBytes;
    // once the video console is up, the platform expert hands out the console's kernel mapping, not the physical
    // address. The GOP surface sits in one of the GPU's BARs (VRAM aperture): find the BAR whose start holds the same
    // pixels as the console mapping. (A descriptor over the mapping would need prepare(): getPhysicalSegment on an
    // unwired one panicked, 2026-10-20.)
    uint64_t base = v.v_baseAddr;
    if (base >= 0xFFFF800000000000ull) {
        const volatile uint32_t *con = (const volatile uint32_t *)(uintptr_t)base;
        const uint32_t probeWords = 64u * 1024u / 4u;
        uint64_t found = 0;
        for (uint32_t i = 0; i < fPCI->getDeviceMemoryCount() && !found; i++) {
            IODeviceMemory *dm = fPCI->getDeviceMemoryWithIndex(i);
            if (!dm || dm->getLength() < (IOByteCount)fRowBytes * fHeight) continue;
            IODeviceMemory *sub = IODeviceMemory::withSubRange(dm, 0, 64u * 1024u);
            IOMemoryMap *m = sub ? sub->map(kIOMapInhibitCache) : nullptr;
            OSSafeReleaseNULL(sub);
            if (!m) continue;
            const volatile uint32_t *bar = (const volatile uint32_t *)m->getVirtualAddress();
            uint32_t same = 0, nz = 0;
            for (uint32_t w = 0; w < probeWords; w++) { const uint32_t a = con[w], b = bar[w]; same += a == b; nz += a != 0; }
            PFBLOG("BAR index %u at 0x%llx: %u of %u words match the console (%u non-zero)", i,
                   (unsigned long long)dm->getPhysicalAddress(), same, probeWords, nz);
            if (same == probeWords && nz) found = dm->getPhysicalAddress();
            m->release();
        }
        if (!found) { PFBLOG("the console surface is in none of the GPU's BARs: leaving the firmware framebuffer"); return false; }
        base = found;
    }
    // early in boot the platform expert hands out the physical address with a flag in its low bits (0x820000001): taken
    // as is, every row WindowServer drew landed one byte off and the panel showed swapped colours (2026-10-20)
    base &= ~(uint64_t)PAGE_MASK;
    fBase = (IOPhysicalAddress)base;
    PFBLOG("console: %ux%u row %u depth %lu at physical 0x%llx", fWidth, fHeight, fRowBytes, v.v_depth, (unsigned long long)fBase);
    if (IODeviceMemory *sm = IODeviceMemory::withRange(fBase, (IOPhysicalLength)fRowBytes * fHeight)) {
        fSurfMap = sm->map(kIOMapWriteCombineCache); sm->release();
    }
    if (!fSurfMap) { PFBLOG("the console surface at 0x%llx would not map write-combined", (unsigned long long)fBase); return false; }
    // IOFramebuffer::start makes the framebuffer's controller only from a dependent ID that is already set
    // (IOFBController::copyController(kDepIDCreate)); without one the controller stays NULL and the first power-change
    // callout dereferences it (IOFramebuffer::powerStateWillChangeTo, panic 2026-10-20)
    setProperty("IOFBDependentID", 0x50485846ull << 32 | 1, 64);   // 'PHXF'
    setProperty("IOFBDependentIndex", 0ull, 32);
    // The backlight PWM (upstream c47212c/de42aed) belongs to an eDP panel. A DVI/HDMI pipe (the 740M desktops) has none:
    // no brightness handler there, so IODisplay keeps the external monitor and nothing writes PWRSEQ/PANEL_BLON.
    fMmio = fPCI->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress5);
    if (fMmio && fMmio->getLength() < (kBlLock + kBlSeqs[1] + 1) * 4) OSSafeReleaseNULL(fMmio);
    bool hdmiPipe = false;
    if (fMmio && fMmio->getLength() >= ((IOByteCount)phxdcn::kHighestReg + 1) * 4) {
        volatile uint32_t *mr = (volatile uint32_t *)fMmio->getVirtualAddress();
        hdmiPipe = phxdcn::pipe([mr](uint32_t reg) { return mr[reg]; }).external();
    }
    uint32_t bl = 0;
    if (PE_parse_boot_argn("-phxbloff", arg, sizeof arg)) PFBLOG("backlight: -phxbloff, no brightness control");
    else if (hdmiPipe) PFBLOG("backlight: DVI/HDMI pipe (external monitor), no brightness control");
    else if (!backlightRead(&bl)) PFBLOG("backlight: the PWM is not running (no MMIO view, or off), no brightness control");
    else if (!(fBacklight = phxBacklightCreate(this))) PFBLOG("backlight: the handler would not attach");
    else PFBLOG("backlight: PWM at %u of %u", bl, kBlMax);
    if (!IOFramebuffer::start(provider)) {
        if (fBacklight) { fBacklight->detach(this); OSSafeReleaseNULL(fBacklight); }
        OSSafeReleaseNULL(fMmio); OSSafeReleaseNULL(fSurfMap); return false;
    }
    fVa = IORangeAllocator::withRange(kVaSize - 1, PAGE_SIZE, 0, IORangeAllocator::kLocking);
    poolInit();
    setProperty("IOFBGammaWidth", 8ull, 32);
    setProperty("IOFBGammaCount", 256ull, 32);
    setProperty("DisplayGammaChannels", 3ull, 32);
    setProperty("DisplayGammaEntryCount", 256ull, 32);
    setProperty("DisplayGammaEntrySize", 1ull, 32);
    setProperty("PhxFBWidth", fWidth, 32); setProperty("PhxFBHeight", fHeight, 32);   // the AGDC's panel timing reads these
    measureRefresh();   // also finds the pipe (phxdcn::pipe): built-in panel or external display
    setProperty("built-in", external() ? kOSBooleanFalse : kOSBooleanTrue);
    if (fPipe.dig >= 0) {
        setProperty("PhxFBOtg", (uint32_t)fPipe.otg, 32); setProperty("PhxFBDig", (uint32_t)fPipe.dig, 32);
        setProperty("PhxFBDigMode", fPipe.digMode, 32);
    }
    setProperty("PhxFBRefreshMilliHz", fRefreshMilliHz, 32);
    if (fHTotal) { setProperty("PhxFBHTotal", fHTotal, 32); setProperty("PhxFBVTotal", fVTotal, 32); }   // the AGDC's panel timing
    nanoseconds_to_absolutetime(fFrameNs, &gPhxFrameAbs);
    fVblTimer = thread_call_allocate(vblFire, this);
    if (fVblTimer) { uint64_t d; clock_interval_to_deadline((uint32_t)(fFrameNs / 1000), kMicrosecondScale, &d); fVblNext = d; thread_call_enter_delayed(fVblTimer, d); }
    static IOPMPowerState states[2] = {
        { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, kIOPMDeviceUsable, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0 },
    };
    PMinit();
    registerPowerDriver(this, states, 2);
    phxHoldInit(this, kBootHoldSec);
    phxAgdcInstall();
    if (!gAliasSysctl) { for (auto *o : kFbOids) sysctl_register_oid(o); gAliasSysctl = true; }
    gWbFb = this;
    gPhxPrimaryFB = this;   // debug.phxfb_hdmi (opt-in) spawns its head from a started framebuffer only
    PFBLOG("panel published: %ux%u row %u at 0x%llx (vbl timer %s)", fWidth, fHeight, fRowBytes,
           (unsigned long long)fBase, fVblTimer ? "running" : "MISSING");
    return true;
}

void PhxAMDFB::stop(IOService *provider)
{
    fStopping = true;
    if (gPhxPrimaryFB == this) gPhxPrimaryFB = nullptr;
    if (gPhxHDMI) {
        gPhxHDMI->terminate();
        gPhxHDMI = nullptr;
    }
    phxAgdcRemove();
    if (gAliasSysctl) { for (auto *o : kFbOids) sysctl_unregister_oid(o); gAliasSysctl = false; }
    gWbFb = nullptr;
    if (fBacklight) { fBacklight->terminate(); OSSafeReleaseNULL(fBacklight); }
    if (fWakeGate) { if (IOWorkLoop *wl = getWorkLoop()) wl->removeEventSource(fWakeGate); OSSafeReleaseNULL(fWakeGate); }
    OSSafeReleaseNULL(fAliasMap);
    OSSafeReleaseNULL(fVa);
    poolFree();
    if (fVblTimer) { thread_call_cancel(fVblTimer); while (!thread_call_free(fVblTimer)) { thread_call_cancel(fVblTimer); IOSleep(5); } fVblTimer = nullptr; }
    gPhxFrameAbs = 0;
    OSSafeReleaseNULL(fMmio);
    OSSafeReleaseNULL(fSurfMap);
    PMstop();
    IOFramebuffer::stop(provider);
}
