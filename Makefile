# Makefile for AMD RDNA 3 (Radeon 780M) macOS Driver Project

CXX ?= clang++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -O2
INCLUDES = -Isrc -Isrc/amd -Isrc/uapi -Isrc/winsys

TEST_BIN_DIR = build/tests
TESTS = $(TEST_BIN_DIR)/test_rdna3_discovery_uma \
        $(TEST_BIN_DIR)/test_rdna3_ip_dcn \
        $(TEST_BIN_DIR)/test_rdna3_ucode \
        $(TEST_BIN_DIR)/test_rdna3_chip_profile \
        $(TEST_BIN_DIR)/test_rdna3_fw_plan \
        $(TEST_BIN_DIR)/test_phx_dcn_pipe \
        $(TEST_BIN_DIR)/test_phx_dcn_i2c \
        $(TEST_BIN_DIR)/test_phx_dcn_timing \
        $(TEST_BIN_DIR)/test_phx_dmub_layout \
        $(TEST_BIN_DIR)/test_phx_dmub_vbios_cmd \
        $(TEST_BIN_DIR)/test_phx_dcn314_plan \
        $(TEST_BIN_DIR)/test_phx_dcn314_step2 \
        $(TEST_BIN_DIR)/test_phx_dcn314_clk \
        $(TEST_BIN_DIR)/test_phx_dmub_inbox \
        $(TEST_BIN_DIR)/test_phx_dcn314_blank \
        $(TEST_BIN_DIR)/test_phx_dcn314_dpms \
        $(TEST_BIN_DIR)/test_phx_dcn314_modeset \
        $(TEST_BIN_DIR)/test_phx_hdmi_avi \
        $(TEST_BIN_DIR)/test_r3n_abi \
        $(TEST_BIN_DIR)/test_phx_topology \
        $(TEST_BIN_DIR)/test_wait_seq \
        $(TEST_BIN_DIR)/test_phx_gpu_queue \
        $(TEST_BIN_DIR)/test_phx_pending_fences \
        $(TEST_BIN_DIR)/test_hdmi_modeset_validation

.PHONY: all test clean kext fixtures

all: test

$(TEST_BIN_DIR):
	mkdir -p $(TEST_BIN_DIR)

$(TEST_BIN_DIR)/test_hdmi_modeset_validation: tests/test_hdmi_modeset_validation.cpp $(wildcard src/amd/phx_*.h) | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@

$(TEST_BIN_DIR)/test_phx_pending_fences: tests/test_phx_pending_fences.cpp src/phxbringup/PhxPendingFences.h | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@

$(TEST_BIN_DIR)/test_phx_gpu_queue: tests/test_phx_gpu_queue.cpp src/phxbringup/PhxGpuQueue.h | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@

$(TEST_BIN_DIR)/test_wait_seq: tests/test_wait_seq.cpp | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ -lpthread

$(TEST_BIN_DIR)/test_phx_topology: tests/test_phx_topology.cpp | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(TEST_BIN_DIR)/test_rdna3_discovery_uma: src/amd/rdna3_ip_discovery.cpp src/amd/rdna3_rebar_uma.cpp tests/test_rdna3_discovery_uma.cpp | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(TEST_BIN_DIR)/test_rdna3_ip_dcn: src/amd/rdna3_ip_blocks.cpp src/amd/rdna3_dcn314_scanout.cpp src/amd/rdna3_rebar_uma.cpp tests/test_rdna3_ip_dcn.cpp | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(TEST_BIN_DIR)/test_rdna3_ucode: src/amd/rdna3_ucode_extract.cpp tests/test_rdna3_ucode.cpp | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(TEST_BIN_DIR)/test_rdna3_chip_profile: src/amd/rdna3_ip_discovery.cpp src/amd/rdna3_chip_profile.cpp tests/test_rdna3_chip_profile.cpp | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(TEST_BIN_DIR)/test_rdna3_fw_plan: src/amd/rdna3_ucode_extract.cpp src/amd/rdna3_fw_plan.cpp tests/test_rdna3_fw_plan.cpp | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(TEST_BIN_DIR)/test_phx_dcn_pipe: tests/test_phx_dcn_pipe.cpp src/phxaccel/accel/phx_dcn_pipe.h | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) -Isrc/phxaccel/accel tests/test_phx_dcn_pipe.cpp -o $@

$(TEST_BIN_DIR)/test_phx_dcn_i2c: tests/test_phx_dcn_i2c.cpp src/amd/phx_dcn_i2c.h | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) -Isrc/amd tests/test_phx_dcn_i2c.cpp -o $@

$(TEST_BIN_DIR)/test_phx_dcn_timing: tests/test_phx_dcn_timing.cpp src/amd/phx_dcn_timing.h | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) -Isrc/amd tests/test_phx_dcn_timing.cpp -o $@

$(TEST_BIN_DIR)/test_phx_dmub_layout: tests/test_phx_dmub_layout.cpp src/amd/phx_dmub_layout.h | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) -Isrc/amd tests/test_phx_dmub_layout.cpp -o $@

$(TEST_BIN_DIR)/test_phx_dmub_vbios_cmd: tests/test_phx_dmub_vbios_cmd.cpp src/amd/phx_dmub_vbios_cmd.h | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) -Isrc/amd tests/test_phx_dmub_vbios_cmd.cpp -o $@

$(TEST_BIN_DIR)/test_phx_dcn314_plan: tests/test_phx_dcn314_plan.cpp src/amd/phx_dcn314_plan.h src/amd/phx_dcn314_prog_regs.h src/amd/phx_dml314_plans.h | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) -Isrc/amd tests/test_phx_dcn314_plan.cpp -o $@

$(TEST_BIN_DIR)/test_phx_dcn314_step2: tests/test_phx_dcn314_step2.cpp src/amd/phx_dcn314_step2.h src/amd/phx_dcn314_plan.h src/amd/phx_dcn314_prog_regs.h src/amd/phx_dml314_plans.h | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) -Isrc/amd tests/test_phx_dcn314_step2.cpp -o $@

$(TEST_BIN_DIR)/test_phx_dcn314_clk: tests/test_phx_dcn314_clk.cpp src/amd/phx_dcn314_clk.h src/amd/phx_dcn314_step2.h src/amd/phx_dcn314_prog_regs.h src/amd/phx_dml314_plans.h | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) -Isrc/amd tests/test_phx_dcn314_clk.cpp -o $@

$(TEST_BIN_DIR)/test_phx_dmub_inbox: tests/test_phx_dmub_inbox.cpp src/amd/phx_dmub_inbox.h src/amd/phx_dmub_vbios_cmd.h src/amd/phx_dcn314_prog_regs.h | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) -Isrc/amd tests/test_phx_dmub_inbox.cpp -o $@

$(TEST_BIN_DIR)/test_phx_dcn314_blank: tests/test_phx_dcn314_blank.cpp src/amd/phx_dcn314_blank.h src/amd/phx_dcn314_step2.h src/amd/phx_dcn314_prog_regs.h | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) -Isrc/amd tests/test_phx_dcn314_blank.cpp -o $@

$(TEST_BIN_DIR)/test_phx_dcn314_dpms: tests/test_phx_dcn314_dpms.cpp src/amd/phx_dcn314_dpms.h src/amd/phx_dmub_inbox.h src/amd/phx_dmub_vbios_cmd.h src/amd/phx_dcn314_step2.h src/amd/phx_dcn314_prog_regs.h | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) -Isrc/amd tests/test_phx_dcn314_dpms.cpp -o $@

$(TEST_BIN_DIR)/test_phx_dcn314_modeset: tests/test_phx_dcn314_modeset.cpp src/amd/phx_dcn314_modeset.h src/amd/phx_dcn314_otg.h src/amd/phx_dcn314_dpms.h src/amd/phx_dcn314_blank.h src/amd/phx_dcn314_step2.h src/amd/phx_dcn_timing.h src/amd/phx_dmub_inbox.h src/amd/phx_dcn314_prog_regs.h | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) -Isrc/amd tests/test_phx_dcn314_modeset.cpp -o $@

$(TEST_BIN_DIR)/test_phx_hdmi_avi: tests/test_phx_hdmi_avi.cpp src/amd/phx_hdmi_avi.h | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) -Isrc/amd tests/test_phx_hdmi_avi.cpp -o $@

$(TEST_BIN_DIR)/test_r3n_abi: src/uapi/r3n_user_client.cpp src/winsys/radv_r3n_winsys.cpp tests/test_r3n_abi.cpp | $(TEST_BIN_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

test: $(TESTS)
	python3 tests/test_native_bo_lifetime.py
	python3 tests/test_native_stop.py
	python3 tests/test_native_vm.py
	python3 tests/test_native_sysmem.py
	python3 tests/test_native_submit.py
	python3 tests/test_install_safety.py
	python3 tests/test_release_pci_matches.py
	python3 tests/test_ue200watch.py
	python3 tests/test_phxboot_watch.py
	python3 tests/test_render_encoding_failure.py
	python3 tests/test_render_load_failure.py
	python3 tests/test_format_view_clear.py
	python3 tests/test_render_surface_clear.py
	python3 tests/test_scanout_dump.py
	python3 tests/test_phxctl_cli.py
	python3 tests/test_bringup_failures.py
	@echo "=== Running RDNA 3 Driver Unit Tests ==="
	@for t in $(TESTS); do \
		echo "Running $$t..."; \
		$$t || exit 1; \
	done
	@echo "All unit tests passed successfully!"

fixtures:
	bash tools/fetch_test_firmware.sh tests/fixtures/phoenix

kext:
	@echo "Building PhxBringup.kext..."
	python3 tools/build_kext.py

clean:
	rm -rf build/
