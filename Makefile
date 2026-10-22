#===============================================================================
# Makefile for I2C/SMBus 3.0 Controller SystemVerilog Project
#
# Targets:
#   sim               - Run standard I2C test (100 kHz)
#   fast-sim          - Run Fast-mode I2C test (400 kHz)
#   smbus-sim         - Run SMBus test (100 kHz)
#   smbus-fast-sim    - Run SMBus Fast-mode test (400 kHz)
#   timeout-sim       - Run SMBus timeout test (100 kHz)
#   timeout-fast-sim  - Run SMBus timeout test (400 kHz)
#   stretch-sim       - Run SMBus stretch+timeout test (100 kHz)
#   stretch-fast-sim  - Run SMBus stretch+timeout test (400 kHz)
#   read-stretch-sim  - Run read-side clock stretch regression
#   read-stretch-fast-sim - Run read-side clock stretch regression (400 kHz)
#   all               - Run all 10 iverilog testbenches
#   veri-sim          - Run Verilator C++ testbench
#   waves             - View waveforms with GTKWave
#   clean             - Remove build artifacts
#   lint              - Run linting (if available)
#   help              - Show available targets
#
# Requirements:
#   - Icarus Verilog (iverilog) for simulation
#   - GTKWave for waveform viewing (optional)
#   - Verilator for C++ testbench (optional)
#===============================================================================

# Directories
RTL_DIR     := rtl
TB_DIR      := tb
BUILD_DIR   := build
VERI_DIR    := verilator
VERI_OBJ    := obj_dir

# RTL files in COMPILATION ORDER (package FIRST, then dependents)
PKG_FILE    := $(RTL_DIR)/i2c_pkg.sv
PEC_FILE    := $(RTL_DIR)/smbus_pec.sv
GLITCH_FILE := $(RTL_DIR)/i2c_glitch_filter.sv
RTL_CORE    := $(RTL_DIR)/i2c_master.sv $(RTL_DIR)/i2c_slave.sv
TOP_FILE    := $(RTL_DIR)/i2c_top.sv
RTL_ORDERED := $(PKG_FILE) $(PEC_FILE) $(GLITCH_FILE) $(RTL_CORE) $(TOP_FILE)

# Compiler flags
VLOG_FLAGS  := -g2012 -Wall -Winfloop
VLOG_FLAGS  += -I $(RTL_DIR) -I $(TB_DIR)

# Simulator command
VLOG        := iverilog
VVP         := vvp

# Waveform viewer
WAVE_VIEWER := gtkwave

# Colors for output
RED     := \033[0;31m
GREEN   := \033[0;32m
YELLOW  := \033[0;33m
CYAN    := \033[0;36m
RESET   := \033[0m

#===============================================================================
# Targets
#===============================================================================

.PHONY: all clean sim fast-sim smbus-sim smbus-fast-sim
.PHONY: timeout-sim timeout-fast-sim stretch-sim stretch-fast-sim read-stretch-sim read-stretch-fast-sim
.PHONY: all-sims veri-sim veri-clean waves lint help

# Create build directory
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# Pattern rule: compile a specific testbench
# Usage: $(BUILD_DIR)/tb_name.vvp  → compiles RTL + that testbench
$(BUILD_DIR)/%.vvp: $(BUILD_DIR) $(RTL_ORDERED) $(TB_DIR)/%.sv
	@echo "$(CYAN)Compiling $*...$(RESET)"
	$(VLOG) $(VLOG_FLAGS) -o $@ $(RTL_ORDERED) $(TB_DIR)/$*.sv
	@echo "$(GREEN)Compilation successful: $@$(RESET)"

#===============================================================================
# Testbench-specific targets
#===============================================================================

# 1. Standard I2C (100 kHz)
sim: $(BUILD_DIR)/tb_i2c.vvp
	@echo "$(CYAN)Running Standard I2C simulation...$(RESET)"
	$(VVP) $<

# 2. Fast-mode I2C (400 kHz)
fast-sim: $(BUILD_DIR)/tb_i2c_fastmode.vvp
	@echo "$(CYAN)Running Fast-mode I2C simulation (400 kHz)...$(RESET)"
	$(VVP) $<

# 3. SMBus (100 kHz)
smbus-sim: $(BUILD_DIR)/tb_i2c_smbus.vvp
	@echo "$(CYAN)Running SMBus simulation (100 kHz)...$(RESET)"
	$(VVP) $<

# 4. SMBus Fast-mode (400 kHz)
smbus-fast-sim: $(BUILD_DIR)/tb_i2c_smbus_fastmode.vvp
	@echo "$(CYAN)Running SMBus Fast-mode simulation (400 kHz)...$(RESET)"
	$(VVP) $<

# 5. SMBus Timeout (100 kHz)
timeout-sim: $(BUILD_DIR)/tb_i2c_smbus_timeout.vvp
	@echo "$(CYAN)Running SMBus Timeout simulation (100 kHz)...$(RESET)"
	$(VVP) $<

# 6. SMBus Timeout Fast-mode (400 kHz)
timeout-fast-sim: $(BUILD_DIR)/tb_i2c_smbus_timeout_fastmode.vvp
	@echo "$(CYAN)Running SMBus Timeout Fast-mode simulation (400 kHz)...$(RESET)"
	$(VVP) $<

# 7. SMBus Stretch+Timeout (100 kHz)
stretch-sim: $(BUILD_DIR)/tb_i2c_smbus_stretch_timeout.vvp
	@echo "$(CYAN)Running SMBus Stretch+Timeout simulation (100 kHz)...$(RESET)"
	$(VVP) $<

# 8. SMBus Stretch+Timeout Fast-mode (400 kHz)
stretch-fast-sim: $(BUILD_DIR)/tb_i2c_smbus_stretch_timeout_fastmode.vvp
	@echo "$(CYAN)Running SMBus Stretch+Timeout Fast-mode simulation (400 kHz)...$(RESET)"
	$(VVP) $<

# 9. Read-side clock stretch regression (byte 2+ of a burst read)
read-stretch-sim: $(BUILD_DIR)/tb_i2c_read_stretch_regression.vvp
	@echo "$(CYAN)Running read-side clock stretch regression...$(RESET)"
	$(VVP) $<

# 10. Read-side clock stretch regression, Fast-mode (400 kHz)
read-stretch-fast-sim: $(BUILD_DIR)/tb_i2c_read_stretch_regression_fastmode.vvp
	@echo "$(CYAN)Running read-side clock stretch regression (400 kHz)...$(RESET)"
	$(VVP) $<

#===============================================================================
# Run all 10 iverilog simulations
#===============================================================================
all: sim fast-sim smbus-sim smbus-fast-sim timeout-sim timeout-fast-sim \
     stretch-sim stretch-fast-sim read-stretch-sim read-stretch-fast-sim
	@echo "$(GREEN)All 10 simulations completed successfully.$(RESET)"

#===============================================================================
# Verilator C++ simulation (no force/release features)
#===============================================================================
veri-sim: $(RTL_ORDERED) $(VERI_DIR)/testbench.cpp
	@echo "$(CYAN)Building Verilator simulation...$(RESET)"
	verilator --cc --exe --build -j 1 --timing \
		--top-module i2c_top_verilator \
		-Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-SYNCASYNCNET \
		-Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-fatal \
		-CFLAGS "-std=c++17 -O0 -g" \
		$(PKG_FILE) $(PEC_FILE) $(GLITCH_FILE) $(RTL_CORE) $(TOP_FILE) \
		$(RTL_DIR)/i2c_top_verilator.sv \
		$(VERI_DIR)/testbench.cpp 2>&1 | tail -10
	@echo "$(GREEN)Verilator build complete. Running simulation...$(RESET)"
	./$(VERI_OBJ)/Vi2c_top_verilator
	@echo "$(GREEN)Verilator simulation complete.$(RESET)"

veri-clean:
	@echo "$(CYAN)Cleaning Verilator artifacts...$(RESET)"
	rm -rf $(VERI_OBJ)
	rm -f $(VERI_DIR)/*.vcd

#===============================================================================
# View waveforms with GTKWave
#===============================================================================
waves: tb_i2c.vcd
	@if command -v $(WAVE_VIEWER) > /dev/null 2>&1; then \
		$(WAVE_VIEWER) $<; \
	else \
		echo "$(YELLOW)GTKWave not found. Install it with:$(RESET)"; \
		echo "  sudo apt-get install gtkwave"; \
		echo "  brew install gtkwave"; \
	fi

#===============================================================================
# Lint (requires Verilator)
#===============================================================================
lint:
	@if command -v verilator > /dev/null 2>&1; then \
		echo "$(CYAN)Running Verilator lint...$(RESET)"; \
		verilator --lint-only -Wall -Wno-fatal \
			-Wno-PINCONNECTEMPTY -Wno-IMPORTSTAR -Wno-TIMESCALEMOD -Wno-VARHIDDEN \
			-I$(RTL_DIR) $(RTL_ORDERED) 2>&1; \
		echo "$(GREEN)Lint check completed.$(RESET)"; \
	else \
		echo "$(YELLOW)Verilator not found. Install it for lint support.$(RESET)"; \
	fi

#===============================================================================
# Clean build artifacts
#===============================================================================
clean:
	@echo "$(CYAN)Cleaning build artifacts...$(RESET)"
	rm -rf $(BUILD_DIR)
	rm -f *.vcd
	rm -f *.lxt
	rm -f *.lxt2
	rm -f *.vpd
	@echo "$(GREEN)Clean complete.$(RESET)"

#===============================================================================
# Help
#===============================================================================
help:
	@echo "I2C/SMBus 3.0 Controller Project - Makefile"
	@echo ""
	@echo "Usage: make <target>"
	@echo ""
	@echo "Targets:"
	@echo "  sim               Standard I2C test (100 kHz)"
	@echo "  fast-sim          Fast-mode I2C test (400 kHz)"
	@echo "  smbus-sim         SMBus test (100 kHz)"
	@echo "  smbus-fast-sim    SMBus Fast-mode test (400 kHz)"
	@echo "  timeout-sim       SMBus timeout test (100 kHz)"
	@echo "  timeout-fast-sim  SMBus timeout test (400 kHz)"
	@echo "  stretch-sim       SMBus stretch+timeout test (100 kHz)"
	@echo "  stretch-fast-sim  SMBus stretch+timeout test (400 kHz)"
	@echo "  read-stretch-sim  Read-side clock stretch regression"
	@echo "  read-stretch-fast-sim  Read-side clock stretch regression (400 kHz)"
	@echo "  all               Run all 9 iverilog simulations"
	@echo "  veri-sim          Run Verilator C++ testbench"
	@echo "  veri-clean        Clean Verilator artifacts"
	@echo "  waves             View waveforms (requires GTKWave)"
	@echo "  lint              Run lint check (requires Verilator)"
	@echo "  clean             Remove build artifacts"
	@echo "  help              Show this help"
