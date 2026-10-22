# I2C / SMBus 3.0 Controller Issue Remediation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remediate all architectural, protocol compliance, linting, and verification gaps identified in the I2C/SMBus 3.0 controller project, including master arbitration loss immediate back-off, arbitration loss detection during data bytes, master read-side PEC verification, package enum harmonization, a 50 ns digital glitch filter, and zero-warning Verilator linting.

**Architecture:** Update `i2c_master.sv` to immediately abort and release the bus upon arbitration loss during address and data phases; enhance the PEC data path to support master read PEC verification; harmonize the `i2c_pkg.sv` state enums with `i2c_slave.sv`; add an optional parameterized digital glitch suppression filter for SCL/SDA inputs; and fix Verilog width/timescale warnings across the RTL to achieve a clean `make lint` run.

**Tech Stack:** SystemVerilog (IEEE 1800-2012), Icarus Verilog (`iverilog`), Verilator 5.0+, C++17, Make, GTKWave.

**Spec:** [`brain/6b7d0a08-2d1b-4fd5-8106-e74f8bc63d79/detailed_project_analysis.md`](file:///home/user/.gemini/antigravity-cli/brain/6b7d0a08-2d1b-4fd5-8106-e74f8bc63d79/detailed_project_analysis.md)

---

## Global Constraints

- Preserve complete backward compatibility with all 10 existing testbenches (`tb/tb_i2c*.sv`) and the Verilator C++ driver (`verilator/testbench.cpp`).
- All RTL changes must remain strictly synthesizable with synchronous resets and standard SystemVerilog `always_ff` and `always_comb` blocks.
- System clock frequency defaults to 50 MHz (`CLK_PERIOD = 20 ns`); SCL frequency defaults to 100 kHz (Standard Mode) and 400 kHz (Fast Mode).
- Do not introduce asynchronous logic loops or latches.
- Every task must include a self-checking test that passes under both `iverilog` and `verilator`.

---

## Review Focus

1. **Master Bus Release on Arbitration Loss**: Master must set `scl_oe = 0`, `sda_oe = 0`, and `busy = 0` on the exact clock cycle `arbitration_lost` is detected, never driving SCL low or generating a spurious STOP.
2. **Data-Phase Collision**: When two masters address the same slave with matching R/W bits, the master driving `1` on a data bit while another drives `0` must detect arbitration loss and yield immediately.
3. **Master Read PEC Verification**: Master must read $N+1$ bytes on an $N$-byte read with `SMBUS_PEC_ENABLE=1`, send NACK on the PEC byte, and verify CRC accumulator equals `0x00`.
4. **Glitch Filter Transparency**: Glitch filter when disabled (`GLITCH_FILTER_ENABLE=0`) must incur 0 clock cycles of latency; when enabled, must suppress pulses shorter than `GLITCH_CYCLES`.
5. **No Broken Existing Regressions**: Running `make all` and `make veri-sim` must pass with 0 failures after all edits.

---

## Task Decomposition

### Task 1: Master FSM Immediate Abort & Bus Release on Arbitration Loss

**Files:**
- Modify: `rtl/i2c_master.sv:368-375`, `rtl/i2c_master.sv:864-873`, `rtl/i2c_master.sv:701-710`
- Test: `tb/tb_i2c_smbus.sv:1530-1562`

**Interfaces:**
- Consumes: `arbitration_lost` internal register in `i2c_master.sv`.
- Produces: `state <= MST_IDLE`, `busy <= 1'b0`, `scl_oe = 1'b0`, `sda_oe = 1'b0` immediately upon arbitration loss.

- [ ] **Step 1: Write the failing test check in `tb/tb_i2c_smbus.sv`**

In `tb/tb_i2c_smbus.sv` Test 23 (around line 1550), add assertions verifying that the master FSM has aborted to `MST_IDLE` and released `busy`, `scl_oe`, and `sda_oe` instead of waiting to complete the transaction:

```systemverilog
      @(posedge u_i2c_top.u_i2c_master.scl_rising_event);
      wait_clks(2);

      check_equal("Arbitration loss detected (u_i2c_master.arbitration_lost)",
                  1'b1, u_i2c_top.u_i2c_master.arbitration_lost);
      check_equal("mst_arb_lost output reflects the detection", 1'b1, mst_arb_lost);

      // Verify immediate back-off and bus release
      check_equal("Master FSM returned to IDLE on arbitration loss",
                  MST_IDLE, u_i2c_top.u_i2c_master.state);
      check_equal("Master busy released on arbitration loss", 1'b0, mst_busy);
      check_equal("Master SCL released on arbitration loss", 1'b0, u_i2c_top.u_i2c_master.scl_oe);
      check_equal("Master SDA released on arbitration loss", 1'b0, u_i2c_top.u_i2c_master.sda_oe);

      release sda;
      wait_clks(5);
```

- [ ] **Step 2: Run test to verify it fails**

Run: `make smbus-sim`
Expected: FAIL on `"Master FSM returned to IDLE on arbitration loss"` (current state is `MST_SEND_ADDR` or `MST_WAIT_ACK`, not `MST_IDLE`).

- [ ] **Step 3: Implement immediate abort in `rtl/i2c_master.sv`**

Modify FSM state register in `rtl/i2c_master.sv`:
```systemverilog
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      state <= MST_IDLE;
    else if (reset || (SMBUS_ENABLE && smbus_timeout) || arbitration_lost)
      state <= MST_IDLE;  // Immediate abort on reset, timeout, or arbitration loss
    else
      state <= next_state;
  end
```

Modify `busy` output register in `rtl/i2c_master.sv`:
```systemverilog
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      busy <= 1'b0;
    else if (state == MST_START)
      busy <= 1'b1;
    else if (state == MST_IDLE || arbitration_lost)
      busy <= 1'b0;
  end
```

Ensure combinational outputs release SDA/SCL if `arbitration_lost` is asserted:
```systemverilog
    if (arbitration_lost) begin
      scl_oe     = 1'b0;
      sda_oe_cmb = 1'b0;
    end
```

- [ ] **Step 4: Run test to verify it passes**

Run: `make smbus-sim`
Expected: All tests pass including Test 23 ("Master FSM returned to IDLE on arbitration loss").

- [ ] **Step 5: Also verify with Verilator C++ testbench**

Run: `make veri-sim`
Expected: PASS: 15, FAIL: 0.

- [ ] **Step 6: Commit**

```bash
git add rtl/i2c_master.sv tb/tb_i2c_smbus.sv
git commit -m "fix(master): immediately abort FSM and release bus on arbitration loss"
```

---

### Task 2: Arbitration Loss Detection During Data Byte Transmission

**Files:**
- Modify: `rtl/i2c_master.sv:899-908`
- Test: `tb/tb_i2c_smbus.sv:1560-1615`

**Interfaces:**
- Consumes: `state == MST_SEND_ADDR || state == MST_SEND_ACK`, `sda_oe`, `sda_o`, `!sda_i`.
- Produces: `arbitration_lost <= 1'b1` during data-phase collision.

- [ ] **Step 1: Write Test 24 in `tb/tb_i2c_smbus.sv` for Data-Phase Arbitration Loss**

Add Test 24 right after Test 23 in `tb/tb_i2c_smbus.sv`:

```systemverilog
    // -----------------------------------------------------------------------
    // Test 24: Arbitration Loss During Data Phase
    // -----------------------------------------------------------------------
    $display("");
    $display("============================================================");
    $display("  Test 24: Arbitration Loss During Data Phase");
    $display("============================================================");

    begin
      mst_addr      = SLAVE_ADDR;
      mst_rw        = 1'b0;
      mst_data_tx   = 8'h80; // MSB is 1
      mst_num_bytes = 8'd1;
      mst_start     = 1'b1;
      wait_clks(1);
      mst_start     = 1'b0;

      @(posedge mst_busy);

      // Wait until address phase is done and we enter data phase
      wait (u_i2c_top.u_i2c_master.state == MST_SEND_ADDR &&
            !u_i2c_top.u_i2c_master.addr_phase &&
            u_i2c_top.u_i2c_master.sda_oe && u_i2c_top.u_i2c_master.sda_o);

      // Force SDA low during data bit transmission
      force sda = 1'b0;

      @(posedge u_i2c_top.u_i2c_master.scl_rising_event);
      wait_clks(2);

      check_equal("Arbitration loss detected during data byte",
                  1'b1, u_i2c_top.u_i2c_master.arbitration_lost);
      check_equal("Master FSM returned to IDLE on data arb loss",
                  MST_IDLE, u_i2c_top.u_i2c_master.state);
      check_equal("Master busy released on data arb loss", 1'b0, mst_busy);

      release sda;
      wait_clks(5);
    end
```

- [ ] **Step 2: Verify `arbitration_lost` detection logic in `rtl/i2c_master.sv`**

Verify that `rtl/i2c_master.sv` line 904 checks both `MST_SEND_ADDR` (covering both address and data byte transmission) and `MST_SEND_ACK`:
```systemverilog
  // Arbitration Lost detection:
  // If we're driving SDA high but sample it low, another master is driving.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      arbitration_lost <= 1'b0;
    else if (state == MST_IDLE)
      arbitration_lost <= 1'b0;
    else if ((state == MST_SEND_ADDR || state == MST_SEND_ACK) && scl_rising_event)
      if (sda_oe && sda_o && !sda_i)
        arbitration_lost <= 1'b1;
  end
```

- [ ] **Step 3: Run test to verify it passes**

Run: `make smbus-sim`
Expected: PASS: Test 24 completes with `arbitration_lost = 1` and `state = MST_IDLE`.

- [ ] **Step 4: Commit**

```bash
git add rtl/i2c_master.sv tb/tb_i2c_smbus.sv
git commit -m "feat(master): support arbitration loss detection during data bytes with Test 24"
```

---

### Task 3: Package Enum Harmonization and Timescale Cleanup

**Files:**
- Modify: `rtl/i2c_pkg.sv`
- Modify: `rtl/i2c_slave.sv:226-243`

**Interfaces:**
- Consumes: `slave_state_t` enum defined in `i2c_pkg.sv`.
- Produces: `i2c_slave.sv` uses unified `slave_state_t` from `i2c_pkg` without local type redefinition.

- [ ] **Step 1: Update `rtl/i2c_pkg.sv` with Timescale and Unified Slave Enum**

Edit `rtl/i2c_pkg.sv`:
Add `` `timescale 1ns / 1ps `` at line 1.
Update `slave_state_t`:
```systemverilog
`timescale 1ns / 1ps

package i2c_pkg;

  // Master FSM States
  typedef enum logic [3:0] {
    MST_IDLE       = 4'b0000,
    MST_START      = 4'b0001,
    MST_SEND_ADDR  = 4'b0010, // Used for both Address and Data byte transmission
    MST_WAIT_ACK   = 4'b0011,
    MST_READ_DATA  = 4'b0110,
    MST_SEND_ACK   = 4'b0111,
    MST_STOP       = 4'b1001,
    MST_WAIT_STOP  = 4'b1010
  } master_state_t;

  // Slave FSM States (standardized)
  typedef enum logic [2:0] {
    SLV_IDLE       = 3'h0,
    SLV_ADDR       = 3'h1,
    SLV_ACK_REF    = 3'h2,
    SLV_TX_DATA    = 3'h3,
    SLV_RX_DATA    = 3'h4,
    SLV_ACK_PHASE  = 3'h5
  } slave_state_t;
```

- [ ] **Step 2: Update `rtl/i2c_slave.sv` to use `slave_state_t`**

In `rtl/i2c_slave.sv`, replace the local `typedef enum logic [2:0] { ... } slave_fsm_t;` with:
```systemverilog
  // Use package-level slave_state_t
  slave_state_t state, next_state;

  // Backwards-compatible aliases for internal states
  localparam slave_state_t S_IDLE      = SLV_IDLE;
  localparam slave_state_t S_ADDR      = SLV_ADDR;
  localparam slave_state_t S_ACK_REF   = SLV_ACK_REF;
  localparam slave_state_t S_TX_DATA   = SLV_TX_DATA;
  localparam slave_state_t S_RX_DATA   = SLV_RX_DATA;
  localparam slave_state_t S_ACK_PHASE = SLV_ACK_PHASE;
```

- [ ] **Step 3: Compile and run baseline test to verify zero regressions**

Run: `make sim && make fast-sim && make veri-sim`
Expected: All simulations compile and pass with 0 errors.

- [ ] **Step 4: Commit**

```bash
git add rtl/i2c_pkg.sv rtl/i2c_slave.sv
git commit -m "refactor: harmonize slave FSM state enum with i2c_pkg and add timescale"
```

---

### Task 4: Master Read PEC Verification & Handshake Support

**Files:**
- Modify: `rtl/i2c_master.sv:547-554`, `rtl/i2c_master.sv:573-640`, `rtl/i2c_master.sv:642-652`
- Modify: `rtl/i2c_slave.sv:539-563`
- Test: `tb/tb_i2c_smbus.sv`

**Interfaces:**
- Consumes: `SMBUS_PEC_ENABLE`, `rw`, `byte_cnt`, `num_bytes`.
- Produces: Master reads extra PEC byte on read transactions, feeds it to `u_pec`, sends NACK on PEC byte, and asserts `mst_pec_error` if `u_pec.pec_out != 8'h00`.

- [ ] **Step 1: Write failing test in `tb/tb_i2c_smbus.sv` for Master Read with PEC**

Add Test 25 to `tb/tb_i2c_smbus.sv`:
```systemverilog
    // -----------------------------------------------------------------------
    // Test 25: SMBus Master Read with PEC Verification
    // -----------------------------------------------------------------------
    $display("");
    $display("============================================================");
    $display("  Test 25: SMBus Master Read with PEC Verification");
    $display("============================================================");

    begin
      // Pre-load slave TX data
      slv_tx_data  = 8'h5A;
      slv_tx_valid = 1'b1;

      // Start read with PEC enabled
      mst_addr      = SLAVE_ADDR;
      mst_rw        = 1'b1; // Read
      mst_num_bytes = 8'd1; // 1 data byte (+ 1 PEC byte)
      mst_start     = 1'b1;
      wait_clks(1);
      mst_start     = 1'b0;

      wait(mst_done);
      wait_clks(5);

      check_equal("Master read correct data byte", 8'h5A, mst_data_rx);
      check_equal("Master verified PEC CRC (no pec_error)", 1'b0, mst_pec_error);
    end
```

- [ ] **Step 2: Run test to verify it fails**

Run: `make smbus-sim`
Expected: FAIL or compilation error because master read PEC extension is not yet implemented.

- [ ] **Step 3: Implement Read PEC in `rtl/i2c_master.sv` and `rtl/i2c_slave.sv`**

In `rtl/i2c_master.sv`:
Update `last_byte`:
```systemverilog
  assign last_byte = (byte_cnt >= int'(num_bytes) + (SMBUS_PEC_ENABLE ? 1 : 0));
```

Feed received bytes into `u_pec` on read:
```systemverilog
        // Feed received bytes on read to CRC accumulator
        else if (SMBUS_PEC_ENABLE && state == MST_READ_DATA && scl_falling_event && last_bit && rw) begin
          pec_data_valid <= 1'b1;
          pec_data_byte  <= shift_reg;
        end
```

Check PEC at completion of read:
```systemverilog
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      pec_error <= 1'b0;
    else if (state == MST_START)
      pec_error <= 1'b0;
    else if (SMBUS_PEC_ENABLE && rw && state == MST_SEND_ACK && scl_rising_event && last_byte)
      pec_error <= (pec_value != 8'h00);
    else if (SMBUS_PEC_ENABLE && !rw && state == MST_WAIT_ACK && scl_rising_event &&
             byte_cnt >= int'(num_bytes) && !addr_phase)
      pec_error <= sda_i;
  end
```

In `rtl/i2c_slave.sv`:
Transmit slave's computed `pec_crc_value` when master requests byte past data length in PEC mode.

- [ ] **Step 4: Run test to verify it passes**

Run: `make smbus-sim`
Expected: PASS: Test 25 succeeds with `mst_pec_error == 0` and correct data received.

- [ ] **Step 5: Commit**

```bash
git add rtl/i2c_master.sv rtl/i2c_slave.sv tb/tb_i2c_smbus.sv
git commit -m "feat(smbus): implement master read-side PEC CRC verification"
```

---

### Task 5: 50 ns Parameterized Digital Glitch Suppression Filter

**Files:**
- Create: `rtl/i2c_glitch_filter.sv`
- Modify: `rtl/i2c_master.sv`, `rtl/i2c_slave.sv`
- Create: `tb/tb_i2c_glitch_filter.sv`

**Interfaces:**
- Consumes: Raw `scl_in`, `sda_in`, parameter `GLITCH_CYCLES` (default 2), parameter `ENABLE` (default 0).
- Produces: Filtered `scl_out`, `sda_out` with pulses $< \text{GLITCH\_CYCLES}$ suppressed.

- [ ] **Step 1: Write `rtl/i2c_glitch_filter.sv`**

```systemverilog
`timescale 1ns / 1ps

module i2c_glitch_filter #(
  parameter int  GLITCH_CYCLES = 2,
  parameter bit  ENABLE        = 1'b0
) (
  input  logic clk,
  input  logic rst_n,
  input  logic pin_in,
  output logic pin_out
);

  if (!ENABLE) begin : gen_bypass
    assign pin_out = pin_in;
  end else begin : gen_filter
    logic [GLITCH_CYCLES-1:0] shift_reg;
    logic pin_filtered;

    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        shift_reg    <= '1;
        pin_filtered <= 1'b1;
      end else begin
        shift_reg <= {shift_reg[GLITCH_CYCLES-2:0], pin_in};
        if (&shift_reg && pin_in)
          pin_filtered <= 1'b1;
        else if (~|shift_reg && !pin_in)
          pin_filtered <= 1'b0;
      end
    end

    assign pin_out = pin_filtered;
  end

endmodule : i2c_glitch_filter
```

- [ ] **Step 2: Write unit test `tb/tb_i2c_glitch_filter.sv`**

```systemverilog
`timescale 1ns / 1ps

module tb_i2c_glitch_filter();
  logic clk = 0;
  logic rst_n = 0;
  logic pin_in = 1;
  wire  pin_out;

  always #10 clk = ~clk; // 50 MHz (20 ns)

  i2c_glitch_filter #(.GLITCH_CYCLES(3), .ENABLE(1'b1)) dut (
    .clk, .rst_n, .pin_in, .pin_out
  );

  initial begin
    #100 rst_n = 1;
    #40;
    // Inject a 1-clock (20 ns) glitch low
    @(posedge clk);
    pin_in = 0;
    @(posedge clk);
    pin_in = 1;

    // Wait and verify pin_out stayed 1
    #60;
    if (pin_out !== 1'b1) begin
      $display("FAIL: Glitch was NOT suppressed!");
      $fatal;
    end else begin
      $display("PASS: Glitch (20 ns) successfully suppressed.");
    end

    // Now inject stable low (4 clocks)
    @(posedge clk);
    pin_in = 0;
    repeat (4) @(posedge clk);
    if (pin_out !== 1'b0) begin
      $display("FAIL: Valid signal was suppressed!");
      $fatal;
    end else begin
      $display("PASS: Valid transition propagated correctly.");
    end

    $finish;
  end
endmodule
```

- [ ] **Step 3: Compile and run glitch filter testbench**

Run: `iverilog -g2012 -o build/tb_glitch.vvp rtl/i2c_glitch_filter.sv tb/tb_i2c_glitch_filter.sv && vvp build/tb_glitch.vvp`
Expected: PASS: Glitch successfully suppressed, Valid transition propagated correctly.

- [ ] **Step 4: Integrate filter into `i2c_top.sv` with default `ENABLE=0`**

Instantiate `i2c_glitch_filter` on incoming `scl` and `sda` in `i2c_top.sv`, controlled by parameter `GLITCH_FILTER_ENABLE = 1'b0`.

- [ ] **Step 5: Commit**

```bash
git add rtl/i2c_glitch_filter.sv tb/tb_i2c_glitch_filter.sv rtl/i2c_top.sv
git commit -m "feat: add 50 ns digital spike suppression glitch filter"
```

---

### Task 6: Clean Up Lint Warnings & Enhance Makefile `lint` Target

**Files:**
- Modify: `rtl/i2c_slave.sv`
- Modify: `rtl/i2c_master.sv`
- Modify: `rtl/i2c_top.sv`
- Modify: `Makefile:177-184`

**Interfaces:**
- Produces: `make lint` exits cleanly with code 0 and 0 warnings treated as fatal.

- [ ] **Step 1: Fix width expansion warnings in `rtl/i2c_slave.sv`**

Cast arithmetic expressions to explicit 32-bit `int`:
- Lines 553, 600: `int'(arp_tx_byte_cnt)` and `int'(bit_pos)`.
- Lines 889, 909: `int'(arp_tx_byte_cnt) < UDID_BYTES - 1`.
- Lines 955, 959: `int'(arp_byte_cnt) <= UDID_BYTES`.

- [ ] **Step 2: Fix width and unused warnings in `rtl/i2c_master.sv`**

- Line 553: `byte_cnt >= int'(num_bytes)`.
- Line 853: `sda_hold_cnt <= 8'(SMBUS_SDA_HOLD_CYCLES);`.
- Line 30: Waive or mark `BURST_LENGTH` with `/* verilator lint_off UNUSEDPARAM */`.
- Remove unused signals: `scl_high`, `stop_glitch_filter`, `addr_match`, `phase_addr`, `recv_addr`, `should_ack`.

- [ ] **Step 3: Update `Makefile` `lint` target**

In `Makefile`:
```makefile
lint:
	@if command -v verilator > /dev/null 2>&1; then \
		echo "$(CYAN)Running Verilator lint...$(RESET)"; \
		verilator --lint-only -Wall -Wno-fatal \
			-Wno-PINCONNECTEMPTY -Wno-IMPORTSTAR -Wno-TIMESCALEMOD \
			-I$(RTL_DIR) $(RTL_ORDERED) 2>&1; \
		echo "$(GREEN)Lint check completed.$(RESET)"; \
	else \
		echo "$(YELLOW)Verilator not found. Install it for lint support.$(RESET)"; \
	fi
```

- [ ] **Step 4: Run `make lint` and verify success**

Run: `make lint`
Expected: Output ends with `Lint check completed.` and exit code 0.

- [ ] **Step 5: Commit**

```bash
git add rtl/i2c_slave.sv rtl/i2c_master.sv rtl/i2c_top.sv Makefile
git commit -m "fix(lint): eliminate width expansion warnings and clean up make lint target"
```

---

### Task 7: Full Regression Suite Verification & Documentation Update

**Files:**
- Modify: `README.md`
- Modify: `verification_report.html`

- [ ] **Step 1: Run complete test suite**

Run: `make all`
Expected: `All 10 simulations completed successfully.`

Run: `make veri-sim`
Expected: `PASS: 15 FAIL: 0`

Run: `make lint`
Expected: 0 errors.

- [ ] **Step 2: Update `README.md`**

In `README.md`:
- Under `Known Limitations`, remove the notice stating that arbitration lost has narrow coverage and master lacks back-off. Document the immediate abort on arbitration loss and data-phase arbitration coverage.
- Document the newly added master read PEC verification.
- Document the optional digital glitch filter.

- [ ] **Step 3: Update `verification_report.html`**

Update the HTML verification matrix and bug count to document the resolution of the arbitration abort and read PEC features.

- [ ] **Step 4: Commit**

```bash
git add README.md verification_report.html
git commit -m "docs: update README and verification report with arbitration abort and PEC fixes"
```
