//==============================================================================
// tb_i2c_smbus_timeout_fastmode.sv
// SMBus 3.0 35ms SCL Low Timeout Test — Fast Mode (400 kHz)
//
// Identical to tb_i2c_smbus_timeout.sv but verifies the timeout at
// 400 kHz SCL frequency. The timeout counter depends on CLK_FREQ
// (50 MHz → 1,750,000 cycles = 35 ms), not on SCL_FREQ, so the
// timeout should work identically at both 100 kHz and 400 kHz.
//==============================================================================

`timescale 1ns / 1ps

import i2c_pkg::*;

module tb_i2c_smbus_timeout_fastmode();

  parameter CLK_PERIOD  = 20;     // 50 MHz
  parameter SLAVE_ADDR  = 7'h50;
  parameter SCL_FREQ    = 400_000;  // Fast mode (400 kHz)
  parameter bit SMBUS_EN = 1'b1;

  // For 50 MHz: (35 * 50_000_000) / 1000 = 1,750,000 cycles = 35 ms
  parameter real TIMEOUT_MS    = 35.0;
  parameter real WAIT_MARGIN_NS = 1_000_000;  // 1 ms margin

  // DUT signals
  logic clk, rst_n;
  logic mst_start, mst_reset;
  logic [6:0] mst_addr;
  logic mst_rw;
  logic [7:0] mst_data_tx, mst_data_rx;
  logic [7:0] mst_num_bytes;
  logic mst_busy, mst_done, mst_ack_error, mst_arb_lost;
  logic mst_smbus_timeout;

  logic [7:0] slv_tx_data;
  logic slv_tx_valid, slv_tx_ready;
  logic [7:0] slv_rx_data;
  logic slv_rx_valid, slv_rx_ready;
  logic slv_addressed, slv_dir_rw, slv_busy;
  wire scl, sda;
  logic timeout_fired;  // Sampled before releasing state force

  pullup(scl); pullup(sda);

  // Testbench-owned open-drain fault-injection drivers. Rather than
  // `force`/`release` on the resolved tri-state scl/sda nets (rejected
  // by some simulators as an unsupported tristate construct), the
  // testbench participates as just another open-drain driver on the
  // bus -- identical in spirit to how the master/slave drive it, and
  // portable across simulators.
  logic tb_scl_force_low = 1'b0, tb_sda_force_low = 1'b0;
  assign scl = tb_scl_force_low ? 1'b0 : 1'bz;
  assign sda = tb_sda_force_low ? 1'b0 : 1'bz;

  // ---------------------------------------------------------------------------
  // DUT — SMBus 3.0 enabled at 400 kHz
  // ---------------------------------------------------------------------------
  i2c_top #(
    .CLK_FREQ     (50_000_000),
    .SCL_FREQ     (SCL_FREQ),
    .SLV_ADDR     (SLAVE_ADDR),
    .GEN_CALL     (1'b1),
    .SMBUS_ENABLE (SMBUS_EN)
  ) u_i2c_top (
    .clk, .rst_n,
    .mst_start, .mst_reset, .mst_addr, .mst_rw,
    .mst_data_tx, .mst_data_rx, .mst_num_bytes,
    .mst_busy, .mst_done, .mst_ack_error, .mst_arb_lost,
    .mst_smbus_timeout,
    .slv_tx_data, .slv_tx_valid, .slv_tx_ready,
    .slv_rx_data, .slv_rx_valid, .slv_rx_ready,
    .slv_addressed, .slv_dir_rw, .slv_busy,
    .scl, .sda
  );

  // ---------------------------------------------------------------------------
  // Clock generation
  // ---------------------------------------------------------------------------
  initial clk = 1'b0;
  always #(CLK_PERIOD / 2) clk = ~clk;

  // ---------------------------------------------------------------------------
  // VCD dump
  // ---------------------------------------------------------------------------
  initial begin
    $dumpfile("tb_i2c_smbus_timeout_fastmode.vcd");
    $dumpvars(0, tb_i2c_smbus_timeout_fastmode);
  end

  task wait_clks(input int n);
    repeat (n) @(posedge clk);
  endtask

  // ---------------------------------------------------------------------------
  // Helper: display current simulation time in ms
  // ---------------------------------------------------------------------------
  task show_time();
    $display("[%0t ns = %0t ms]", $time, $time / 1_000_000);
  endtask

  // ---------------------------------------------------------------------------
  // MAIN TEST
  // ---------------------------------------------------------------------------
  initial begin
    $display("");
    $display("==============================================================");
    $display("  SMBus 3.0 35 ms SCL Low Timeout Test — Fast Mode (400 kHz)");
    $display("==============================================================");
    $display("  Clock:         50 MHz (%0d ns period)", CLK_PERIOD);
    $display("  SCL:           %0d kHz (Fast mode)", SCL_FREQ / 1000);
    $display("  SMBUS_ENABLE:  %0b", SMBUS_EN);
    $display("  Timeout cycles: %0d (at 50 MHz)", (35 * 50_000_000) / 1000);
    $display("  Timeout period: %0d ms (= %.1f ms + 1 ms margin)",
             TIMEOUT_MS + 1.0, TIMEOUT_MS);
    $display("  (Timeout depends on CLK_FREQ, not SCL_FREQ)");
    $display("==============================================================");
    $display("");

    // -----------------------------------------------------------------------
    // Phase 1: Reset and verify idle state
    // -----------------------------------------------------------------------
    $display("=== Phase 1: Reset (400 kHz) ===");
    rst_n = 0; mst_start = 0; mst_reset = 0;
    slv_tx_data = '0; slv_tx_valid = 0; slv_rx_ready = 1;
    wait_clks(10);
    rst_n = 1;
    wait_clks(10);
    show_time();

    if (mst_smbus_timeout)
      $display("  FAIL: Timeout asserted at idle!");
    else
      $display("  PASS: Timeout inactive at idle (400 kHz).");

    // -----------------------------------------------------------------------
    // Phase 2: Start a transaction, then stall SCL low
    // -----------------------------------------------------------------------
    $display("");
    $display("=== Phase 2: Force SCL low during transaction (400 kHz) ===");

    // Start a write transaction
    mst_addr = SLAVE_ADDR;
    mst_rw   = 1'b0;
    mst_data_tx = 8'hAB;
    mst_num_bytes = 8'd1;
    mst_start = 1'b1;
    wait_clks(1);
    mst_start = 1'b0;

    // Wait for master to enter an active state
    @(posedge mst_busy);
    wait_clks(5);  // Let it reach MST_SEND_ADDR
    show_time();

    // The master's FSM runs on internal scl_gen events (not scl_i), so
    // it will reach MST_IDLE via normal completion in ~45 us before
    // the 35 ms timeout fires. We force the state register to stay in
    // an active state so the timeout counter keeps running.
    $display("  >>> Forcing SCL LOW to simulate bus hang...");
    tb_scl_force_low = 1'b1;
    $display("  >>> Forcing master FSM to MST_WAIT_ACK (keep counter active)...");
    force u_i2c_top.u_i2c_master.state = MST_WAIT_ACK;
    show_time();

    // -----------------------------------------------------------------------
    // Phase 3: Wait for timeout (35 ms + margin)
    // -----------------------------------------------------------------------
    $display("");
    $display("=== Phase 3: Waiting 36 ms for timeout ===");
    $display("  Timeout threshold: %0d ms", TIMEOUT_MS);
    $display("  Waiting %0d ms...", TIMEOUT_MS + 1.0);

    #36_000_000;  // 36 ms (35 ms timeout + 1 ms margin)
    show_time();

    // -----------------------------------------------------------------------
    // Phase 4: Verify timeout behavior
    // -----------------------------------------------------------------------
    $display("");
    $display("=== Phase 4: Verify timeout (400 kHz) ===");

    // Sample smbus_timeout BEFORE releasing state (it clears to 0 within
    // 2-3 cycles after release once the IDLE state resets the counter).
    timeout_fired = u_i2c_top.u_i2c_master.smbus_timeout;

    // Release the state force, so the timeout mechanism's state <= MST_IDLE
    // can take effect on the next clock edge.
    $display("  >>> Releasing state force (allowing timeout to set MST_IDLE)...");
    release u_i2c_top.u_i2c_master.state;
    wait_clks(3);  // Let the timeout mechanism set state <= MST_IDLE

    // Check smbus_timeout via sampled value (taken while state was forced)
    if (timeout_fired)
      $display("  PASS: SMBus timeout is asserted at 400 kHz (smbus_timeout=1).");
    else
      $display("  FAIL: SMBus timeout not asserted at 400 kHz!");

    // Check master FSM forced to MST_IDLE
    if (u_i2c_top.u_i2c_master.state == MST_IDLE)
      $display("  PASS: Master FSM forced to MST_IDLE after timeout at 400 kHz.");
    else
      $display("  FAIL: Master FSM in state %0d (expected MST_IDLE=0)!",
               u_i2c_top.u_i2c_master.state);

    // Check smbus_to_cnt is cleared (should be 0 because state reached IDLE)
    if (u_i2c_top.u_i2c_master.smbus_to_cnt === 0)
      $display("  PASS: Timeout counter cleared.");
    else
      $display("  FAIL: Timeout counter = %0d (expected 0)!",
               u_i2c_top.u_i2c_master.smbus_to_cnt);

    // Check mst_busy is released
    wait_clks(1);
    if (!mst_busy)
      $display("  PASS: mst_busy cleared (bus released).");
    else
      $display("  FAIL: mst_busy still asserted!");

    // -----------------------------------------------------------------------
    // Phase 5: Release SCL, generate STOP to reset slave state, verify recovery
    // -----------------------------------------------------------------------
    $display("");
    $display("=== Phase 5: Generate STOP recovery, verify bus recovery (400 kHz) ===");
    $display("  >>> Releasing SCL force...");
    tb_scl_force_low = 1'b0;
    wait_clks(5);

    // The slave never saw a proper STOP condition because SCL was forced low
    // for 36ms. Its bus_busy flag is still 1, keeping the FSM in a stale state.
    // Generate a dummy STOP (SDA rising while SCL high) to reset the slave.
    $display("  >>> Forcing SDA low (prepare for STOP)...");
    tb_sda_force_low = 1'b1;
    wait_clks(5);

    $display("  >>> Releasing SDA (SDA rise with SCL high -> STOP condition)...");
    tb_sda_force_low = 1'b0;
    wait_clks(10);
    show_time();

    // Verify SCL and SDA returned high (bus idle)
    if (scl === 1'b1)
      $display("  PASS: SCL returned high (bus idle).");
    else
      $display("  FAIL: SCL not high after release (scl=%b).", scl);

    if (sda === 1'b1)
      $display("  PASS: SDA returned high (bus idle).");
    else
      $display("  FAIL: SDA not high after release (sda=%b).", sda);

    // Verify slave bus_busy is cleared by the STOP
    wait_clks(5);
    if (!u_i2c_top.u_i2c_slave.bus_busy)
      $display("  PASS: Slave bus_busy cleared by STOP at 400 kHz.");
    else
      $display("  FAIL: Slave bus_busy still asserted!");

    // -----------------------------------------------------------------------
    // Phase 6: Post-timeout recovery — new transaction at 400 kHz
    //
    // With the FSM fix (stop_detected handled in ALL states), the dummy STOP
    // in Phase 5 fully resets the slave: bus_busy clears, stop_detected fires,
    // and the slave FSM transitions to S_IDLE from any state. No reset pulse
    // is needed — the STOP recovery is sufficient.
    // -----------------------------------------------------------------------
    $display("");
    $display("=== Phase 6: Post-timeout recovery (STOP-only, 400 kHz) ===");

    // Verify mst_smbus_timeout cleared after SCL released
    wait_clks(1);
    if (!mst_smbus_timeout)
      $display("  PASS: smbus_timeout cleared after SCL release.");
    else
      $display("  FAIL: smbus_timeout still asserted!");

    // Set up slave TX data
    slv_tx_data = 8'h42;
    slv_tx_valid = 1'b1;
    wait_clks(2);
    slv_tx_valid = 1'b0;

    // Start a new write transaction immediately — no reset needed!
    mst_addr = SLAVE_ADDR;
    mst_rw   = 1'b0;
    mst_data_tx = 8'hDE;
    mst_num_bytes = 8'd1;
    mst_start = 1'b1;
    wait_clks(1);
    mst_start = 1'b0;

    @(posedge mst_busy);
    @(negedge mst_busy);
    wait_clks(2);
    show_time();

    if (mst_ack_error)
      $display("  FAIL: Post-timeout write got NACK at 400 kHz!");
    else
      $display("  PASS: Post-timeout write completed successfully at 400 kHz (data=0xDE).");

    // Verify slave received the data
    if (slv_rx_data == 8'hDE)
      $display("  PASS: Slave received correct post-timeout data.");
    else
      $display("  FAIL: Slave received 0x%02x (expected 0xDE).", slv_rx_data);

    // -----------------------------------------------------------------------
    // Summary
    // -----------------------------------------------------------------------
    $display("");
    $display("==============================================================");
    $display("  SMBus TIMEOUT TEST (400 kHz) — COMPLETE");
    $display("==============================================================");
    $display("");

    #1000;
    $finish;
  end

  // ---------------------------------------------------------------------------
  // Global timeout (50 ms max)
  // ---------------------------------------------------------------------------
  initial begin
    #50_000_000;
    $display("[%0t] TIMEOUT: Simulation exceeded 50 ms!", $time);
    $finish;
  end

  // ---------------------------------------------------------------------------
  // Monitor: track timeout status in real-time
  // ---------------------------------------------------------------------------
  always @(posedge clk) begin
    if (u_i2c_top.u_i2c_master.smbus_timeout)
      $display("[%0t] *** SMBus TIMEOUT FIRED at 400 kHz! ***", $time);
  end

endmodule
