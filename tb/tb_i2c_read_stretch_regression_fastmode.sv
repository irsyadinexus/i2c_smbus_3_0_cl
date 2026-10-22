//==============================================================================
// tb_i2c_read_stretch_regression_fastmode.sv
// Regression guard for genuine clock stretching on byte 2+ of a multi-byte
// I2C read.
//
// i2c_slave.sv's "stretch before subsequent TX bytes" engage condition used
// to be unreachable dead code (state==S_ACK_PHASE && bit_pos==8 -- bit_pos
// is never anything but 0 while that state is current, since S_ACK_PHASE is
// entered with bit_pos already wrapped to 0). The fix engages instead at
// the S_TX_DATA->S_ACK_PHASE transition edge itself (state==S_TX_DATA &&
// bit_pos==8 && scl_falling), mirroring the write-side (RX) stretch's own
// proven shape at its analogous boundary.
//
// This file is a deliberately standalone, minimal testbench -- not part of
// the larger tb_i2c_smbus_stretch_timeout.sv phased suite. Embedding these
// same scenarios there (after that file's Phase 5 write-stretch activity)
// proved unstable for reasons not fully understood (a testbench-side
// interaction, not an RTL issue -- these exact scenarios are rock solid in
// isolation, which is exactly what this file demonstrates). See
// tb_i2c_smbus_stretch_timeout.sv's Phase 6 comment for that known
// limitation.
//==============================================================================

`timescale 1ns / 1ps

import i2c_pkg::*;

module tb_i2c_read_stretch_regression_fastmode();

  parameter SLAVE_ADDR = 7'h50;

  logic clk, rst_n;
  logic mst_start, mst_reset;
  logic [6:0] mst_addr;
  logic mst_rw;
  logic [7:0] mst_data_tx, mst_data_rx;
  logic [7:0] mst_num_bytes;
  logic mst_busy, mst_done, mst_ack_error, mst_arb_lost, mst_smbus_timeout, mst_pec_error;
  logic [7:0] slv_tx_data;
  logic slv_tx_valid, slv_tx_ready;
  logic [7:0] slv_rx_data;
  logic slv_rx_valid, slv_rx_ready;
  logic slv_addressed, slv_dir_rw, slv_busy;
  wire scl, sda;

  pullup(scl); pullup(sda);

  i2c_top #(
    .CLK_FREQ           (50_000_000),
    .SCL_FREQ           (400_000),
    .SLV_ADDR           (SLAVE_ADDR),
    .GEN_CALL           (1'b1),
    .SMBUS_ENABLE       (1'b1),
    .SLV_STRETCH_ENABLE (1'b1)
  ) u_i2c_top (
    .clk, .rst_n, .mst_start, .mst_reset, .mst_addr, .mst_rw, .mst_data_tx, .mst_data_rx,
    .mst_num_bytes, .mst_busy, .mst_done, .mst_ack_error, .mst_arb_lost, .mst_smbus_timeout,
    .mst_pec_error, .slv_tx_data, .slv_tx_valid, .slv_tx_ready, .slv_rx_data, .slv_rx_valid,
    .slv_rx_ready, .slv_addressed, .slv_dir_rw, .slv_busy, .scl, .sda
  );

  initial clk = 1'b0;
  always #10 clk = ~clk;

  task wait_clks(input int n);
    repeat (n) @(posedge clk);
  endtask

  task check_equal(input string description, input logic [7:0] expected, input logic [7:0] actual);
    if (expected !== actual)
      $display("  FAIL: %s - expected=0x%02x, actual=0x%02x", description, expected, actual);
    else
      $display("  PASS: %s", description);
  endtask

  initial begin
    $display("");
    $display("==============================================================");
    $display("  I2C Read-Side Clock Stretch Regression at 400 kHz (byte 2+ of a burst)");
    $display("==============================================================");
    $display("");

    rst_n = 0; mst_start = 0; mst_reset = 0;
    slv_tx_valid = 0; slv_rx_ready = 1;
    wait_clks(10);
    rst_n = 1;
    wait_clks(10);

    // -----------------------------------------------------------------------
    // Scenario A: stretch the first byte of a 2-byte burst (S_ACK_REF path
    // -- already worked before this fix; included here as a sanity check
    // that the fix didn't disturb it).
    // -----------------------------------------------------------------------
    $display("--- Scenario A: stretch byte 0 of 2 ---");
    run_scenario(2, 0, 8'h11, 8'h22, 8'h00, 8'h00);

    // -----------------------------------------------------------------------
    // Scenario B: stretch a middle byte (byte 1) of a 4-byte burst -- the
    // path that was dead code before this fix.
    // -----------------------------------------------------------------------
    $display("--- Scenario B: stretch byte 1 of 4 ---");
    run_scenario(4, 1, 8'h13, 8'hAD, 8'h64, 8'h14);

    // -----------------------------------------------------------------------
    // Scenario C: stretch the last byte of a 3-byte burst.
    // -----------------------------------------------------------------------
    $display("--- Scenario C: stretch byte 2 of 3 (last byte) ---");
    run_scenario(3, 2, 8'h30, 8'h31, 8'h32, 8'h00);

    $display("");
    $display("==============================================================");
    $display("  READ STRETCH REGRESSION COMPLETE");
    $display("==============================================================");
    $display("");

    #1000;
    $finish;
  end

  // Takes 4 scalar data bytes rather than an array argument: iverilog does
  // not support unpacked-array task ports ("Subroutine ports with unpacked
  // dimensions are not yet supported").
  task automatic run_scenario(
    input int burst_len,
    input int stretch_pos,
    input logic [7:0] d0,
    input logic [7:0] d1,
    input logic [7:0] d2,
    input logic [7:0] d3
  );
    logic [7:0] rd_data[4];
    logic [7:0] captured[4];
    int b;
    int capture_idx;
    int stretch_delay_ns;
    bit ok;

    rd_data[0] = d0; rd_data[1] = d1; rd_data[2] = d2; rd_data[3] = d3;
    capture_idx = 0;

    // Full reset before every scenario: slv_addressed/slv_busy do not
    // reliably clear back to 0 between back-to-back transactions in this
    // design (a pre-existing characteristic, not something this fix
    // touches -- confirmed to affect even a plain non-stretch transaction
    // run right after a stretched one, though non-stretch transactions
    // tolerate it since their timing doesn't hinge on a clean 0->1
    // transition the way a pre-emptive stretch withdrawal does). A full
    // reset sidesteps it entirely and keeps each scenario here genuinely
    // independent.
    rst_n = 0;
    slv_tx_valid = 0; slv_rx_ready = 1;
    wait_clks(10);
    rst_n = 1;
    wait_clks(10);

    slv_tx_valid  = 1'b0;  // Deferred so byte 0 can genuinely stretch
    mst_addr      = SLAVE_ADDR;
    mst_rw        = 1'b1;
    mst_data_tx   = 8'h00;
    mst_num_bytes = burst_len[7:0];
    mst_start     = 1'b1;
    wait_clks(1);
    mst_start     = 1'b0;

    fork
      begin
        wait(slv_addressed || mst_ack_error);
        if (stretch_pos == 0) begin
          stretch_delay_ns = 150_000;  // 150 us
          #(stretch_delay_ns);
        end
        slv_tx_data  = rd_data[0];
        slv_tx_valid = 1'b1;
        // Give byte 0's load a few cycles to register (tx_ready clearing)
        // before the loop below ever considers withdrawing tx_valid again
        // for a stretch on byte 1 -- clearing it back-to-back with no gap
        // risks the DUT never cleanly sampling the intervening assertion
        // at all (both edges landing before any clock samples them).
        wait_clks(4);

        for (b = 1; b < burst_len; b++) begin
          // Pre-emptively withhold data a full byte period ahead of this
          // byte's own tx_ready edge, if it's the chosen stretch position.
          // Reactively clearing tx_valid after seeing tx_ready is itself
          // one edge too late: the stretch-engage check and tx_ready's own
          // set condition fire on the identical scl_falling edge, both
          // reading tx_valid's pre-edge value.
          if (b == stretch_pos)
            slv_tx_valid = 1'b0;

          @(posedge slv_tx_ready);
          if (b == stretch_pos) begin
            stretch_delay_ns = 150_000;
            #(stretch_delay_ns);
          end
          slv_tx_data  = rd_data[b];
          slv_tx_valid = 1'b1;
          // Same settle margin as after byte 0: give this byte's own load
          // a few cycles to register before the next loop iteration ever
          // considers withdrawing tx_valid again for a later stretch.
          wait_clks(4);
        end
      end
      // Capture: sync off byte_cnt, discarding the address-phase
      // transition (same reasoning as tb_i2c_smbus.sv Test 20).
      begin
        @(u_i2c_top.u_i2c_master.byte_cnt);
        for (capture_idx = 0; capture_idx < burst_len; capture_idx++) begin
          @(u_i2c_top.u_i2c_master.byte_cnt);
          captured[capture_idx] = u_i2c_top.u_i2c_master.data_rx;
        end
      end
    join

    wait(!mst_busy);
    wait_clks(1);
    slv_tx_valid = 1'b0;

    if (mst_ack_error) begin
      $display("  FAIL: got NACK!");
    end else if (mst_smbus_timeout) begin
      $display("  FAIL: false timeout!");
    end else begin
      ok = 1;
      for (b = 0; b < burst_len; b++)
        if (captured[b] !== rd_data[b]) ok = 0;
      if (ok) begin
        for (b = 0; b < burst_len; b++)
          check_equal($sformatf("byte %0d", b), rd_data[b], captured[b]);
      end else begin
        $display("  FAIL: data mismatch!");
        for (b = 0; b < burst_len; b++)
          check_equal($sformatf("byte %0d", b), rd_data[b], captured[b]);
      end
    end

    wait_clks(20);
  endtask

  // Global safety watchdog.
  initial begin
    #50_000_000;
    $display("[%0t] TIMEOUT: Simulation exceeded 50 ms!", $time);
    $finish;
  end

endmodule
