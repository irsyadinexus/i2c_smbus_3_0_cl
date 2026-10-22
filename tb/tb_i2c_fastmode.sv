`timescale 1ns / 1ps

import i2c_pkg::*;

module tb_i2c_fastmode();
  parameter SCL_FREQ = 400000;
  parameter SLAVE_ADDR = 7'h50;
  parameter CLK_PERIOD = 20;

  logic clk, rst_n;
  logic mst_start, mst_reset;
  logic [6:0] mst_addr;
  logic mst_rw;
  logic [7:0] mst_data_tx, mst_data_rx;
  logic [7:0] mst_num_bytes;
  logic mst_busy, mst_done, mst_ack_error, mst_arb_lost;
  logic [7:0] slv_tx_data;
  logic slv_tx_valid, slv_tx_ready;
  logic [7:0] slv_rx_data;
  logic slv_rx_valid, slv_rx_ready;
  logic slv_addressed, slv_dir_rw, slv_busy;
  wire scl, sda;

  pullup(scl); pullup(sda);

  i2c_top #(
    .CLK_FREQ (50_000_000),
    .SCL_FREQ (SCL_FREQ),
    .SLV_ADDR (SLAVE_ADDR),
    .GEN_CALL (1'b1)
  ) u_i2c_top (
    .clk, .rst_n, .mst_start, .mst_reset, .mst_addr, .mst_rw,
    .mst_data_tx, .mst_data_rx, .mst_num_bytes,
    .mst_busy, .mst_done, .mst_ack_error, .mst_arb_lost,
    .slv_tx_data, .slv_tx_valid, .slv_tx_ready,
    .slv_rx_data, .slv_rx_valid, .slv_rx_ready,
    .slv_addressed, .slv_dir_rw, .slv_busy,
    .scl, .sda
  );

  initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD / 2) clk = ~clk;
  end

  initial begin
    $dumpfile("tb_i2c_fastmode.vcd");
    $dumpvars(0, tb_i2c_fastmode);
  end

  task wait_clks(int n); repeat(n) @(posedge clk); endtask

  initial begin
    $display("");
    $display("========================================================");
    $display("  I2C Fast Mode (400 kHz) Test");
    $display("========================================================");
    $display("");

    rst_n = 0; mst_start = 0; mst_reset = 0; slv_tx_data = '0;
    slv_tx_valid = 0; slv_rx_ready = 1;
    wait_clks(10);
    rst_n = 1;
    wait_clks(10);

    // Write transaction
    $display("[%0t] Starting write to 0x%02x...", $time, SLAVE_ADDR);
    mst_addr = SLAVE_ADDR;
    mst_rw = 0;
    mst_data_tx = 8'hAB;
    mst_num_bytes = 8'd1;
    mst_start = 1;
    wait_clks(1);
    mst_start = 0;
    wait(mst_done || mst_ack_error);
    wait_clks(2);
    
    if (mst_ack_error)
      $display("[%0t] FAIL: NACK received!", $time);
    else
      $display("[%0t] PASS: Write complete. SCL=%0d kHz", $time, SCL_FREQ/1000);

    wait_clks(20);
    $display("[%0t] Simulation complete.", $time);
    $finish;
  end

  initial begin
    #500_000;
    $display("[%0t] TIMEOUT", $time);
    $finish;
  end
endmodule
