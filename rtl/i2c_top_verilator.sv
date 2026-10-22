//==============================================================================
// i2c_top_verilator.sv — Verilator-compatible I2C Controller
//
// This module is identical in function to i2c_top but uses logic ports for
// scl/sda instead of inout. The open-drain bus with pullup is modeled using
// combinational logic that reads the master/slave output enables and values
// directly, computing the bus state as: LOW if any device drives low, HIGH
// otherwise (pullup effect). This avoids Verilator's unreliable inout port
// resolution for Z-state open-drain signaling.
//
// Usage (Verilator builds only — for iverilog use the original i2c_top):
//   i2c_top_verilator #(...) u_i2c (...);
//     .scl_o  - SCL bus state (0=low, 1=high with pullup)
//     .sda_o  - SDA bus state (0=low, 1=high with pullup)
//==============================================================================

`timescale 1ns / 1ps

import i2c_pkg::*;

module i2c_top_verilator #(
  parameter int CLK_FREQ          = 50_000_000,
  parameter int SCL_FREQ          = 100_000,
  parameter int MST_BURST_LEN     = 1,
  parameter bit  SMBUS_ENABLE     = 1'b0,
  parameter bit  SMBUS_PEC_ENABLE = 1'b0,
  parameter logic [6:0] SLV_ADDR  = 7'h50,
  parameter bit         GEN_CALL  = 1'b0,
  parameter bit         SMBALERT_ENABLE = 1'b0,
  parameter bit         SMARP_ENABLE    = 1'b0,
  parameter logic [127:0] UDID = 128'h0,
  parameter bit         SLV_STRETCH_ENABLE = 1'b0
) (
  input  logic       clk,
  input  logic       rst_n,

  // Testbench-only fault injection: drives the shared bus low exactly as
  // a second, competing master would. Verilator's C++ flow has no
  // force/release, so this is the equivalent of the iverilog testbenches'
  // "force sda = 1'b0" used for arbitration-loss testing.
  input  logic       tb_sda_force_low,

  // Master Host Interface
  input  logic       mst_start,
  input  logic       mst_reset,
  input  logic [6:0] mst_addr,
  input  logic       mst_rw,
  input  logic [7:0] mst_data_tx,
  output logic [7:0] mst_data_rx,
  input  logic [7:0] mst_num_bytes,
  output logic       mst_busy,
  output logic       mst_done,
  output logic       mst_ack_error,
  output logic       mst_arb_lost,
  output logic       mst_smbus_timeout,
  output logic       mst_pec_error,

  // Slave Host Interface
  input  logic [7:0] slv_tx_data,
  input  logic       slv_tx_valid,
  output logic       slv_tx_ready,
  output logic [7:0] slv_rx_data,
  output logic       slv_rx_valid,
  input  logic       slv_rx_ready,
  output logic       slv_addressed,
  output logic       slv_dir_rw,
  output logic       slv_busy,

  // I2C Bus (logic outputs — Verilator-safe)
  output logic       scl_o,       // 0 = driven low, 1 = high (pullup)
  output logic       sda_o,       // 0 = driven low, 1 = high (pullup)
  output logic       smbalert_n_o // 0 = asserted, 1 = released
);

  // ---------------------------------------------------------------------------
  // Internal I2C Bus Signals
  // ---------------------------------------------------------------------------
  logic scl_o_mst,  scl_oe_mst;
  logic sda_o_mst,  sda_oe_mst;
  logic scl_i_mst,  sda_i_mst;

  logic sda_o_slv,  sda_oe_slv;
  logic scl_o_slv,  scl_oe_slv;
  logic slv_smbalert_n_o_int, slv_smbalert_n_oe;

  // ---------------------------------------------------------------------------
  // I2C Bus State Computation (Verilator-safe — pure combinational, no inout)
  //
  // In real I2C, the bus is open-drain with pullup resistors: any device can
  // drive the line LOW (0), or release it (high-Z → pullup brings it HIGH).
  //
  // Our model: if ANY device has oe=1 and drives o=0, the bus is 0.
  // Otherwise, the pullup brings it to 1.
  // ---------------------------------------------------------------------------
  logic scl_driven_low, sda_driven_low;

  always_comb begin
    scl_driven_low = (scl_oe_mst && !scl_o_mst) || (scl_oe_slv && !scl_o_slv);
    sda_driven_low = (sda_oe_mst && !sda_o_mst) || (sda_oe_slv && !sda_o_slv) ||
                     tb_sda_force_low;
    scl_o = scl_driven_low ? 1'b0 : 1'b1;   // Pullup: Z → 1
    sda_o = sda_driven_low ? 1'b0 : 1'b1;   // Pullup: Z → 1
  end

  // Master reads the resolved bus state
  assign scl_i_mst = scl_o;
  assign sda_i_mst = sda_o;

  // ---------------------------------------------------------------------------
  // I2C Master Instance (reads bus state from scl_o/sda_o via comb assign)
  // ---------------------------------------------------------------------------
  i2c_master #(
    .CLK_FREQ         (CLK_FREQ),
    .SCL_FREQ         (SCL_FREQ),
    .BURST_LENGTH     (MST_BURST_LEN),
    .SMBUS_ENABLE     (SMBUS_ENABLE),
    .SMBUS_PEC_ENABLE (SMBUS_PEC_ENABLE)
  ) u_i2c_master (
    .clk, .rst_n,
    .start           (mst_start),
    .reset           (mst_reset),
    .addr            (mst_addr),
    .rw              (mst_rw),
    .data_tx         (mst_data_tx),
    .data_rx         (mst_data_rx),
    .num_bytes       (mst_num_bytes),
    .busy            (mst_busy),
    .done            (mst_done),
    .ack_error       (mst_ack_error),
    .arbitration_lost(mst_arb_lost),
    .smbus_timeout   (mst_smbus_timeout),
    .pec_error       (mst_pec_error),
    .bus_busy        ('0),
    .scl_o           (scl_o_mst),
    .scl_oe          (scl_oe_mst),
    .scl_i           (scl_i_mst),
    .sda_o           (sda_o_mst),
    .sda_oe          (sda_oe_mst),
    .sda_i           (sda_i_mst)
  );

  // ---------------------------------------------------------------------------
  // I2C Slave Instance (reads bus state from scl_o/sda_o directly)
  // ---------------------------------------------------------------------------
  i2c_slave #(
    .SLAVE_ADDR       (SLV_ADDR),
    .GEN_CALL         (GEN_CALL),
    .SMBUS_PEC_ENABLE (SMBUS_PEC_ENABLE),
    .SMBALERT_ENABLE  (SMBALERT_ENABLE),
    .SMARP_ENABLE     (SMARP_ENABLE),
    .UDID             (UDID),
    .SLV_STRETCH_ENABLE (SLV_STRETCH_ENABLE)
  ) u_i2c_slave (
    .clk, .rst_n,
    .tx_data          (slv_tx_data),
    .tx_valid         (slv_tx_valid),
    .tx_ready         (slv_tx_ready),
    .rx_data          (slv_rx_data),
    .rx_valid         (slv_rx_valid),
    .rx_ready         (slv_rx_ready),
    .addressed        (slv_addressed),
    .dir_rw           (slv_dir_rw),
    .busy             (slv_busy),
    .gen_call_detected(),
    .start_detected   (),
    .stop_detected    (),
    .pec_error        (),
    .alert_pending    (),
    .scl_i            (scl_o),       // Reads resolved bus state
    .scl_o_slv        (scl_o_slv),
    .scl_oe_slv       (scl_oe_slv),
    .sda_o            (sda_o_slv),
    .sda_oe           (sda_oe_slv),
    .sda_i            (sda_o),       // Reads resolved bus state
    .smbalert_n_o     (slv_smbalert_n_o_int),
    .smbalert_n_oe    (slv_smbalert_n_oe)
  );

  // SMBALERT#: open-drain, active low. Released line = high (pullup).
  assign smbalert_n_o = (slv_smbalert_n_oe && !slv_smbalert_n_o_int) ? 1'b0 : 1'b1;

endmodule : i2c_top_verilator
