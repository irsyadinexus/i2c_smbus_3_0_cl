//==============================================================================
// i2c_top.sv
// Top-Level I2C Controller
//
// Integrates the I2C Master and Slave controllers into a single module
// with bidirectional I2C bus management (SCL, SDA with pull-up simulation).
//
// This module connects to external I2C bus via tri-state pads.
// The host interface provides independent access to both master and slave
// sub-modules.
//==============================================================================

`timescale 1ns / 1ps

import i2c_pkg::*;

module i2c_top #(
  // I2C Master parameters
  parameter int CLK_FREQ        = 50_000_000,
  parameter int SCL_FREQ        = 100_000,
  parameter int MST_BURST_LEN   = 1,
  parameter bit  SMBUS_ENABLE   = 1'b0,  // SMBus 3.0 compatibility
  parameter bit  SMBUS_PEC_ENABLE = 1'b0,  // SMBus PEC CRC-8
  // I2C Slave parameters
  parameter logic [6:0] SLV_ADDR      = 7'h50,
  parameter bit         GEN_CALL      = 1'b0,
  parameter bit         SMBALERT_ENABLE = 1'b0,
  // SMBus 3.0 ARP (Address Resolution Protocol)
  parameter bit         SMARP_ENABLE    = 1'b0,
  parameter logic [127:0] UDID = 128'h00000000_00000000_00000000_00000000,
  // Slave clock stretching
  parameter bit         SLV_STRETCH_ENABLE = 1'b0
) (
  // ---------------------------------------------------------------------------
  // System Interface
  // ---------------------------------------------------------------------------
  input  logic       clk,
  input  logic       rst_n,

  // ---------------------------------------------------------------------------
  // Master Host Interface
  // ---------------------------------------------------------------------------
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

  // ---------------------------------------------------------------------------
  // Slave Host Interface
  // ---------------------------------------------------------------------------
  input  logic [7:0] slv_tx_data,
  input  logic       slv_tx_valid,
  output logic       slv_tx_ready,
  output logic [7:0] slv_rx_data,
  output logic       slv_rx_valid,
  input  logic       slv_rx_ready,
  output logic       slv_addressed,
  output logic       slv_dir_rw,
  output logic       slv_busy,

  // ---------------------------------------------------------------------------
  // I2C Bus (direct connection to bidirectional pads)
  // ---------------------------------------------------------------------------
  inout  wire        scl,
  inout  wire        sda,

  // SMBALERT# open-drain interrupt (optional)
  output wire        slv_smbalert_n
);

  // ---------------------------------------------------------------------------
  // Internal I2C Bus Signals
  // ---------------------------------------------------------------------------
  logic scl_o_mst,  scl_oe_mst;
  logic sda_o_mst,  sda_oe_mst;
  logic scl_i_mst,  sda_i_mst;

  logic sda_o_slv,  sda_oe_slv;
  logic scl_o_slv,  scl_oe_slv;
  logic slv_smbalert_n_o, slv_smbalert_n_oe;

  logic scl_i, sda_i;

  // ---------------------------------------------------------------------------
  // Bidirectional Pad Buffers (inout -> logic)
  //
  // I2C bus uses open-drain with external pull-ups.
  // When oe=0, the pad is high-impedance and the line is pulled high.
  // When oe=1, the pad drives the output value.
  //
  // SCL is wire-AND: both master and slave can drive it low for clock
  // stretching. The master drives SCL high/low during normal operation.
  // The slave drives SCL low during stretching, and releases otherwise.
  // ---------------------------------------------------------------------------

  // SCL pad: true open-drain wire-AND
  // In I2C, if ANY device drives the line low, the line is low.
  // Devices should only drive low (oe=1, o=0) or release (oe=0, high-Z).
  // When no device drives low, the pull-up brings the line high.
  // Use 1'bz (high-Z) when released so the pullup can resolve the
  // signal. The testbench provides pullup(scl) and pullup(sda).
  wire scl_driven_low = (scl_oe_mst && !scl_o_mst) ||
                        (scl_oe_slv && !scl_o_slv);
  assign scl   = scl_driven_low ? 1'b0 : 1'bz;
  assign scl_i = scl;

  // SDA pad: true open-drain wire-AND (same as SCL)
  wire sda_driven_low = (sda_oe_mst && !sda_o_mst) ||
                        (sda_oe_slv && !sda_o_slv);
  assign sda   = sda_driven_low ? 1'b0 : 1'bz;
  assign sda_i = sda;

  // Inputs for master (from bus)
  assign scl_i_mst = scl_i;
  assign sda_i_mst = sda_i;

  // ---------------------------------------------------------------------------
  // I2C Master Instance
  // ---------------------------------------------------------------------------
  i2c_master #(
    .CLK_FREQ         (CLK_FREQ),
    .SCL_FREQ         (SCL_FREQ),
    .BURST_LENGTH     (MST_BURST_LEN),
    .SMBUS_ENABLE     (SMBUS_ENABLE),
    .SMBUS_PEC_ENABLE (SMBUS_PEC_ENABLE)
  ) u_i2c_master (
    .clk             (clk),
    .rst_n           (rst_n),
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
    .bus_busy        ('0),          // Not used in this simple configuration
    .scl_o           (scl_o_mst),
    .scl_oe          (scl_oe_mst),
    .scl_i           (scl_i_mst),
    .sda_o           (sda_o_mst),
    .sda_oe          (sda_oe_mst),
    .sda_i           (sda_i_mst)
  );

  // SMBALERT# pad: open-drain (drive low when asserted, high-Z otherwise)
  assign slv_smbalert_n = slv_smbalert_n_oe ? slv_smbalert_n_o : 1'bz;

  // ---------------------------------------------------------------------------
  // I2C Slave Instance
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
    .clk              (clk),
    .rst_n            (rst_n),
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
    .scl_i            (scl_i),
    .scl_o_slv        (scl_o_slv),
    .scl_oe_slv       (scl_oe_slv),
    .sda_o            (sda_o_slv),
    .sda_oe           (sda_oe_slv),
    .sda_i            (sda_i),
    .smbalert_n_o     (slv_smbalert_n_o),
    .smbalert_n_oe    (slv_smbalert_n_oe)
  );

  pullup(slv_smbalert_n);

endmodule : i2c_top
