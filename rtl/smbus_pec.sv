//==============================================================================
// smbus_pec.sv
// SMBus 3.0 PEC (Packet Error Checking) CRC-8 Computation Module
//
// Implements the SMBus PEC CRC-8 generator polynomial:
//   C(x) = x^8 + x^2 + x^1 + x^0  →  0x107  (8-bit value: 0x07)
//
// The CRC-8 is computed over the entire SMBus transaction:
//   Slave Address (7 bits) + R/W bit + Data bytes
//
// Usage:
//   1. Assert `reset_crc` for one cycle at START condition
//   2. Feed each byte (address and data) with `data_valid` for one cycle
//   3. Read `pec_out` after all data bytes have been fed
//
// The CRC is computed in a single clock cycle per byte using a bit-serial
// LFSR implementation. The module is synthesizable and resource-efficient.
//==============================================================================

`timescale 1ns / 1ps

module smbus_pec (
  input  logic       clk,
  input  logic       rst_n,

  // Control
  input  logic       reset_crc,    // Reset CRC to initial value (use at START)
  input  logic       data_valid,   // One-cycle pulse when data_byte is valid
  input  logic [7:0] data_byte,    // Byte to feed into CRC computation

  // Output
  output logic [7:0] pec_out       // Computed PEC CRC-8 value
);

  // ---------------------------------------------------------------------------
  // CRC-8 Internal Register
  // ---------------------------------------------------------------------------
  logic [7:0] crc_reg;

  // ---------------------------------------------------------------------------
  // CRC-8 Combinational Function
  //
  // Computes CRC-8 using polynomial 0x07 (x^8 + x^2 + x + 1), MSB-first
  // (no reflection) as required by the SMBus 3.0 specification.
  //
  // Algorithm for each byte:
  //   1. XOR the input byte with the current CRC
  //   2. For each of 8 bits (MSB to LSB):
  //      - If MSB is 1: shift left, XOR with polynomial 0x07
  //      - If MSB is 0: shift left only
  //
  // This is a bit-level LFSR implementation using MSB-first ordering.
  // Each call processes one byte and produces the updated CRC.
  //
  // Verification: CRC-8(0x00, 0x01) = 0x07 (matches SMBus PEC spec)
  // ---------------------------------------------------------------------------
  function automatic logic [7:0] crc8_update(
    input logic [7:0] crc_current,
    input logic [7:0] data
  );
    logic [7:0] tmp;
    tmp = crc_current ^ data;
    for (int i = 0; i < 8; i++) begin
      if (tmp[7])
        tmp = {tmp[6:0], 1'b0} ^ 8'h07;  // Shift left, XOR with polynomial
      else
        tmp = {tmp[6:0], 1'b0};           // Shift left only
    end
    return tmp;
  endfunction

  // ---------------------------------------------------------------------------
  // CRC-8 Register Update
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      crc_reg <= 8'h00;
    else if (reset_crc)
      crc_reg <= 8'h00;  // Initialize to 0x00 per SMBus spec
    else if (data_valid)
      crc_reg <= crc8_update(crc_reg, data_byte);
  end

  // ---------------------------------------------------------------------------
  // Output
  // ---------------------------------------------------------------------------
  assign pec_out = crc_reg;

  // ---------------------------------------------------------------------------
  // Debug
  // ---------------------------------------------------------------------------
  // synthesis translate_off
  // Debug is intentionally placed after the synthesizable logic so it
  // can be safely excluded from synthesis with translate_off/on.
  // synthesis translate_on

`ifdef DEBUG_PEC
  always @(posedge clk) begin
    if (data_valid)
      $display("[%0t] PEC: CRC update data=0x%02x crc=0x%02x", $time, data_byte, crc_reg);
    if (reset_crc)
      $display("[%0t] PEC: CRC reset to 0x00", $time);
  end
`endif

endmodule : smbus_pec
