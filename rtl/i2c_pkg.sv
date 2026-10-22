//==============================================================================
// i2c_pkg.sv
// I2C Controller Package - Parameters, typedefs, and constants
//
// This package defines the parameters and types used by the I2C master and
// slave controllers. It also defines I2C protocol timing constants.
//==============================================================================

package i2c_pkg;

  // ---------------------------------------------------------------------------
  // I2C State Encodings
  // ---------------------------------------------------------------------------

  // Master FSM States
  typedef enum logic [3:0] {
    MST_IDLE       = 4'b0000,
    MST_START      = 4'b0001,
    MST_SEND_ADDR  = 4'b0010,
    MST_WAIT_ACK   = 4'b0011,
    MST_SEND_DATA  = 4'b0100,
    MST_WAIT_DATA  = 4'b0101,
    MST_READ_DATA  = 4'b0110,
    MST_SEND_ACK   = 4'b0111,
    MST_STOP_PREP  = 4'b1000,
    MST_STOP       = 4'b1001,
    MST_WAIT_STOP  = 4'b1010
  } master_state_t;

  // Slave FSM States
  typedef enum logic [2:0] {
    SLV_IDLE       = 3'b000,
    SLV_GET_ADDR   = 3'b001,
    SLV_ACK_ADDR   = 3'b010,
    SLV_RECV_DATA  = 3'b011,
    SLV_SEND_DATA  = 3'b100,
    SLV_ACK_DATA   = 3'b101
  } slave_state_t;

  // ---------------------------------------------------------------------------
  // I2C Protocol Constants
  // ---------------------------------------------------------------------------

  // Number of bits in a data byte
  parameter int DATA_WIDTH    = 8;

  // Number of bits in a 7-bit address
  parameter int ADDR_WIDTH    = 7;

  // Index of the R/W bit position
  parameter int RW_BIT_POS    = 0;

  // R/W bit values
  parameter bit WRITE_BIT     = 1'b0;
  parameter bit READ_BIT      = 1'b1;

  // ACK value (SDA low = ACK)
  parameter bit ACK           = 1'b0;
  parameter bit NACK          = 1'b1;

  // ---------------------------------------------------------------------------
  // SMBus 3.0 Protocol Constants
  // ---------------------------------------------------------------------------

  // Alert Response Address (ARA) — SMBus 3.0, section 4.9
  // When SMBALERT# is asserted, the master sends a read to this address.
  // The asserting slave responds with its own 7-bit address.
  parameter logic [6:0] ARA_ADDR = 7'h0C;

  // SMBALERT# active level (open-drain, active low)
  parameter bit SMBALERT_ACTIVE = 1'b0;

  // ---------------------------------------------------------------------------
  // SMBus 3.0 ARP (Address Resolution Protocol) Constants
  // ---------------------------------------------------------------------------

  // SMBus Default Address — all ARP-capable devices respond here when AV=0
  parameter logic [6:0] SMBUS_DEFAULT_ADDR = 7'h61;

  // ARP Command Codes
  parameter logic [7:0] ARP_CMD_PREPARE_TO_ARP = 8'h01;  // Prepare to ARP
  parameter logic [7:0] ARP_CMD_GET_UDID       = 8'h02;  // Get UDID (General)
  parameter logic [7:0] ARP_CMD_ASSIGN_ADDR    = 8'h03;  // Assign Address
  parameter logic [7:0] ARP_CMD_GET_UDID_DIR   = 8'h04;  // Get UDID Directed
  parameter logic [7:0] ARP_CMD_RESET_DEVICE   = 8'h05;  // Reset Device (General)
  parameter logic [7:0] ARP_CMD_RESET_DEV_DIR  = 8'h06;  // Reset Device Directed

  // UDID is 16 bytes (128 bits)
  parameter int UDID_BYTES = 16;
  parameter int UDID_WIDTH = 128;

endpackage : i2c_pkg
