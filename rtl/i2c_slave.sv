//==============================================================================
// i2c_slave.sv
// I2C Bus Slave Controller
//
// Features:
//   - 7-bit address detection with configurable address
//   - General call (0x00) address detection
//   - Read/write transaction handling
//   - ACK/NACK generation
//   - START/STOP condition detection
//   - Bus busy monitoring
//   - Double-buffered data interface
//
// The slave continuously monitors the I2C bus for its own address and
// responds accordingly. It can be accessed via a simple FIFO/handshake
// interface for data exchange with the core logic.
//==============================================================================

`timescale 1ns / 1ps

import i2c_pkg::*;

module i2c_slave #(
  // 7-bit slave address
  parameter logic [6:0] SLAVE_ADDR = 7'h50,
  // Enable general call (0x00) address detection
  parameter bit         GEN_CALL   = 1'b0,
  // SMBus 3.0 PEC (Packet Error Checking) — CRC-8 verification
  parameter bit         SMBUS_PEC_ENABLE = 1'b0,
  // SMBus 3.0 SMBALERT# — open-drain interrupt pin
  parameter bit         SMBALERT_ENABLE = 1'b0,
  // SMBus 3.0 ARP — Address Resolution Protocol support
  parameter bit         SMARP_ENABLE    = 1'b0,
  // 128-bit UDID (Unique Device Identifier) for ARP — 16 bytes
  // Byte 0=Capabilities, 1=Version/Rev, 2-3=VendorID, 4-7=DeviceID,
  // 8-11=Interface, 12-13=Subsystem, 14-15=Vendor Specific
  parameter logic [UDID_WIDTH-1:0] UDID = '0,
  // Clock stretching support
  // When enabled, the slave holds SCL low after receiving a byte until
  // the user logic has consumed rx_data (rx_ready), or before transmitting
  // a byte until the user logic has provided tx_data (tx_valid).
  parameter bit         SLV_STRETCH_ENABLE = 1'b0
) (
  // ---------------------------------------------------------------------------
  // System Interface
  // ---------------------------------------------------------------------------
  input  logic       clk,
  input  logic       rst_n,

  // ---------------------------------------------------------------------------
  // Host Data Interface (for internal logic)
  // ---------------------------------------------------------------------------
  // Data to transmit when master reads (pre-loaded by user logic)
  input  logic [7:0] tx_data,
  // TX data valid (user logic has loaded data to send)
  input  logic       tx_valid,
  // TX ready (slave has consumed tx_data)
  output logic       tx_ready,
  // Data received from master (for writes)
  output logic [7:0] rx_data,
  // RX data valid (new data received from master)
  output logic       rx_valid,
  // RX ready (user logic has consumed rx_data)
  input  logic       rx_ready,

  // ---------------------------------------------------------------------------
  // Status Outputs
  // ---------------------------------------------------------------------------
  // Slave is being addressed (address match)
  output logic       addressed,
  // Transaction direction (1 = read, 0 = write), valid when addressed
  output logic       dir_rw,
  // Transaction in progress (from address match to STOP)
  output logic       busy,
  // General call detected
  output logic       gen_call_detected,
  // START condition detected
  output logic       start_detected,
  // STOP condition detected
  output logic       stop_detected,
  // SMBus PEC error (CRC mismatch — checked at STOP)
  output logic       pec_error,
  // SMBALERT# pending (slave needs attention, asserted when data to tx)
  output logic       alert_pending,

  // ---------------------------------------------------------------------------
  // I2C Bus Interface (bidirectional with tristate controls)
  // ---------------------------------------------------------------------------
  input  logic       scl_i,      // SCL input
  output logic       scl_o_slv,  // SCL output (for clock stretching)
  output logic       scl_oe_slv, // SCL output enable (1 = driving)
  output logic       sda_o,      // SDA output value
  output logic       sda_oe,     // SDA output enable (1 = driving)
  input  logic       sda_i,      // SDA input
  // SMBALERT# open-drain output (active low, shared bus)
  output logic       smbalert_n_o,   // SMBALERT# output value
  output logic       smbalert_n_oe   // SMBALERT# output enable (1 = driving)
);

  // ---------------------------------------------------------------------------
  // Edge Detection
  // ---------------------------------------------------------------------------
  logic scl_sync, scl_prev;
  logic sda_sync, sda_prev;
  logic scl_rising, scl_falling;
  logic sda_rising, sda_falling;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      scl_sync <= 1'b1;
      scl_prev <= 1'b1;
      sda_sync <= 1'b1;
      sda_prev <= 1'b1;
    end else begin
      scl_sync <= scl_i;
      scl_prev <= scl_sync;
      sda_sync <= sda_i;
      sda_prev <= sda_sync;
    end
  end

  assign scl_rising  =  scl_sync && ~scl_prev;
  assign scl_falling = ~scl_sync &&  scl_prev;
  assign sda_rising  =  sda_sync && ~sda_prev;
  assign sda_falling = ~sda_sync &&  sda_prev;

  // ---------------------------------------------------------------------------
  // START / STOP Condition Detectors
  // ---------------------------------------------------------------------------
  // START: SDA falling while SCL high
  // STOP:  SDA rising while SCL high

  logic scl_high;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      scl_high <= 1'b0;
    else
      scl_high <= scl_sync;  // simple registered version
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      start_detected <= 1'b0;
      stop_detected  <= 1'b0;
      gen_call_detected <= 1'b0;
    end else begin
      // Default: no new detection
      start_detected <= 1'b0;
      stop_detected  <= 1'b0;
      gen_call_detected <= 1'b0;

      // START: SDA falls while SCL is high
      // Only valid in IDLE (initial START) or data phase states (repeated START).
      // During S_ADDR (address capture), the slave's ACK driving at bit_pos=8
      // can create an SDA falling edge that, due to delta-cycle resolution with
      // the open-drain pad model, is spuriously detected as a START condition.
      if (scl_sync && sda_falling &&
          (state == S_IDLE || state == S_TX_DATA ||
           state == S_RX_DATA || state == S_ACK_PHASE)) begin
        start_detected <= 1'b1;
      end

      // STOP: SDA rises while SCL is sampled high
      if (scl_sync && sda_rising) begin
        stop_detected <= 1'b1;
      end

      // General call detection
      if (GEN_CALL && start_detected)
        gen_call_detected <= 1'b1;
    end
  end

  // ---------------------------------------------------------------------------
  // Bus Busy Detection
  // ---------------------------------------------------------------------------
  logic bus_busy;
  logic stop_glitch_filter;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      bus_busy <= 1'b0;
    else if (start_detected)
      bus_busy <= 1'b1;
    else if (stop_detected)
      bus_busy <= 1'b0;
  end

  // ---------------------------------------------------------------------------
  // Internal State
  // ---------------------------------------------------------------------------

  // 9-bit shift register: 8 data bits + 1 ACK/NACK bit
  logic [8:0] shift_reg;
  logic [3:0] bit_pos;       // bit position counter (0-8)
  logic       addr_match;    // address match flag
  logic       ack_sent;      // ACK has been sent for current phase
  logic       phase_addr;    // Currently in address phase
  logic       my_addr;       // This transaction is addressed to us (own addr, ARA, or default)
  logic       ara_addressed; // Addressed via Alert Response Address
  logic       smbalert_asserted; // Internal: SMBALERT# being driven low

  // SMBus 3.0 ARP registers
  logic [6:0] assigned_addr;       // Current operational address (may change via ARP)
  logic       av;                   // Address Valid flag (1 = assigned via ARP)
  logic [7:0] arp_cmd;             // Captured ARP command code
  // Byte counter within ARP transaction. Must be wide enough to count past
  // the Assign Address sequence (1 cmd byte + UDID_BYTES UDID bytes + 1
  // new-address byte = UDID_BYTES + 2); a too-narrow counter would wrap
  // silently and misinterpret UDID bytes as new command bytes.
  logic [$clog2(UDID_BYTES + 2)-1:0] arp_byte_cnt;
  logic       arp_get_udid_pending; // Expect read for UDID after Get UDID cmd
  logic [3:0] arp_tx_byte_cnt;     // UDID byte index currently being transmitted
  logic [UDID_WIDTH-1:0] arp_rx_udid_buf;   // Buffer for received UDID during Assign Address
  logic [6:0] arp_new_addr;        // New address received during Assign Address
  logic       arp_rx_udid;         // Currently receiving UDID bytes for Assign Address
  logic       def_addr;            // Addressed via SMBus Default Address (0x61)
  logic       own_arp_write;       // Addressed via own assigned address, write direction
                                    // (used for directed ARP commands: Get UDID Directed,
                                    // Reset Device Directed — SMBus 3.0 section 4.9)

  // ---------------------------------------------------------------------------
  // Main Slave State Machine
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    S_IDLE       = 3'h0,
    S_ADDR       = 3'h1,
    S_ACK_REF    = 3'h2,  // ACK reference (decision point)
    S_TX_DATA    = 3'h3,  // Transmitting data to master
    S_RX_DATA    = 3'h4,  // Receiving data from master
    S_ACK_PHASE  = 3'h5   // ACK after data byte
  } slave_fsm_t;

  slave_fsm_t state, next_state;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      state <= S_IDLE;
    else
      state <= next_state;
  end

  // ---------------------------------------------------------------------------
  // Next State Logic
  // ---------------------------------------------------------------------------
  always_comb begin
    next_state = state;

    case (state)

      S_IDLE: begin
        // Wait for START condition, then begin address capture
        if (start_detected)
          next_state = S_ADDR;
      end

      S_ADDR: begin
        // Capture 8 bits (7-bit address + R/W) + ACK bit
        // Transition on 9th scl_rising (ACK sample point) when bit_pos=0.
        // bit_pos wraps from 8→0 on the 9th scl_falling, so the first
        // scl_rising after the wrap (the 10th scl_rising) has bit_pos=0.
        // The guard on shift-register capture uses bit_pos != 0 to
        // match the original proven behavior.
        if (stop_detected)
          next_state = S_IDLE;  // Abort on STOP (SMBus timeout recovery)
        else if (bit_pos == 0 && scl_rising)
          next_state = S_ACK_REF;
      end

      S_ACK_REF: begin
        // Decision point: if address matched, proceed with data phase
        // Addr phase has 9 SCL cycles (8 addr bits + 1 ACK)
        if (stop_detected)
          next_state = S_IDLE;  // Abort on STOP (SMBus timeout recovery)
        else if (my_addr && scl_falling) begin
          if (dir_rw)
            next_state = S_TX_DATA;  // Master reading from us
          else
            next_state = S_RX_DATA;  // Master writing to us
        end else if (!my_addr && scl_falling) begin
          next_state = S_IDLE;  // Not our address, go idle
        end
      end

      S_TX_DATA: begin
        // Transmit data byte + receive ACK from master
        if (stop_detected)
          next_state = S_IDLE;
        else if (start_detected)
          next_state = S_ADDR;  // Repeated START
        else if (bit_pos == 8 && scl_falling)
          next_state = S_ACK_PHASE;
      end

      S_RX_DATA: begin
        // Receive data byte + send ACK
        if (stop_detected)
          next_state = S_IDLE;
        else if (start_detected)
          next_state = S_ADDR;  // Repeated START
        else if (bit_pos == 8 && scl_falling)
          next_state = S_ACK_PHASE;
      end

      S_ACK_PHASE: begin
        // After ACK phase, check for STOP or repeated START
        if (stop_detected)
          next_state = S_IDLE;
        else if (start_detected)
          next_state = S_ADDR;  // Repeated START
        else if (scl_falling && bit_pos == 0)
          // Continue with next byte in same direction
          if (dir_rw)
            next_state = S_TX_DATA;
          else
            next_state = S_RX_DATA;
      end

      default: next_state = S_IDLE;
    endcase
  end

  // ---------------------------------------------------------------------------
  // Bit Position Counter
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      bit_pos <= 0;
    else begin
      // Reset on START condition or IDLE entry
      if (start_detected || (state == S_IDLE))
        bit_pos <= 0;
      // Advance on every SCL falling edge during active states
      // NOTE: S_ACK_REF is NOT included because S_TX_DATA needs
      // bit_pos=0 at entry to drive tx_data[7] as the first bit.
      // S_RX_DATA handles the first capture (bit_pos=0) via a
      // separate capture guard (bit_pos < 8 instead of != 0).
      else if (scl_falling && 
               (state == S_ADDR || state == S_TX_DATA || 
                state == S_RX_DATA || state == S_ACK_PHASE))
        if (bit_pos == 8)
          bit_pos <= 0;
        else
          bit_pos <= bit_pos + 1;
    end
  end

  // ---------------------------------------------------------------------------
  // Shift Register: Capture data on SCL rising edges
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      shift_reg <= '0;
    else if (scl_rising && state == S_ADDR && (bit_pos != 0)) begin
      // Address phase: capture at bit_pos=1..8 = 8 values
      // bit_pos=1 captures the first address bit on the second
      // scl_rising after entering S_ADDR.
      // bit_pos=8 captures the ACK bit on the 9th scl_rising.
      shift_reg <= {shift_reg[7:0], sda_i};
    end else if (scl_rising && state == S_RX_DATA && (bit_pos < 8)) begin
      // Data phase: capture at bit_pos=0..7 = 8 values
      // bit_pos=0 captures the first data bit (MSB) at the first
      // scl_rising after entering S_RX_DATA.
      // bit_pos=8 is the ACK sample point and is NOT captured.
      shift_reg <= {shift_reg[7:0], sda_i};
    end else if (scl_rising && state == S_ACK_PHASE && !dir_rw) begin
      // Second+ byte of a write burst: the master starts driving the next
      // byte's MSB immediately after the ACK bit, but this slave FSM only
      // moves from S_ACK_PHASE to S_RX_DATA on the *following* scl_falling
      // (one full bit period later, to leave a window for STOP/repeated-
      // START detection). Without this capture, that MSB is silently
      // dropped — S_RX_DATA's own capture only starts from bit_pos=1
      // (bit_pos having already advanced by 1 while still in S_ACK_PHASE),
      // so it only ever sees the remaining 7 bits. This mirrors the
      // S_RX_DATA capture above, just one cycle earlier in this state.
      shift_reg <= {shift_reg[7:0], sda_i};
    end else if (state == S_TX_DATA) begin
      // TX stays loaded from tx_data
    end
  end

  // ---------------------------------------------------------------------------
  // Address Matching
  //
  // After 8 shifts of {shift_reg[7:0], sda_i}, the shift_reg holds:
  //   shift_reg[7:1] = {bit0, bit1, ..., bit6} = 7-bit address
  //   shift_reg[0]   = bit7 = R/W bit
  // ---------------------------------------------------------------------------
  logic [7:0] recv_addr;

  assign recv_addr = {shift_reg[7:1], shift_reg[0]};  // 7-bit addr + R/W (kept for debug)

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      my_addr  <= 1'b0;
      dir_rw   <= 1'b0;
      ara_addressed <= 1'b0;
      def_addr <= 1'b0;
      own_arp_write <= 1'b0;
    end else if (state == S_ADDR && bit_pos == 8 && scl_falling) begin
      // Address fully captured (8 bits: 7 addr + 1 R/W) at 9th scl_falling
      // Match: (1) Own assigned address, (2) ARA, (3) SMBus Default Addr (ARP, !av)
      my_addr <= (shift_reg[7:1] == assigned_addr) ||
                 (SMBALERT_ENABLE && (shift_reg[7:1] == ARA_ADDR)) ||
                 (SMARP_ENABLE && !av && (shift_reg[7:1] == SMBUS_DEFAULT_ADDR));
      dir_rw  <= shift_reg[0];  // 0 = write, 1 = read
      ara_addressed <= SMBALERT_ENABLE && (shift_reg[7:1] == ARA_ADDR);
      def_addr <= SMARP_ENABLE && !av && (shift_reg[7:1] == SMBUS_DEFAULT_ADDR) &&
                  (shift_reg[0] == WRITE_BIT);  // Write to default = ARP cmd
      // Directed ARP commands (Get UDID Directed, Reset Device Directed) are
      // addressed to the device's own assigned address rather than the
      // default address 0x61. Command codes 0x04/0x06 are reserved by the
      // SMBus 3.0 ARP spec for this purpose, so an ARP-compliant device's
      // normal register map must avoid them.
      own_arp_write <= SMARP_ENABLE && (shift_reg[7:1] == assigned_addr) &&
                       (shift_reg[0] == WRITE_BIT);
    end else if (state == S_IDLE) begin
      my_addr <= 1'b0;
      ara_addressed <= 1'b0;
      def_addr <= 1'b0;
      own_arp_write <= 1'b0;
    end
  end

  // ---------------------------------------------------------------------------
  // ACK Generation
  // ---------------------------------------------------------------------------
  // ACK is driven on the 9th SCL cycle (bit_pos == 8)
  // We need to drive SDA low on the falling edge of SCL at bit_pos 7

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      ack_sent <= 1'b0;
    else if (state == S_ACK_REF)
      ack_sent <= 1'b1;  // We generate ACK on addr match
    else if (state == S_ACK_PHASE)
      ack_sent <= 1'b1;  // We generate ACK after receiving data
    else if (state == S_IDLE)
      ack_sent <= 1'b0;
  end

  // ---------------------------------------------------------------------------
  // ACK Decision Register
  // ---------------------------------------------------------------------------
  // Captured at the same time as my_addr, used during the 9th SCL cycle.
  logic should_ack;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      should_ack <= 1'b0;
    else if (state == S_ADDR && bit_pos == 8 && scl_falling)
      // ACK if: (1) our assigned address, (2) ARA, (3) Default Addr (ARP)
      should_ack <= (shift_reg[7:1] == assigned_addr) ||
                    (SMBALERT_ENABLE && (shift_reg[7:1] == ARA_ADDR)) ||
                    (SMARP_ENABLE && !av && (shift_reg[7:1] == SMBUS_DEFAULT_ADDR));
    else if (state == S_IDLE)
      should_ack <= 1'b0;
  end

  // ---------------------------------------------------------------------------
  // SDA Output Control
  //
  // I2C ACK timing:
  //   - During the 9th SCL cycle: receiver drives SDA low (ACK)
  //   - bit_pos == 8 corresponds to the 9th SCL cycle (after 8 data bits)
  //   - The ACK is driven from scl_falling (entering 9th cycle) until
  //     the next scl_falling (completing 9th cycle)
  // ---------------------------------------------------------------------------
  always_comb begin
    sda_o  = 1'b1;
    sda_oe = 1'b0;

    case (state)

      S_IDLE: begin
        // Release SDA
        sda_oe = 1'b0;
        sda_o  = 1'b1;
      end

      // ---------------------------------------------------------------------
      // S_ADDR: Address phase
      //   - bits 0-7: master drives address + R/W, slave releases SDA
      //   - bit 8 (9th cycle): slave drives ACK/NACK
      //
      // During bit_pos==8 (9th SCL cycle: scl_falling #8 → scl_rising #9),
      // shift_reg has all 8 address bits (last sampled on the 8th capture
      // at bit_pos=7). Use combinational address match for ACK/NACK.
      // ---------------------------------------------------------------------
      S_ADDR: begin
        if (bit_pos == 8 || (bit_pos == 0 && my_addr)) begin
          // 9th SCL cycle & ACK sample: combinational address match
          // bit_pos==8 covers the 9th cycle; bit_pos==0 with my_addr covers
          // the 10th scl_rising where S_ACK_REF takes over.
          // ACK if: (1) assigned addr, (2) ARA, (3) Default Addr (ARP, !av)
          sda_o  = ((shift_reg[7:1] == assigned_addr) ||
                    (SMBALERT_ENABLE && (shift_reg[7:1] == ARA_ADDR)) ||
                    (SMARP_ENABLE && !av && (shift_reg[7:1] == SMBUS_DEFAULT_ADDR)))
                   ? ACK : NACK;
          sda_oe = 1'b1;
        end else begin
          // First 8 SCL cycles: release SDA (master drives address bits)
          sda_oe = 1'b0;
          sda_o  = 1'b1;
        end
      end

      // ---------------------------------------------------------------------
      // S_ACK_REF: Decision point after address ACK
      //   - ACK was already driven in S_ADDR on bit_pos == 8
      //   - Keep SDA driven throughout S_ACK_REF to handle clock
      //     stretching. When the slave stretches SCL during the
      //     ACK cycle, the master samples SDA at the delayed
      //     scl_rising (after stretch releases). If SDA were
      //     released when S_ACK_REF is entered, the pullup would
      //     bring it HIGH and the master would see NACK even
      //     though we intended ACK.
      //   - SDA is released when the state transitions to
      //     S_TX_DATA/S_RX_DATA where data-phase control takes over.
      // ---------------------------------------------------------------------
      S_ACK_REF: begin
        // Combinational address match (same as in S_ADDR)
        sda_o  = ((shift_reg[7:1] == assigned_addr) ||
                  (SMBALERT_ENABLE && (shift_reg[7:1] == ARA_ADDR)) ||
                  (SMARP_ENABLE && !av && (shift_reg[7:1] == SMBUS_DEFAULT_ADDR)))
                 ? ACK : NACK;
        sda_oe = 1'b1;
      end

      // ---------------------------------------------------------------------
      // S_TX_DATA: Transmit data to master (read transaction)
      //   - Normal read:  slave drives tx_data MSB-first
      //   - ARA read:     slave drives own address {SLAVE_ADDR, 1'b0}
      //   - ARP Get UDID: slave drives UDID bytes MSB-first
      //   - bits 0-7: slave drives SDA
      //   - bit 8 (9th cycle): master drives ACK/NACK, slave releases SDA
      // ---------------------------------------------------------------------
      S_TX_DATA: begin
        if (bit_pos <= 7) begin
          // Select data source:
          //   1. ARA: send own address
          //   2. ARP Get UDID: send UDID bytes
          //   3. Normal: send tx_data
          if (ara_addressed && bit_pos <= 6)
            sda_o  = SLAVE_ADDR[6 - bit_pos];  // Send own address (ARA response)
          else if (ara_addressed)
            sda_o  = 1'b0;  // LSB = 0 for ARA response
          else if (arp_get_udid_pending)
            // Extract bit from current UDID byte (MSB-first)
            // arp_tx_byte_cnt selects which of UDID_BYTES bytes to send
            // Byte 0 transmitted first = UDID[UDID_WIDTH-1 -: 8]
            sda_o  = UDID[(UDID_BYTES - 1 - arp_tx_byte_cnt) * 8 + 7 - bit_pos];
          else
            sda_o  = tx_data[7 - bit_pos];
          sda_oe = 1'b1;
        end else begin
          // Release SDA for master ACK (bit_pos == 8)
          sda_oe = 1'b0;
          sda_o  = 1'b1;
        end
      end

      // ---------------------------------------------------------------------
      // S_RX_DATA: Receive data from master (write transaction)
      //   - bits 0-7: master drives data, slave releases SDA
      //   - bit 8 (9th cycle): slave drives ACK
      // ---------------------------------------------------------------------
      S_RX_DATA: begin
        if (bit_pos == 8) begin
          // 9th SCL cycle: drive ACK = acknowledge data received
          sda_o  = ACK;
          sda_oe = 1'b1;
        end else begin
          // First 8 SCL cycles: master drives, slave listens
          sda_oe = 1'b0;
          sda_o  = 1'b1;
        end
      end

      // ---------------------------------------------------------------------
      // S_ACK_PHASE: Decision point after data byte ACK
      //   - For writes: ACK already driven in S_RX_DATA on bit_pos == 8
      //   - For reads:  master drove ACK/NACK in S_TX_DATA's 9th cycle
      //   - Release SDA and wait for next state (write direction) — but
      //     for a continuing read, drive the next byte's first bit right
      //     here instead of releasing. The master starts sampling that
      //     bit immediately after the ACK, while this FSM doesn't move
      //     back to S_TX_DATA until the *following* scl_falling (mirrors
      //     the S_RX_DATA capture fix above; same root cause). Left as
      //     released, a floating (pulled-high) bus would corrupt a true
      //     0 bit to 1 — this is bit_pos=0 of the next byte, computed the
      //     same way S_TX_DATA's bit_pos<=7 branch does.
      // ---------------------------------------------------------------------
      S_ACK_PHASE: begin
        if (dir_rw) begin
          if (ara_addressed)
            sda_o = SLAVE_ADDR[6];
          else if (arp_get_udid_pending)
            sda_o = UDID[(UDID_BYTES - 1 - arp_tx_byte_cnt) * 8 + 7];
          else
            sda_o = tx_data[7];
          sda_oe = 1'b1;
        end else begin
          sda_oe = 1'b0;
          sda_o  = 1'b1;
        end
      end

      default: begin
        sda_oe = 1'b0;
        sda_o  = 1'b1;
      end
    endcase
  end

  // ---------------------------------------------------------------------------
  // Data Interface to Host Logic
  // ---------------------------------------------------------------------------

  // rx_data: valid when a full byte has been received from master
  // After 8 captures on scl_rising (bit_pos=1..8), shift_reg[7:0] has the
  // received byte. Capture on scl_falling so the 8th capture has completed.
  // Use a wire to avoid iverilog constant-part-select warnings.
  wire [7:0] rx_data_from_sreg = shift_reg[7:0];
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      rx_data <= '0;
    else if (state == S_RX_DATA && bit_pos == 8 && scl_falling)
      rx_data <= rx_data_from_sreg;  // Capture received data byte
  end

  // rx_valid: pulse when new rx_data is available
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      rx_valid <= 1'b0;
    else if (state == S_RX_DATA && bit_pos == 8 && scl_falling)
      rx_valid <= 1'b1;
    else if (rx_ready)
      rx_valid <= 1'b0;
  end

  // tx_ready: slave is ready for new tx_data (previous byte has been sent)
  // Transmitted byte is complete at bit_pos==8, capture on scl_falling.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      tx_ready <= 1'b0;
    else if (state == S_TX_DATA && bit_pos == 8 && scl_falling)
      tx_ready <= 1'b1;  // Previous byte sent, ready for next
    else if (tx_valid)
      tx_ready <= 1'b0;  // Data loaded
  end

  // ---------------------------------------------------------------------------
  // Status Outputs
  // ---------------------------------------------------------------------------

  // addressed: pulse when our address is matched
  // Cleared on transition to S_IDLE (not on stop_detected which can
  // trigger on false STOPs from SDA release after ACK)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      addressed <= 1'b0;
    else if (state == S_ACK_REF && my_addr && scl_falling)
      addressed <= 1'b1;
    else if (state == S_IDLE)
      addressed <= 1'b0;
  end

  // busy: active from address match to STOP
  // Cleared on transition to S_IDLE (real STOP or error)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      busy <= 1'b0;
    else if (state == S_ACK_REF && my_addr)
      busy <= 1'b1;
    else if (state == S_IDLE)
      busy <= 1'b0;
  end

  // ---------------------------------------------------------------------------
  // SMBus 3.0 SMBALERT# — Open-drain Interrupt Pin
  // ---------------------------------------------------------------------------
  // SMBALERT# is an active-low, open-drain signal that a slave asserts
  // when it needs attention from the master (e.g., has data to transmit).
  // The master responds by sending a read to the Alert Response Address
  // (ARA = 0x0C). The asserting slave then returns its own 7-bit address.
  //
  // Assertion: when the slave has data to send (tx_valid && !tx_ready)
  // and is not currently in a transaction.
  //
  // De-assertion: when the master addresses this slave (own addr or ARA).
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      alert_pending <= 1'b0;
      smbalert_asserted <= 1'b0;
    end else if (SMBALERT_ENABLE) begin
      // Assert: data pending and we're not already in a transaction
      if (tx_valid && !tx_ready && !busy && !my_addr)
        alert_pending <= 1'b1;
      else if (addressed || (!tx_valid || tx_ready))
        alert_pending <= 1'b0;

      // SMBALERT# follows alert_pending, cleared when addressed
      if (alert_pending && !addressed)
        smbalert_asserted <= 1'b1;
      else if (addressed || state == S_IDLE)
        smbalert_asserted <= 1'b0;
    end else begin
      alert_pending <= 1'b0;
      smbalert_asserted <= 1'b0;
    end
  end

  // SMBALERT# output: open-drain (drive to the active level when asserted,
  // high-Z otherwise)
  assign smbalert_n_o  = smbalert_asserted ? SMBALERT_ACTIVE : ~SMBALERT_ACTIVE;
  assign smbalert_n_oe = smbalert_asserted ? 1'b1 : 1'b0;

  // ---------------------------------------------------------------------------
  // Slave Clock Stretching — Flow Control
  // ---------------------------------------------------------------------------
  // The slave can stretch SCL (hold it low after the master releases it) to
  // give the user logic more time to process received data or provide
  // data for transmission.
  //
  // Write stretch: after receiving a byte into rx_data, the slave holds
  // SCL low during the ACK phase until the user asserts rx_ready.
  //
  // Read stretch: before transmitting a byte, the slave holds SCL low
  // during the ACK phase until the user asserts tx_valid with data.
  //
  // The stretch is implemented as an open-drain output: when stretch is
  // active, scl_oe_slv=1 and scl_o_slv=0, forcing SCL low even after the
  // master has released it. When the condition clears, scl_oe_slv=0
  // releases SCL, and the pullup brings it high (master detects and resumes).
  // ---------------------------------------------------------------------------
  logic stretch_active;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      stretch_active <= 1'b0;
    else if (SLV_STRETCH_ENABLE) begin
      // Default: retain current value. The stretch stays active until
      // explicitly released by one of the release conditions below.
      // IMPORTANT: Do NOT use a default of 1'b0 here — that would make
      // stretch_active a one-cycle pulse rather than a persistent hold.
      stretch_active <= stretch_active;

      // --- Write stretch: after receiving a byte, hold SCL low until
      //     rx_data is consumed. Assert on EVERY byte-receive event;
      //     this avoids a race with rx_valid being set on the same cycle.
      //     Guard with addressed (which is stable by the time S_RX_DATA
      //     is reached, many cycles after S_ACK_REF).
      if (addressed && state == S_RX_DATA && bit_pos == 8 && scl_falling)
        stretch_active <= 1'b1;  // Always stretch on byte received

      // --- Read stretch: before TX, hold SCL low until tx_data is provided.
      //     Assert when entering TX phase but no data is available yet.
      //     Two cases, both checked at the transition edge *out of* the
      //     state that's ending (using the pre-transition state name),
      //     not the state being entered — mirroring the write-side (RX)
      //     stretch's own proven shape below (state==S_RX_DATA &&
      //     bit_pos==8 && scl_falling):
      //     (a) First byte: transition S_ACK_REF → S_TX_DATA (check on scl_falling
      //         at the addr ACK end, when state is still S_ACK_REF)
      //     (b) Subsequent bytes: transition S_TX_DATA → S_ACK_PHASE (check
      //         on scl_falling at bit_pos==8, when state is still S_TX_DATA)
      //
      //     NOTE: addressed is NOT used here because it's registered and
      //     transitions from 0→1 on the SAME cycle as this condition evaluates.
      //     Due to non-blocking assignment ordering, addressed would still be 0
      //     when checked, preventing the stretch from ever firing.
      //     The state + my_addr + dir_rw guards are sufficient.
      //
      //     Skip stretch when the slave uses internal data sources (ARP UDID
      //     reads, ARA responses) that don't require tx_data from user logic.
      if (state == S_ACK_REF && my_addr && dir_rw && scl_falling && !tx_valid &&
          !arp_get_udid_pending && !ara_addressed)
        stretch_active <= 1'b1;  // Before first TX byte (only if needs user data)

      // The previous version of this check used state==S_ACK_PHASE with
      // bit_pos==8, which was unreachable dead code: S_ACK_PHASE is
      // entered with bit_pos already wrapped to 0 (the same edge that
      // leaves S_TX_DATA at bit_pos==8 wraps it), and the single
      // scl_falling that occurs while state==S_ACK_PHASE both advances
      // bit_pos 0->1 *and* transitions state away from S_ACK_PHASE on
      // that same edge — bit_pos is never anything but 0 while that state
      // is current, so a check keyed on it (whichever bit_pos value) fires
      // either a full bit-period too late (S_ACK_PHASE's own combinational
      // sda_o block below has already driven the next byte's first bit,
      // tx_data[7], using whatever stale value was present — confirmed via
      // a minimal repro, 0xAD received as 0x2D, only the MSB wrong) or, if
      // moved a bit-period earlier instead, deadlocks the protocol (holds
      // SCL low before the current byte's own ACK bit is even exchanged).
      // Checking instead at the S_TX_DATA->S_ACK_PHASE transition edge
      // itself (this state, bit_pos==8, scl_falling) engages stretch_active
      // at the exact same edge tx_ready also sets on, giving it a full
      // half-bit-period of margin before the master's own next sampling
      // edge — the same timing shape the write-side (RX) stretch above
      // already uses successfully at its own analogous boundary.
      if (state == S_TX_DATA && scl_falling && bit_pos == 8 && dir_rw)
        if (!tx_valid && !arp_get_udid_pending && !ara_addressed)
          stretch_active <= 1'b1;  // Before subsequent TX bytes (needs user data)

      // --- Release conditions ---
      // For writes: release when rx_ready consumes rx_valid
      if (stretch_active && !dir_rw && rx_ready && rx_valid)
        stretch_active <= 1'b0;
      // For reads: release when tx_valid provides new data
      if (stretch_active && dir_rw && tx_valid && !tx_ready)
        stretch_active <= 1'b0;
      // Force release on STOP or START (transaction ending)
      if (stop_detected || start_detected)
        stretch_active <= 1'b0;
    end else begin
      stretch_active <= 1'b0;
    end
  end

  // SCL output for clock stretching: open-drain (drive low when stretching)
  // Default is released (high-Z, pullup keeps SCL high)
  assign scl_o_slv  = stretch_active ? 1'b0 : 1'b1;
  assign scl_oe_slv = stretch_active ? 1'b1 : 1'b0;

  // ---------------------------------------------------------------------------
  // SMBus 3.0 ARP — Address Resolution Protocol Logic
  // ---------------------------------------------------------------------------
  // Consolidated always_ff block for ALL ARP state to avoid multiple-driver
  // errors. Handles command reception, byte accumulation, pending flag
  // management, and command processing at STOP.
  //
  // General ARP Commands (received as first byte of write to Default
  // Address 0x61 — apply to any un-assigned device on the bus):
  //   0x01 (Prepare to ARP):  Clear AV flag, reset address
  //   0x02 (Get UDID):        Set pending flag — next read returns 16 UDID bytes
  //   0x03 (Assign Address):  Receive UDID (16 bytes) + new address (1 byte)
  //   0x05 (Reset Device):    Clear AV, reset address
  //
  // Directed ARP Commands (received as first byte of write to this
  // device's own assigned address — target a single, already-addressed
  // device without needing its UDID):
  //   0x04 (Get UDID Directed):    Next read returns 16 UDID bytes
  //   0x06 (Reset Device Directed): Clear AV, reset address
  //
  // The Get UDID sequence spans two transactions:
  //   1. Write: command 0x02 (to 0x61) or 0x04 (to own address)
  //   2. Read (from the same address): 16 UDID bytes transmitted
  //--------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      assigned_addr        <= SLAVE_ADDR;  // Always init (used even without ARP)
      av                   <= 1'b0;
      arp_cmd              <= 8'h00;
      arp_byte_cnt         <= '0;
      arp_rx_udid          <= 1'b0;
      arp_rx_udid_buf      <= '0;
      arp_new_addr         <= 7'h0;
      arp_get_udid_pending <= 1'b0;
      arp_tx_byte_cnt      <= 4'h0;
    end else begin
      // ---------------------------------------------------------
      // Byte counter management
      // ---------------------------------------------------------
      // arp_rx_udid must persist across the many clock cycles between
      // I2C byte arrivals (it spans the whole UDID-receiving phase of
      // Assign Address), so — like arp_byte_cnt — it is NOT reset to 0
      // by default every cycle. It is cleared explicitly: when the
      // address byte completes the sequence (below), or here when the
      // FSM returns to idle (e.g. transaction aborted mid-UDID).
      if (state == S_IDLE && !arp_get_udid_pending) begin
        arp_byte_cnt <= '0;
        arp_rx_udid  <= 1'b0;
      end

      // ---------------------------------------------------------
      // UDID TX byte counter: increment after each UDID byte sent
      // Must check bit_pos==7 (pre-NBA) because bit_pos becomes 8
      // only AFTER the NBA of this scl_falling event. Checking
      // bit_pos==8 (post-NBA value) would never trigger in the
      // always_ff block since it reads the pre-NBA value.
      // Counts 0→1→...→(UDID_BYTES-1)→0 (4-bit wraps, UDID_BYTES <= 16).
      // The clear condition checks >= UDID_BYTES-1, reached after the
      // UDID_BYTES-th increment (0→1→...→(UDID_BYTES-2)→(UDID_BYTES-1)).
      // ---------------------------------------------------------
      if (SMARP_ENABLE && arp_get_udid_pending &&
          state == S_TX_DATA && scl_falling && (bit_pos == 7) &&
          arp_tx_byte_cnt < UDID_BYTES - 1)
        arp_tx_byte_cnt <= arp_tx_byte_cnt + 1;

      // ---------------------------------------------------------
      // Get UDID pending flag management
      // ---------------------------------------------------------
      // Set: when first byte of write to 0x61 is command 0x02 (General),
      // or first byte of write to own address is command 0x04 (Directed).
      if (SMARP_ENABLE && !dir_rw &&
          state == S_RX_DATA && bit_pos == 8 && scl_falling &&
          arp_byte_cnt == 0 &&
          ((def_addr && shift_reg[7:0] == ARP_CMD_GET_UDID) ||
           (own_arp_write && shift_reg[7:0] == ARP_CMD_GET_UDID_DIR))) begin
        arp_get_udid_pending <= 1'b1;
        arp_tx_byte_cnt      <= 4'h0;
      end
      // Clear: STOP after all UDID_BYTES bytes transmitted.
      // arp_tx_byte_cnt is 4 bits (max 15), so this only works correctly
      // for UDID_BYTES <= 16.
      if (SMARP_ENABLE && arp_get_udid_pending && stop_detected &&
          arp_tx_byte_cnt >= UDID_BYTES - 1)
        arp_get_udid_pending <= 1'b0;
      // Clear: Prepare to ARP or Reset Device
      if (SMARP_ENABLE && (arp_cmd == ARP_CMD_PREPARE_TO_ARP ||
          arp_cmd == ARP_CMD_RESET_DEVICE ||
          arp_cmd == ARP_CMD_RESET_DEV_DIR) && stop_detected)
        arp_get_udid_pending <= 1'b0;

      // ---------------------------------------------------------
      // ARP command reception: accumulate bytes during write to 0x61
      // (general commands) or to our own assigned address (directed
      // commands: Get UDID Directed 0x04, Reset Device Directed 0x06).
      // ---------------------------------------------------------
      if (SMARP_ENABLE && (def_addr || own_arp_write) && !dir_rw &&
          state == S_RX_DATA && bit_pos == 8 && scl_falling) begin
        if (arp_byte_cnt == 0) begin
          // First byte = ARP command code. General (broadcast) command
          // codes are only valid via the default address; directed
          // command codes are only valid via our own assigned address.
          if (def_addr && (shift_reg[7:0] == ARP_CMD_PREPARE_TO_ARP ||
                           shift_reg[7:0] == ARP_CMD_GET_UDID       ||
                           shift_reg[7:0] == ARP_CMD_ASSIGN_ADDR    ||
                           shift_reg[7:0] == ARP_CMD_RESET_DEVICE)) begin
            arp_cmd      <= shift_reg[7:0];
            arp_byte_cnt <= 1;

            if (shift_reg[7:0] == ARP_CMD_ASSIGN_ADDR)
              arp_rx_udid <= 1'b1;  // Expect UDID bytes next
          end else if (own_arp_write &&
                       (shift_reg[7:0] == ARP_CMD_GET_UDID_DIR ||
                        shift_reg[7:0] == ARP_CMD_RESET_DEV_DIR)) begin
            arp_cmd      <= shift_reg[7:0];
            arp_byte_cnt <= 1;
          end else begin
            // First byte matched neither a general nor a directed ARP
            // command: this is not an ARP transaction at all (own_arp_write
            // is set for *every* write to the assigned address, ARP or
            // not — it's just plain application data this time). Without
            // this branch arp_byte_cnt stays stuck at 0 for the whole
            // transaction, so every later byte in the burst gets
            // re-checked as a fresh "first ARP command byte" — any
            // ordinary data byte that happens to equal 0x04/0x06
            // anywhere in the burst would then be misinterpreted as an
            // ARP command mid-transaction.
            arp_byte_cnt <= arp_byte_cnt + 1;
          end
        end else if (arp_rx_udid && arp_byte_cnt <= UDID_BYTES) begin
          // Accumulate UDID_BYTES UDID bytes (bytes 2..UDID_BYTES+1)
          arp_rx_udid_buf <= {arp_rx_udid_buf[UDID_WIDTH-9:0], shift_reg[7:0]};
          arp_byte_cnt <= arp_byte_cnt + 1;
        end else if (arp_rx_udid && arp_byte_cnt == UDID_BYTES + 1) begin
          // Byte UDID_BYTES+2 = new address byte
          arp_new_addr  <= shift_reg[7:1];
          arp_byte_cnt  <= arp_byte_cnt + 1;
          arp_rx_udid   <= 1'b0;
        end else begin
          arp_byte_cnt <= arp_byte_cnt + 1;
        end
      end

      // ---------------------------------------------------------
      // ARP command processing at STOP
      // ---------------------------------------------------------
      if (SMARP_ENABLE && stop_detected && (def_addr || own_arp_write)) begin
        case (arp_cmd) // synopsys parallel_case
          ARP_CMD_PREPARE_TO_ARP: begin
            av            <= 1'b0;
            assigned_addr <= SLAVE_ADDR;
          end
          ARP_CMD_ASSIGN_ADDR: begin
            if (arp_rx_udid_buf == UDID) begin
              assigned_addr <= arp_new_addr;
              av            <= 1'b1;
            end
          end
          ARP_CMD_RESET_DEVICE,
          ARP_CMD_RESET_DEV_DIR: begin
            av            <= 1'b0;
            assigned_addr <= SLAVE_ADDR;
          end
          default: ;
        endcase
      end
    end
  end

  // ---------------------------------------------------------------------------
  // SMBus PEC CRC-8 Instance
  // ---------------------------------------------------------------------------
  // CRC computed over ALL received bytes (address + data + PEC).
  // At STOP, the CRC should be 0x00 if the PEC byte was correct.
  // Property: CRC(message || CRC(message)) = 0
  // ---------------------------------------------------------------------------
  logic        pec_data_valid;
  logic [7:0]  pec_data_byte;
  logic [7:0]  pec_crc_value;

  smbus_pec u_pec (
    .clk         (clk),
    .rst_n       (rst_n),
    .reset_crc   (start_detected),  // Reset CRC at each START
    .data_valid  (pec_data_valid),
    .data_byte   (pec_data_byte),
    .pec_out     (pec_crc_value)
  );

  // Feed received bytes to CRC computation
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pec_data_valid <= 1'b0;
      pec_data_byte  <= '0;
    end else begin
      pec_data_valid <= 1'b0;  // One-cycle pulse by default

      if (SMBUS_PEC_ENABLE) begin
        // Feed address byte (fully captured at end of address phase)
        if (state == S_ADDR && bit_pos == 8 && scl_falling) begin
          pec_data_valid <= 1'b1;
          pec_data_byte  <= shift_reg[7:0];  // {addr[6:0], rw}
        end
        // Feed data/PEC bytes (every byte received in write mode)
        if (state == S_RX_DATA && bit_pos == 8 && scl_falling) begin
          pec_data_valid <= 1'b1;
          pec_data_byte  <= shift_reg[7:0];
        end
      end
    end
  end

  // PEC error: at STOP, CRC should be 0x00 for valid PEC.
  //
  // Must clear only on start_detected (a new transaction beginning), not on
  // state==S_IDLE — the FSM returns to S_IDLE the very cycle after
  // stop_detected fires, so gating the clear on S_IDLE wiped the verdict
  // one cycle after it was set, making pec_error observable for exactly one
  // clock edge (effectively unreadable by anything outside the RTL).
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      pec_error <= 1'b0;
    else if (SMBUS_PEC_ENABLE && stop_detected)
      pec_error <= (pec_crc_value != 8'h00);  // != 0 means CRC mismatch
    else if (start_detected)
      pec_error <= 1'b0;
  end

  // ---------------------------------------------------------------------------
  // Debug
  // ---------------------------------------------------------------------------
  // synthesis translate_off
  logic [2:0] slv_fsm_state_dbg;
  assign slv_fsm_state_dbg = state;

  // Debug: print slave state and shift_reg on each SCL edge
`ifdef DEBUG
  always @(posedge clk) begin
    if (scl_rising) begin
      $display("[%0t] SLV: scl_rising state=%0d bit_pos=%0d shift_reg=%b sda_i=%b sda_oe=%b capture=%b",
               $time, state, bit_pos, shift_reg, sda_i, sda_oe,
               (scl_rising && (state == S_ADDR || state == S_RX_DATA) && (bit_pos != 0)));
    end
    if (scl_falling) begin
      $display("[%0t] SLV: scl_falling state=%0d bit_pos=%0d my_addr=%b should_ack=%b sda_i=%b shift_reg=%b sda_oe=%b",
               $time, state, bit_pos, my_addr, should_ack, sda_i, shift_reg, sda_oe);
      if (state == S_ADDR && (bit_pos == 7 || bit_pos == 8))
        $display("[%0t] SLV: *** ACK EVAL: state=%0d bit_pos=%0d should_ack=%b sda_i=%b sda_oe=%b shift_reg[7:1]=%b SLAVE=%b match=%b",
                 $time, state, bit_pos, should_ack, sda_i, sda_oe, shift_reg[7:1], SLAVE_ADDR,
                 (shift_reg[7:1] == SLAVE_ADDR));
    end
  end
`endif
  // synthesis translate_on

endmodule : i2c_slave
