//==============================================================================
// i2c_master.sv
// I2C Bus Master Controller
//
// Features:
//   - 7-bit addressing (expandable to 10-bit)
//   - Standard (100 kHz) and Fast (400 kHz) mode operation
//   - Read/write transactions with multiple bytes
//   - Repeated START condition support
//   - Configurable clock prescaler
//   - Bus busy detection
//   - ACK/NACK detection
//   - Timeout detection on SCL stretching
//
// Interface: Simple host interface with start/enable control.
// SCL/SDA are treated as bidirectional signals with separate output
// enable controls for tristate operation.
//==============================================================================

`timescale 1ns / 1ps

import i2c_pkg::*;

module i2c_master #(
  // System clock frequency in Hz (for timing calculations)
  parameter int CLK_FREQ      = 50_000_000,
  // I2C SCL frequency in Hz (max 400_000 for Fast-mode)
  parameter int SCL_FREQ      = 100_000,
  // Number of bytes per transaction (burst length)
  parameter int BURST_LENGTH  = 1,
  // SMBus 3.0 compatibility mode (enables 35 ms timeout, t_HD;DAT hold)
  parameter bit  SMBUS_ENABLE    = 1'b0,
  // SMBus 3.0 PEC (Packet Error Checking) — CRC-8 appended after data
  parameter bit  SMBUS_PEC_ENABLE = 1'b0
) (
  // ---------------------------------------------------------------------------
  // System Interface
  // ---------------------------------------------------------------------------
  input  logic       clk,
  input  logic       rst_n,

  // ---------------------------------------------------------------------------
  // Host Control Interface (simple memory-mapped style)
  // ---------------------------------------------------------------------------
  // Start a transaction (must hold until busy goes high)
  input  logic       start,
  // Reset the controller mid-transaction
  input  logic       reset,
  // 7-bit slave address
  input  logic [6:0] addr,
  // Read (1) or Write (0) transaction
  input  logic       rw,
  // Data byte to transmit (for writes)
  input  logic [7:0] data_tx,
  // Received data byte (for reads)
  output logic [7:0] data_rx,
  // Number of bytes to transfer (1-256, 0 = 256)
  input  logic [7:0] num_bytes,

  // ---------------------------------------------------------------------------
  // Status Outputs
  // ---------------------------------------------------------------------------
  // Controller busy (transaction in progress)
  output logic       busy,
  // Transaction complete (single pulse)
  output logic       done,
  // ACK error (slave did not acknowledge)
  output logic       ack_error,
  // Arbitration lost (another master is driving the bus)
  output logic       arbitration_lost,
  // SMBus timeout (SCL low > 35 ms)
  output logic       smbus_timeout,
  // SMBus PEC error (CRC mismatch)
  output logic       pec_error,
  // Bus is busy (START detected, no STOP yet)
  input  logic       bus_busy,

  // ---------------------------------------------------------------------------
  // I2C Bus Interface (bidirectional with tristate controls)
  // ---------------------------------------------------------------------------
  output logic       scl_o,      // SCL output value
  output logic       scl_oe,     // SCL output enable (1 = driving)
  input  logic       scl_i,      // SCL input (monitored)
  output logic       sda_o,      // SDA output value
  output logic       sda_oe,     // SDA output enable (1 = driving)
  input  logic       sda_i       // SDA input (monitored)
);

  // ---------------------------------------------------------------------------
  // Clock Divider Calculation (Asymmetric for Fast mode compliance)
  // ---------------------------------------------------------------------------
  // SCL period = (SCL_LOW_CNT + SCL_HIGH_CNT + 2) * clk_period
  //
  // I2C spec requires minimum t_LOW:
  //   Standard mode (<= 100 kHz): t_LOW >= 4.7 us
  //   Fast mode    (400 kHz):     t_LOW >= 1.3 us
  //
  // With a symmetric 50% duty divider, t_LOW = period/2.
  // For 100 kHz:  period=10 us, t_LOW=5.0 us   >= 4.7 ✓ (symmetric)
  // For 400 kHz:  period=2.5 us, t_LOW=1.25 us  <  1.3 ✗ (too short)
  //
  // The asymmetric divider solves this by counting separate values for
  // the SCL low and high phases, ensuring t_LOW always meets the spec.
  //
  //   LOW_CYCLES  = max(spec_min_low_cycles, total_cycles / 2)
  //   HIGH_CYCLES = max(total_cycles - LOW_CYCLES, 1)  // clamp >= 1
  //   SCL_LOW_CNT  = LOW_CYCLES - 1
  //   SCL_HIGH_CNT = HIGH_CYCLES - 1
  // ---------------------------------------------------------------------------
  localparam int TOTAL_CYCLES   = CLK_FREQ / SCL_FREQ;
  // spec_min_low_cycles = ceil(t_LOW_min * CLK_FREQ) using integer ceiling division
  localparam int MIN_LOW_CYCLES = (SCL_FREQ <= 100_000) ?
      (47 * CLK_FREQ + 10_000_000 - 1) / 10_000_000 :
      (13 * CLK_FREQ + 10_000_000 - 1) / 10_000_000;
  // Use at least 50% duty when MIN_LOW_CYCLES allows it
  localparam int LOW_CYCLES     = (MIN_LOW_CYCLES > TOTAL_CYCLES / 2) ?
                                   MIN_LOW_CYCLES : TOTAL_CYCLES / 2;
  // Defensive clamp: ensure at least 1 high cycle to prevent negative counter
  localparam int HIGH_CYCLES    = (TOTAL_CYCLES > LOW_CYCLES) ?
                                   (TOTAL_CYCLES - LOW_CYCLES) : 1;
  localparam int SCL_LOW_CNT    = LOW_CYCLES  - 1;
  localparam int SCL_HIGH_CNT   = HIGH_CYCLES - 1;

  // ---------------------------------------------------------------------------
  // Internal Signals
  // ---------------------------------------------------------------------------
  master_state_t state, next_state;

  // Bit counter for data/address
  integer        bit_cnt;
  logic          last_bit;       // Last bit of current byte

  // Byte counter for burst
  integer        byte_cnt;
  logic          last_byte;

  // Phase tracking: 1 = address phase, 0 = data phase
  logic          addr_phase;

  // SCL stretching timeout counter (short, for slave clock stretching)
  integer        timeout_cnt;
  localparam int TIMEOUT_LIMIT = 16;  // ~16 clk cycles

  // SMBus 3.0 35 ms timeout counter (SCL low timeout for bus hang detection)
  // 35 ms / clk_period = 35 * CLK_FREQ / 1000 clock cycles
  // For 50 MHz: 35 * 50_000_000 / 1000 = 1_750_000
  localparam int SMBUS_TIMEOUT_CYCLES = (35 * CLK_FREQ) / 1000;
  integer        smbus_to_cnt;

  // SMBus 3.0 t_HD;DAT >= 300 ns SDA hold time
  // Calculated as: ceil(300 ns * CLK_FREQ) without 32-bit overflow
  // Using: (CLK_FREQ * 3 + 10_000_000 - 1) / 10_000_000
  // For 50 MHz: (150_000_000 + 9_999_999) / 10_000_000 = 15
  localparam int SMBUS_SDA_HOLD_CYCLES = (SMBUS_ENABLE) ?
      (CLK_FREQ * 3 + 10_000_000 - 1) / 10_000_000 : 0;
  logic [7:0]    sda_hold_cnt;   // Hold counter (enough for 15 at 50 MHz)
  logic          sda_o_cmb;      // Combinational desired SDA value
  logic          sda_oe_cmb;     // Combinational desired SDA OE

  // Shift register for TX/RX data
  logic [7:0]    shift_reg;

  // Event flags for timing control (driven by stretch-aware SCL FSM)
  logic          scl_falling_event;
  logic          scl_rising_event;

  // ---------------------------------------------------------------------------
  // SCL Clock Generation — Stretch-Aware Asymmetric Divider
  // ---------------------------------------------------------------------------
  // Extends the original scl_gen toggling divider with stretch-aware SCL
  // release. During the SCL low→high transition, SCL is released (high-Z)
  // so the slave can stretch the clock by holding it low. The divider
  // freezes during the stretch and continues when scl_i goes high.
  //
  // scl_falling_event: fires on SCL_HIGH→SCL_LOW transition (same as original)
  // scl_rising_event:  fires when scl_i goes high during SCL_WAIT
  //                    (delayed by stretch, unlike original internal timer)
  //
  // SCL output is driven low during SCL_LOW, released during SCL_WAIT,
  // and driven high during SCL_HIGH.
  // ---------------------------------------------------------------------------

  // SCL phase encoding
  typedef enum logic [1:0] {
    SCL_HIGH  = 2'b10,  // SCL driven high, counting HIGH_CNT
    SCL_LOW   = 2'b00,  // SCL driven low, counting LOW_CNT
    SCL_WAIT  = 2'b01   // SCL released, waiting for scl_i rise (stretch)
  } scl_phase_t;

  scl_phase_t scl_phase;
  integer     clk_cnt;
  logic       scl_gen;     // Legacy: follows SCL output value

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      clk_cnt  <= 0;
      scl_gen  <= 1'b1;
      scl_phase <= SCL_HIGH;
      scl_falling_event <= 1'b0;
      scl_rising_event  <= 1'b0;
    end else begin
      // Defaults: events pulse off
      scl_falling_event <= 1'b0;
      scl_rising_event  <= 1'b0;

      if (state == MST_IDLE) begin
        // Idle: SCL high, divider reset
        clk_cnt   <= 0;
        scl_gen   <= 1'b1;
        scl_phase <= SCL_HIGH;

      end else if (state == MST_START) begin
        // START: time high phase, then generate falling event
        if (clk_cnt >= SCL_HIGH_CNT) begin
          clk_cnt   <= 0;
          scl_gen   <= 1'b0;
          scl_phase <= SCL_LOW;
          scl_falling_event <= 1'b1;
        end else begin
          clk_cnt <= clk_cnt + 1;
        end

      end else begin
        // Active: run 3-phase SCL generator
        case (scl_phase)

          SCL_HIGH: begin
            if (clk_cnt >= SCL_HIGH_CNT) begin
              clk_cnt  <= 0;
              scl_gen  <= 1'b0;
              scl_phase <= SCL_LOW;
              scl_falling_event <= 1'b1;
            end else begin
              clk_cnt <= clk_cnt + 1;
            end
          end

          SCL_LOW: begin
            if (clk_cnt >= SCL_LOW_CNT) begin
              clk_cnt   <= 0;
              // Release SCL and check for stretch
              scl_phase <= SCL_WAIT;
            end else begin
              clk_cnt <= clk_cnt + 1;
            end
          end

          SCL_WAIT: begin
            // Released SCL. Check scl_i directly (no synchronizer to
            // avoid 2-cycle delay skewing FSM event timing).
            if (scl_i) begin
              scl_gen   <= 1'b1;
              scl_phase <= SCL_HIGH;
              scl_rising_event <= 1'b1;
            end
            // else: slave stretching — wait
          end

          default: begin
            scl_phase <= SCL_HIGH;
            scl_gen   <= 1'b1;
            clk_cnt   <= 0;
          end
        endcase
      end
    end
  end

  // SCL phase output values for combinational SCL driver
  logic scl_fsm_o, scl_fsm_oe;
  always_comb begin
    case (scl_phase)
      SCL_LOW:  begin scl_fsm_o = 1'b0; scl_fsm_oe = 1'b1; end  // Driven low
      SCL_WAIT: begin scl_fsm_o = 1'b1; scl_fsm_oe = 1'b0; end  // Released
      SCL_HIGH: begin scl_fsm_o = 1'b1; scl_fsm_oe = 1'b1; end  // Driven high
      default:  begin scl_fsm_o = 1'b1; scl_fsm_oe = 1'b0; end
    endcase
  end

  // ---------------------------------------------------------------------------
  // SCL Stretching Detection (the slave holds SCL low)
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      timeout_cnt <= 0;
    else if (state == MST_WAIT_ACK || state == MST_READ_DATA)
      // Check if SCL is being stretched low
      if (!scl_i) begin
        if (timeout_cnt < TIMEOUT_LIMIT)
          timeout_cnt <= timeout_cnt + 1;
      end else
        timeout_cnt <= 0;
    else
      timeout_cnt <= 0;
  end

  // ---------------------------------------------------------------------------
  // SMBus 3.0 35 ms SCL Low Timeout
  // ---------------------------------------------------------------------------
  // SMBus 3.0 requires that a device timeout if SCL is held low for more
  // than 35 ms (t_TIMEOUT). This prevents a single device from hanging
  // the bus indefinitely. The timeout counter runs during any transaction
  // when SCL is sampled low. When the timeout fires, the FSM is forced
  // to MST_IDLE (handled in the FSM next-state logic).
  //
  // Note: SCL-low time from slave clock stretching also counts toward
  // this timeout per SMBus spec. tb_i2c_smbus.sv's basic stretch tests
  // disable SMBUS_ENABLE to isolate the stretch mechanism itself;
  // tb_i2c_smbus_stretch_timeout.sv covers the real interaction —
  // a genuine slave stretch well under 35 ms completes normally, and
  // one held past 35 ms correctly triggers this timeout.
  //
  // smbus_timeout must clear when SCL actually recovers (goes high again),
  // not merely because the FSM gave up and returned to MST_IDLE — the
  // FSM's own state register jumps to MST_IDLE *directly because of*
  // smbus_timeout (see FSM State Register below), so a clear condition
  // keyed on "state == MST_IDLE" wiped the flag within 2-3 cycles of it
  // ever being set, regardless of whether SCL was still actually stuck
  // low. That's the same self-referential trap that corrupted
  // i2c_slave.sv's pec_error (fixed elsewhere this session): a real host
  // polling mst_smbus_timeout at ordinary software speed would very likely
  // never observe a flag that only survives a few clocks. Once latched,
  // smbus_timeout now holds regardless of FSM state until scl_i is
  // actually sampled high, which both fixes the visibility window and
  // correctly reflects the real underlying condition (SCL still hung).
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      smbus_to_cnt <= 0;
      smbus_timeout <= 1'b0;
    end else if (!SMBUS_ENABLE) begin
      smbus_to_cnt <= 0;
      smbus_timeout <= 1'b0;
    end else if (smbus_timeout) begin
      // Latched: stays asserted until SCL is actually released, no matter
      // what the FSM state is doing in the meantime.
      smbus_to_cnt <= 0;
      if (scl_i)
        smbus_timeout <= 1'b0;
    end else if (state != MST_IDLE) begin
      // Not yet timed out: count while SCL is low during an active
      // transaction.
      if (!scl_i) begin
        if (smbus_to_cnt >= SMBUS_TIMEOUT_CYCLES - 1) begin
          // 35 ms reached: assert timeout
          smbus_to_cnt <= 0;
          smbus_timeout <= 1'b1;
        end else begin
          smbus_to_cnt <= smbus_to_cnt + 1;
        end
      end else begin
        smbus_to_cnt <= 0;
      end
    end else begin
      // MST_IDLE, not currently timed out: nothing to count.
      smbus_to_cnt <= 0;
    end
  end

  // ---------------------------------------------------------------------------
  // FSM State Register
  // ---------------------------------------------------------------------------
  // SMBus 3.0 35 ms timeout forces an immediate abort to MST_IDLE from any
  // active state. This overrides the normal next_state logic because the
  // timeout indicates a bus hang (SCL stuck low), so we cannot rely on
  // SCL edge-triggered transitions to reach IDLE through the STOP sequence.
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      state <= MST_IDLE;
    else if (reset || (SMBUS_ENABLE && smbus_timeout))
      state <= MST_IDLE;  // Timeout: release bus immediately
    else
      state <= next_state;
  end

  // ---------------------------------------------------------------------------
  // FSM Next State Logic
  //
  // The I2C protocol works as follows for each byte transferred:
  //   - 8 SCL pulses for data (MSB first)
  //   - 1 SCL pulse for ACK/NACK
  // Total: 9 SCL pulses per byte.
  //
  // WRITE:  START | ADDR+W(8) | ACK(1) | DATA(8) | ACK(1) | ... | STOP
  // READ:   START | ADDR+R(8) | ACK(1) | DATA(8) | ACK(1) | ... | STOP
  //
  // The state machine advances on SCL edges:
  //   - SCL falling edge: SDA changes (data setup)
  //   - SCL rising edge:  SDA sampled (data capture)
  // ---------------------------------------------------------------------------
  always_comb begin
    next_state = state;

    case (state)

      // -----------------------------------------------------------------------
      // IDLE: Wait for a transaction request
      // -----------------------------------------------------------------------
      MST_IDLE: begin
        // SMBus timeout auto-clears when entering IDLE
        if (start && !bus_busy)
          next_state = MST_START;
      end

      // -----------------------------------------------------------------------
      // START: Generate START condition
      // SDA goes low while SCL is high.
      // Hold for at least one scl_tick (half SCL period).
      // -----------------------------------------------------------------------
      MST_START: begin
        if (scl_falling_event)
          next_state = MST_SEND_ADDR;
      end

      // -----------------------------------------------------------------------
      // SEND_ADDR: Transmit 7-bit address + R/W bit, MSB first
      // Also handles sending data bytes in write mode via address phase flag.
      // After 8 bits, move to WAIT_ACK.
      // -----------------------------------------------------------------------
      MST_SEND_ADDR: begin
        if (scl_falling_event && last_bit)
          next_state = MST_WAIT_ACK;
      end

      // -----------------------------------------------------------------------
      // WAIT_ACK: Wait for slave acknowledgment
      //   - SDA is released (slave drives it low for ACK)
      //   - Sampled on the 9th SCL rising edge
      // After ACK phase (scl_falling_event after 9th pulse):
      //   - If address phase + write: MST_SEND_DATA (master sends data)
      //   - If address phase + read:  MST_READ_DATA  (master receives data)
      //   - If data phase: go to STOP or continue (reuse SEND_ADDR/SEND_DATA)
      // -----------------------------------------------------------------------
      MST_WAIT_ACK: begin
        if (scl_falling_event) begin
          if (addr_phase) begin
            // Address phase complete: transition to data phase
            if (rw)
              next_state = MST_READ_DATA;   // Read: slave drives data
            else
              next_state = MST_SEND_ADDR;   // Write: master drives data
          end else begin
            // Data byte ACK phase complete
            if (last_byte)
              next_state = MST_STOP;        // Last byte: go to STOP
            else
              next_state = MST_SEND_ADDR;   // More bytes: continue
          end
        end
      end

      // -----------------------------------------------------------------------
      // READ_DATA: Receive data byte from slave
      //   - SDA is released (slave drives it)
      //   - Sample SDA on each SCL rising edge
      // After 8 bits, move to SEND_ACK (master drives ACK/NACK).
      // -----------------------------------------------------------------------
      MST_READ_DATA: begin
        if (scl_falling_event && last_bit)
          next_state = MST_SEND_ACK;
      end

      // -----------------------------------------------------------------------
      // SEND_ACK: Master sends ACK after receiving a data byte
      //   - Master drives SDA low (ACK) if more bytes to read
      //   - Master drives SDA high (NACK) if last byte
      // After ACK phase (scl_falling_event):
      //   - If more bytes: MST_READ_DATA (continue reading)
      //   - If last byte:  MST_STOP        (done reading)
      // -----------------------------------------------------------------------
      MST_SEND_ACK: begin
        if (scl_falling_event) begin
          if (last_byte)
            next_state = MST_STOP;
          else
            next_state = MST_READ_DATA;
        end
      end

      // -----------------------------------------------------------------------
      // STOP: Generate STOP condition
      //   - SDA goes high while SCL is high
      //   - First drive SCL low, then release SCL, then release SDA
      // -----------------------------------------------------------------------
      MST_STOP: begin
        if (scl_falling_event)
          next_state = MST_WAIT_STOP;
      end

      // -----------------------------------------------------------------------
      // WAIT_STOP: Hold STOP condition for one SCL period, then return to IDLE
      // -----------------------------------------------------------------------
      MST_WAIT_STOP: begin
        if (scl_falling_event)
          next_state = MST_IDLE;
      end

      default: next_state = MST_IDLE;
    endcase
  end

  // ---------------------------------------------------------------------------
  // Phase Tracking (address phase vs data phase)
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      addr_phase <= 1'b1;
    else if (state == MST_IDLE)
      addr_phase <= 1'b1;  // Start in address phase
    else if ((state == MST_WAIT_ACK) && scl_falling_event && addr_phase)
      addr_phase <= 1'b0;  // Address ACK done, switch to data phase
  end

  // ---------------------------------------------------------------------------
  // Bit and Byte Counters
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bit_cnt  <= 0;
      byte_cnt <= 0;
    end else begin
      // Reset bit counter
      if (state == MST_IDLE)
        bit_cnt <= 0;
      else if ((state == MST_SEND_ADDR || state == MST_READ_DATA) &&
                scl_falling_event && last_bit)
        bit_cnt <= 0;  // Reset after last bit sent
      else if ((state == MST_SEND_ADDR || state == MST_READ_DATA) &&
                scl_falling_event)
        bit_cnt <= bit_cnt + 1;  // Increment on each bit
      else if (state == MST_WAIT_ACK || state == MST_SEND_ACK)
        bit_cnt <= 0;  // Ensure reset during ACK phase

      // Byte counter
      if (state == MST_IDLE)
        byte_cnt <= 0;
      else if ((state == MST_WAIT_ACK || state == MST_SEND_ACK) &&
                scl_falling_event) begin
        // After ACK/NACK phase, byte is complete
        byte_cnt <= byte_cnt + 1;
      end
    end
  end

  assign last_bit  = (bit_cnt == 7);
  // PEC adds one extra byte (the CRC) AFTER all data bytes, but only for
  // WRITE transactions (!rw). Reads do not append PEC in this implementation.
  // last_byte comparison uses byte_cnt BEFORE increment.
  // byte_cnt includes the address byte as count 0.
  // For writes with PEC: total bytes = num_data_bytes + 1 (PEC)
  // For reads or no PEC:       total bytes = num_data_bytes
  assign last_byte = (byte_cnt >= num_bytes + ((SMBUS_PEC_ENABLE && !rw) ? 1 : 0));

  // ---------------------------------------------------------------------------
  // SMBus PEC CRC-8 Instance
  // ---------------------------------------------------------------------------
  logic        pec_data_valid;
  logic [7:0]  pec_data_byte;
  logic [7:0]  pec_value;

  // PEC CRC reset: pulse on rising edge of MST_START (one cycle per transaction)
  logic pec_start_d1;
  logic pec_reset_pulse;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      pec_start_d1 <= 1'b0;
    else
      pec_start_d1 <= (state == MST_START);
  end
  assign pec_reset_pulse = SMBUS_PEC_ENABLE && (state == MST_START) && !pec_start_d1;

  smbus_pec u_pec (
    .clk         (clk),
    .rst_n       (rst_n),
    .reset_crc   (pec_reset_pulse),
    .data_valid  (pec_data_valid),
    .data_byte   (pec_data_byte),
    .pec_out     (pec_value)
  );

  // Latches "address byte already fed to CRC this transaction". MST_START
  // holds for a full SCL half-period (~250 clocks at 100kHz/50MHz), so a
  // transient !pec_data_valid guard re-arms every other cycle and re-feeds
  // the address dozens of times before MST_SEND_ADDR is even reached,
  // corrupting the CRC long before any real data byte arrives. This latch
  // fires exactly once per transaction, clearing on the next reset_crc pulse.
  logic pec_addr_fed;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      pec_addr_fed <= 1'b0;
    else if (pec_reset_pulse)
      pec_addr_fed <= 1'b0;
    else if ((state == MST_START) && pec_start_d1 && !pec_addr_fed)
      pec_addr_fed <= 1'b1;
  end

  // PEC data feeding: feed address and data bytes to CRC
  // Address is fed one cycle after reset_crc (to give the CRC register
  // time to clear before the first byte is XOR'd in). The address byte
  // is loaded into shift_reg at MST_START, so we feed the CRC on the
  // SECOND cycle of MST_START (after reset_crc pulse).
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pec_data_valid <= 1'b0;
      pec_data_byte  <= '0;
    end else begin
      pec_data_valid <= 1'b0;  // One-cycle pulse by default

      if (SMBUS_PEC_ENABLE) begin
        // Feed address byte: one cycle after reset, on the second MST_START cycle
        // pec_start_d1 is 1 from cycle 2 of MST_START onwards
        if ((state == MST_START) && pec_start_d1 && !pec_addr_fed) begin
          pec_data_valid <= 1'b1;
          pec_data_byte  <= {addr, rw};
        end
        // Feed each data byte to the CRC at the exact edge it's loaded into
        // shift_reg for transmission — not later, at that same byte's own
        // ACK completion. u_pec's crc_reg takes one extra cycle to register
        // a fed byte's contribution after data_valid asserts; feeding "the
        // byte just sent" at its own ACK-completion edge collided with the
        // *same* edge that (for the last data byte) loads shift_reg with
        // pec_value for transmission — pec_value was read one cycle before
        // it had absorbed that very byte, so the master transmitted a
        // one-byte-stale PEC. Feeding at load time instead (one full byte
        // period earlier, mirroring the shift_reg reload conditions below)
        // gives crc_reg a full byte period of margin to settle before
        // pec_value is ever needed.
        else if (state == MST_WAIT_ACK && scl_falling_event && addr_phase && !rw) begin
          pec_data_valid <= 1'b1;
          pec_data_byte  <= data_tx;
        end
        else if (state == MST_WAIT_ACK && scl_falling_event && !addr_phase && !rw &&
                 !(byte_cnt >= num_bytes)) begin
          pec_data_valid <= 1'b1;
          pec_data_byte  <= data_tx;
        end
      end
    end
  end

  // PEC error: detected when slave NACKs the PEC byte
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      pec_error <= 1'b0;
    else if (SMBUS_PEC_ENABLE && state == MST_WAIT_ACK && scl_rising_event &&
             byte_cnt >= num_bytes && !addr_phase)
      pec_error <= sda_i;  // NACK on PEC byte = PEC error
    else if (state == MST_IDLE)
      pec_error <= 1'b0;
  end

  // ---------------------------------------------------------------------------
  // Shift Register & Data Path
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      shift_reg <= '0;
      data_rx   <= '0;
    end else begin
      // Load address at beginning of transaction
      if (state == MST_START) begin
        shift_reg <= {addr, rw};  // 7-bit addr + R/W bit
      end
      // Load data to send at the start of each write byte
      else if (state == MST_WAIT_ACK && scl_falling_event && !addr_phase && !rw) begin
        // Data ACK just completed in write mode
        if (SMBUS_PEC_ENABLE && byte_cnt >= num_bytes) begin
          shift_reg <= pec_value;  // Load PEC byte after all data bytes
        end else begin
          shift_reg <= data_tx;    // Load next data byte
        end
      end
      // Load first data byte when transitioning from address to data phase (write)
      else if (state == MST_WAIT_ACK && scl_falling_event && addr_phase && !rw) begin
        shift_reg <= data_tx;
      end
      // Shift in data on SCL rising edges during read
      else if (state == MST_READ_DATA && scl_rising_event) begin
        shift_reg <= {shift_reg[6:0], sda_i};
      end
      // Capture received data at end of byte
      else if (state == MST_SEND_ACK && scl_falling_event) begin
        data_rx <= shift_reg;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // SDA and SCL Output Controls (combinational)
  // ---------------------------------------------------------------------------
  // Computes the *desired* immediate SDA/SCL values based on FSM state.
  // The actual sda_o/sda_oe outputs may be delayed through a hold pipeline
  // when SMBUS_ENABLE is active (see SMBus t_HD;DAT hold logic below).
  //
  // SCL during active clocking states (SEND_ADDR, WAIT_ACK, READ_DATA,
  // SEND_ACK) is driven by the stretch-aware SCL FSM. The FSM releases SCL
  // during low→high transitions to allow slave clock stretching.
  // ---------------------------------------------------------------------------
  always_comb begin
    // Defaults: both released (high-Z, pulled high by pull-ups)
    scl_o    = 1'b1;
    scl_oe   = 1'b0;
    sda_o_cmb  = 1'b1;
    sda_oe_cmb = 1'b0;

    case (state)

      // -----------------------------------------------------------------------
      // IDLE: Release bus
      // -----------------------------------------------------------------------
      MST_IDLE: begin
        scl_o    = 1'b1;
        scl_oe   = 1'b0;
        sda_o_cmb  = 1'b1;
        sda_oe_cmb = 1'b0;
      end

      // -----------------------------------------------------------------------
      // START: SDA goes low while SCL is high
      // -----------------------------------------------------------------------
      MST_START: begin
        scl_o    = 1'b1;
        scl_oe   = 1'b1;   // Drive SCL high
        sda_o_cmb  = 1'b0;
        sda_oe_cmb = 1'b1;   // Drive SDA low (START condition)
      end

      // -----------------------------------------------------------------------
      // SEND_ADDR / SEND_DATA: Drive address or data bits onto SDA
      // Both address and data byte transmission use the same pattern.
      // SCL is driven by the stretch-aware SCL FSM.
      // -----------------------------------------------------------------------
      MST_SEND_ADDR: begin
        scl_o    = scl_fsm_o;   // Stretch-aware SCL
        scl_oe   = scl_fsm_oe;
        if (bit_cnt <= 7) begin
          sda_o_cmb  = shift_reg[7 - bit_cnt];  // MSB first
          sda_oe_cmb = 1'b1;   // Drive SDA with data bit
        end else begin
          sda_o_cmb  = 1'b1;
          sda_oe_cmb = 1'b0;   // Release for ACK (handled in WAIT_ACK)
        end
      end

      // -----------------------------------------------------------------------
      // WAIT_ACK: Release SDA to let slave drive ACK/NACK
      // SCL continues toggling via the stretch-aware SCL FSM.
      // -----------------------------------------------------------------------
      MST_WAIT_ACK: begin
        scl_o    = scl_fsm_o;   // Stretch-aware SCL
        scl_oe   = scl_fsm_oe;
        sda_o_cmb  = 1'b1;
        sda_oe_cmb = 1'b0;   // Release SDA (slave drives ACK)
      end

      // -----------------------------------------------------------------------
      // READ_DATA: Release SDA, slave drives data, master samples
      // SCL continues toggling via the stretch-aware SCL FSM.
      // -----------------------------------------------------------------------
      MST_READ_DATA: begin
        scl_o    = scl_fsm_o;   // Stretch-aware SCL
        scl_oe   = scl_fsm_oe;
        sda_o_cmb  = 1'b1;
        sda_oe_cmb = 1'b0;   // Release SDA (slave drives data)
      end

      // -----------------------------------------------------------------------
      // SEND_ACK: Master drives ACK (low) or NACK (high) after receiving a byte
      // ACK  = SDA low  = proceed with more bytes
      // NACK = SDA high = last byte, slave releases bus
      // SCL continues toggling via the stretch-aware SCL FSM.
      // -----------------------------------------------------------------------
      MST_SEND_ACK: begin
        scl_o    = scl_fsm_o;   // Stretch-aware SCL
        scl_oe   = scl_fsm_oe;
        // Send NACK if last byte, ACK otherwise
        sda_o_cmb  = last_byte ? NACK : ACK;
        sda_oe_cmb = 1'b1;   // Drive SDA with ACK/NACK
      end

      // -----------------------------------------------------------------------
      // STOP: Generate STOP condition
      //
      // I2C STOP protocol:
      //   1. SCL is HIGH
      //   2. SDA transitions from LOW to HIGH (while SCL is high)
      //
      // Sequence:
      //   - MST_STOP:    Drive SDA LOW, release SCL (pullup keeps SCL high)
      //   - MST_WAIT_STOP: Release SDA → SDA rises while SCL is high = STOP
      // -----------------------------------------------------------------------
      MST_STOP: begin
        scl_o    = 1'b1;
        scl_oe   = 1'b0;   // Release SCL (pull-up keeps it high)
        sda_o_cmb  = 1'b0;   // Drive SDA low to prepare for STOP
        sda_oe_cmb = 1'b1;   // Drive SDA low
      end

      // -----------------------------------------------------------------------
      // WAIT_STOP: Complete the STOP condition by releasing SDA
      // SDA goes high via pull-up while SCL is high → STOP condition detected
      // -----------------------------------------------------------------------
      MST_WAIT_STOP: begin
        scl_o    = 1'b1;
        scl_oe   = 1'b0;   // SCL released (pull-up keeps it high)
        sda_o_cmb  = 1'b1;
        sda_oe_cmb = 1'b0;   // Release SDA → pull-up makes it high → STOP!
      end

      default: begin
        scl_o    = 1'b1;
        scl_oe   = 1'b0;
        sda_o_cmb  = 1'b1;
        sda_oe_cmb = 1'b0;
      end
    endcase
  end

  // ---------------------------------------------------------------------------
  // SMBus t_HD;DAT SDA Hold Time (registered pipeline)
  // ---------------------------------------------------------------------------
  // SMBus 3.0 requires SDA to remain valid for at least 300 ns after SCL
  // falls (t_HD;DAT). This is implemented by delaying SDA output changes
  // after each SCL falling edge. On scl_falling_event, a hold counter is
  // started. While the counter is active, the SDA outputs retain their
  // previous values. Once the counter reaches 0, the combinational values
  // pass through to the outputs.
  // ---------------------------------------------------------------------------
  // The hold counter reset on each new scl_falling ensures the hold time
  // is measured from the most recent SCL edge. This keeps SDA stable
  // through the hold window into the SCL low period.
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sda_hold_cnt <= 0;
      sda_o  <= 1'b1;
      sda_oe <= 1'b0;
    end else begin
      // Default: hold counter inactive, pass combinational through
      sda_hold_cnt <= 0;
      sda_o  <= sda_o_cmb;
      sda_oe <= sda_oe_cmb;

      if (SMBUS_ENABLE && sda_hold_cnt > 0) begin
        // Hold active: keep previous output values, decrement counter
        sda_hold_cnt <= sda_hold_cnt - 1;
        sda_o  <= sda_o;   // Retain previous
        sda_oe <= sda_oe;  // Retain previous
      end else if (SMBUS_ENABLE && scl_falling_event &&
                  (state == MST_SEND_ADDR || state == MST_SEND_ACK)) begin
        // SCL falling during active data driving: start hold timer
        sda_hold_cnt <= SMBUS_SDA_HOLD_CYCLES;
        sda_o  <= sda_o;   // Freeze SDA at current value
        sda_oe <= sda_oe;  // Freeze SDA OE at current value
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Status Outputs
  // ---------------------------------------------------------------------------

  // Busy: active from START to STOP
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      busy <= 1'b0;
    else if (state == MST_START)
      busy <= 1'b1;
    else if (state == MST_IDLE)
      busy <= 1'b0;
  end

  // Done: pulsed at end of transaction (when entering IDLE from WAIT_STOP)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      done <= 1'b0;
    else
      done <= (state == MST_WAIT_STOP && scl_falling_event);
  end

  // ACK Error: sampled on SCL rising edge during WAIT_ACK
  // SDA low  (0) = ACK  (no error)
  // SDA high (1) = NACK (error)
  // Cleared on MST_START (beginning of transaction) so that the value
  // persists through MST_IDLE and can be sampled by the testbench after
  // the transaction completes.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      ack_error <= 1'b0;
    else if (state == MST_START)
      ack_error <= 1'b0;
    else if (state == MST_WAIT_ACK && scl_rising_event)
      ack_error <= sda_i;  // 1 = NACK = error
  end

  // Arbitration Lost detection:
  // If we're driving SDA high but sample it low, another master is driving.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      arbitration_lost <= 1'b0;
    else if (state == MST_IDLE)
      arbitration_lost <= 1'b0;
    else if ((state == MST_SEND_ADDR) && scl_rising_event)
      if (sda_oe && sda_o && !sda_i)
        arbitration_lost <= 1'b1;
  end

  // ---------------------------------------------------------------------------
  // Debug Output (for waveform inspection)
  // ---------------------------------------------------------------------------
  // synthesis translate_off
  logic [3:0] fsm_state_dbg;
  assign fsm_state_dbg = state;

`ifdef DEBUG
  always @(posedge clk) begin
    if (state == MST_WAIT_ACK && scl_rising_event) begin
      $display("[%0t] MST: ACK SAMPLE state=%0d sda_i=%b ack_error=%b scl_rising_event=%b",
               $time, state, sda_i, ack_error, scl_rising_event);
      $display("[%0t] MST:   scl_o=%b scl_oe=%b sda_o=%b sda_oe=%b",
               $time, scl_o, scl_oe, sda_o, sda_oe);
    end
    if (state == MST_SEND_ADDR) begin
      if (scl_falling_event && last_bit)
        $display("[%0t] MST: SEND_ADDR last_bit, transitioning to WAIT_ACK", $time);
      if (scl_falling_event && addr_phase == 0)
        $display("[%0t] MST: SEND_DATA scl_falling_event bit_cnt=%0d shift_reg=%b sda_o=%b sda_oe=%b",
                 $time, bit_cnt, shift_reg, sda_o, sda_oe);
    end
    if (state == MST_WAIT_ACK && scl_falling_event) begin
      $display("[%0t] MST: WAIT_ACK falling event, addr_phase=%b rw=%b last_byte=%b shift_reg(prev)=%b data_tx=%b",
               $time, addr_phase, rw, last_byte, shift_reg, data_tx);
    end
  end
`endif
  // synthesis translate_on

endmodule : i2c_master
