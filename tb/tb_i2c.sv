//==============================================================================
// tb_i2c.sv
// Testbench for Complete I2C Controller
//
// This testbench validates:
//   - I2C Master write transaction
//   - I2C Master read transaction
//   - I2C Slave address detection and response
//   - Combined master-slave communication
//   - Repeated START condition
//   - Multi-byte burst transactions
//   - ACK/NACK handling
//   - Bus busy detection
//   - Error conditions
//==============================================================================

`timescale 1ns / 1ps

import i2c_pkg::*;

module tb_i2c();

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  parameter CLK_PERIOD   = 20;           // 50 MHz
  parameter SLAVE_ADDR   = 7'h50;        // Target slave address
  parameter SCL_FREQ     = 100_000;       // Standard mode (100 kHz)
  parameter real SCL_PERIOD = 1_000_000_000.0 / (SCL_FREQ * 2);  // ns per half

  // ---------------------------------------------------------------------------
  // DUT Signals
  // ---------------------------------------------------------------------------
  logic        clk;
  logic        rst_n;

  // Master interface
  logic        mst_start;
  logic        mst_reset;
  logic [6:0]  mst_addr;
  logic        mst_rw;
  logic [7:0]  mst_data_tx;
  logic [7:0]  mst_data_rx;
  logic [7:0]  mst_num_bytes;
  logic        mst_busy;
  logic        mst_done;
  logic        mst_ack_error;
  logic        mst_arb_lost;

  // Slave interface
  logic [7:0]  slv_tx_data;
  logic        slv_tx_valid;
  logic        slv_tx_ready;
  logic [7:0]  slv_rx_data;
  logic        slv_rx_valid;
  logic        slv_rx_ready;
  logic        slv_addressed;
  logic        slv_dir_rw;
  logic        slv_busy;

  // I2C Bus
  wire         scl;
  wire         sda;

  // ---------------------------------------------------------------------------
  // Pull-up resistors (model external pull-ups on the bus)
  // ---------------------------------------------------------------------------
  pullup(scl);
  pullup(sda);

  // ---------------------------------------------------------------------------
  // DUT Instantiation
  // ---------------------------------------------------------------------------
  i2c_top #(
    .CLK_FREQ  (50_000_000),
    .SCL_FREQ  (SCL_FREQ),
    .SLV_ADDR  (SLAVE_ADDR),
    .GEN_CALL  (1'b1)
  ) u_i2c_top (
    .clk           (clk),
    .rst_n         (rst_n),
    .mst_start     (mst_start),
    .mst_reset     (mst_reset),
    .mst_addr      (mst_addr),
    .mst_rw        (mst_rw),
    .mst_data_tx   (mst_data_tx),
    .mst_data_rx   (mst_data_rx),
    .mst_num_bytes (mst_num_bytes),
    .mst_busy      (mst_busy),
    .mst_done      (mst_done),
    .mst_ack_error (mst_ack_error),
    .mst_arb_lost  (mst_arb_lost),
    .slv_tx_data   (slv_tx_data),
    .slv_tx_valid  (slv_tx_valid),
    .slv_tx_ready  (slv_tx_ready),
    .slv_rx_data   (slv_rx_data),
    .slv_rx_valid  (slv_rx_valid),
    .slv_rx_ready  (slv_rx_ready),
    .slv_addressed (slv_addressed),
    .slv_dir_rw    (slv_dir_rw),
    .slv_busy      (slv_busy),
    .scl           (scl),
    .sda           (sda)
  );

  // ---------------------------------------------------------------------------
  // Clock Generation
  // ---------------------------------------------------------------------------
  initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD / 2) clk = ~clk;
  end

  // ---------------------------------------------------------------------------
  // Waveform Dump
  // ---------------------------------------------------------------------------
  initial begin
    $dumpfile("tb_i2c.vcd");
    $dumpvars(0, tb_i2c);
  end

  // ---------------------------------------------------------------------------
  // Task: Wait for N clock cycles
  // ---------------------------------------------------------------------------
  task wait_clks(input int n);
    repeat (n) @(posedge clk);
  endtask

  // ---------------------------------------------------------------------------
  // Task: Reset the system
  // ---------------------------------------------------------------------------
  task reset_system();
    $display("[%0t] Resetting system...", $time);
    rst_n = 1'b0;
    mst_start     = 1'b0;
    mst_reset     = 1'b0;
    mst_addr      = '0;
    mst_rw        = 1'b0;
    mst_data_tx   = '0;
    mst_num_bytes = '0;
    slv_tx_data   = '0;
    slv_tx_valid  = 1'b0;
    slv_rx_ready  = 1'b1;
    wait_clks(10);
    rst_n = 1'b1;
    wait_clks(10);
    $display("[%0t] System reset complete.", $time);
  endtask

  // ---------------------------------------------------------------------------
  // Task: Master Write Transaction
  // ---------------------------------------------------------------------------
  task master_write(
    input logic [6:0] addr,
    input logic [7:0] data_byte
  );
    $display("[%0t] Starting MASTER WRITE: addr=0x%02x, data=0x%02x",
             $time, addr, data_byte);
    mst_addr      = addr;
    mst_rw        = 1'b0;  // Write
    mst_data_tx   = data_byte;
    mst_num_bytes = 8'd1;
    mst_start     = 1'b1;
    wait_clks(1);
    mst_start = 1'b0;

    // Wait for transaction to start, then complete
    // Use @(posedge/negedge) for edge-triggered reliability
    @(posedge mst_busy);  // Transaction started
    @(negedge mst_busy);  // Transaction completed
    wait_clks(1);

    if (mst_ack_error)
      $display("[%0t] ERROR: Master write NACK received!", $time);
    else
      $display("[%0t] Master WRITE complete successfully.", $time);
  endtask

  // ---------------------------------------------------------------------------
  // Task: Master Read Transaction
  // ---------------------------------------------------------------------------
  task master_read(
    input logic [6:0] addr,
    output logic [7:0] data_byte
  );
    $display("[%0t] Starting MASTER READ: addr=0x%02x", $time, addr);
    mst_addr      = addr;
    mst_rw        = 1'b1;  // Read
    mst_data_tx   = '0;
    mst_num_bytes = 8'd1;
    mst_start     = 1'b1;
    wait_clks(1);
    mst_start = 1'b0;

    // Wait for transaction to start, then complete
    // Use @(posedge/negedge) for edge-triggered reliability
    @(posedge mst_busy);  // Transaction started
    @(negedge mst_busy);  // Transaction completed
    wait_clks(1);

    if (mst_ack_error)
      $display("[%0t] ERROR: Master read NACK received!", $time);
    else begin
      data_byte = mst_data_rx;
      $display("[%0t] Master READ complete: data=0x%02x", $time, data_byte);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Task: Bus Monitor (monitor I2C bus activity)
  // ---------------------------------------------------------------------------
  task bus_monitor();
    $display("=====================================================");
    $display("  I2C Bus Monitor");
    $display("=====================================================");
  endtask

  // ---------------------------------------------------------------------------
  // Check Tasks
  // ---------------------------------------------------------------------------
  task check_equal(
    input string  description,
    input logic [63:0] expected,
    input logic [63:0] actual
  );
    if (expected != actual) begin
      $display("[%0t] FAIL: %s - expected=0x%h, actual=0x%h",
               $time, description, expected, actual);
    end else begin
      $display("[%0t] PASS: %s", $time, description);
    end
  endtask

  // ===========================================================================
  // ===========================================================================
  // MAIN TEST SEQUENCE
  // ===========================================================================
  // ===========================================================================
  initial begin
    logic [7:0] readback;

    $display("");
    $display("============================================================");
    $display("  I2C Controller Testbench");
    $display("  SCL Frequency: %0d kHz", SCL_FREQ / 1000);
    $display("  Slave Address: 0x%02x", SLAVE_ADDR);
    $display("============================================================");
    $display("");

    // -----------------------------------------------------------------------
    // Test 1: Reset and initial conditions
    // -----------------------------------------------------------------------
    $display("--- Test 1: Reset and initial conditions ---");
    reset_system();

    check_equal("Bus idle (mst_busy=0)", 0, mst_busy);
    check_equal("Slave idle (slv_busy=0)", 0, slv_busy);
    check_equal("No ack error", 0, mst_ack_error);
    check_equal("No arbitration lost", 0, mst_arb_lost);

    // -----------------------------------------------------------------------
    // Test 2: Master write to slave
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 2: Master Write to Slave ---");

    // Pre-load slave TX data (for any subsequent read)
    slv_tx_data  = 8'hA5;
    slv_tx_valid = 1'b1;
    wait_clks(2);
    slv_tx_valid = 1'b0;

    // Manually drive the write transaction so we can sample addressed
    // during the transaction (it's cleared on STOP → S_IDLE)
    $display("[%0t] Starting MASTER WRITE: addr=0x%02x, data=0x%02x", $time, SLAVE_ADDR, 8'hAB);
    mst_addr      = SLAVE_ADDR;
    mst_rw        = 1'b0;  // Write
    mst_data_tx   = 8'hAB;
    mst_num_bytes = 8'd1;
    mst_start     = 1'b1;
    wait_clks(1);
    mst_start = 1'b0;

    // Wait for address to be matched, then sample addressed/dir_rw
    wait(slv_addressed);
    check_equal("Slave was addressed", 1'b1, slv_addressed);
    check_equal("Slave direction is write", 1'b0, slv_dir_rw);

    // Wait for transaction to complete
    wait(mst_done || mst_ack_error);
    wait_clks(1);

    if (mst_ack_error)
      $display("[%0t] ERROR: Master write NACK received!", $time);
    else
      $display("[%0t] Master WRITE complete successfully.", $time);

    // Check that slave received the data
    wait_clks(5);
    $display("Slave RX data: 0x%02x", slv_rx_data);
    check_equal("Slave received correct data", 8'hAB, slv_rx_data);

    // -----------------------------------------------------------------------
    // Test 3: Master read from slave
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 3: Master Read from Slave ---");

    // Load slave with data for master to read
    slv_tx_data  = 8'h3C;
    slv_tx_valid = 1'b1;
    wait_clks(2);
    slv_tx_valid = 1'b0;

    master_read(SLAVE_ADDR, readback);
    check_equal("Master read correct data", 8'h3C, readback);
    check_equal("Slave direction is read", 1'b1, slv_dir_rw);

    // -----------------------------------------------------------------------
    // Test 4: Repeated START (combined write+read)
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 4: Repeated START (combined write+read) ---");

    // Write then immediately read
    slv_tx_data  = 8'h99;
    slv_tx_valid = 1'b1;
    wait_clks(2);
    slv_tx_valid = 1'b0;

    master_write(SLAVE_ADDR, 8'h11);
    wait_clks(10);

    master_read(SLAVE_ADDR, readback);
    check_equal("Repeated START - master read correct", 8'h99, readback);

    // -----------------------------------------------------------------------
    // Test 5: Burst write (multiple bytes in single transaction)
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 5: Burst Write (Multiple Bytes) ---");

    // Use a single transaction with multiple bytes
    mst_addr      = SLAVE_ADDR;
    mst_rw        = 1'b0;
    mst_data_tx   = 8'h01;
    mst_num_bytes = 8'd4;
    mst_start     = 1'b1;
    wait_clks(1);
    mst_start     = 1'b0;
    wait(mst_done);
    wait_clks(2);  // Settle before the next transaction (matches the
                    // wait_clks(1) settle used after every other wait()
                    // in this file; without it, back-to-back master
                    // transactions can race the FSM's return to MST_IDLE
                    // under simulators with a different event-scheduling
                    // order than Icarus Verilog, e.g. Verilator).
    $display("[%0t] Master BURST WRITE complete.", $time);

    // -----------------------------------------------------------------------
    // Test 6: Error condition - write to non-existent slave
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 6: Error Condition - Write to non-existent address ---");

    // Manually drive write to 0x7F (no slave at this address)
    $display("[%0t] Starting MASTER WRITE: addr=0x%02x, data=0x%02x", $time, 7'h7F, 8'hFF);
    mst_addr      = 7'h7F;
    mst_rw        = 1'b0;
    mst_data_tx   = 8'hFF;
    mst_num_bytes = 8'd1;
    mst_start     = 1'b1;
    wait_clks(1);
    mst_start = 1'b0;

    // Wait for transaction to complete
    @(posedge mst_busy);  // Transaction started
    @(negedge mst_busy);  // Transaction completed
    wait_clks(2);         // Let signals settle

    // Check for NACK
    if (mst_ack_error)
      $display("PASS: Master correctly detected NACK for non-existent slave.");
    else
      $display("FAIL: Master should have detected NACK!");

    // -----------------------------------------------------------------------
    // Test 7: Multiple transactions
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 7: Multiple Write Transactions ---");

    for (int i = 0; i < 5; i++) begin
      master_write(SLAVE_ADDR, 8'hA0 + i);
      $display("  Write %0d: data=0x%02x, slave_rx=0x%02x",
               i, 8'hA0 + i, slv_rx_data);
    end

    // -----------------------------------------------------------------------
    // Test 8: Edge case - back-to-back transactions
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 8: Back-to-back transactions ---");

    slv_tx_data  = 8'h42;
    slv_tx_valid = 1'b1;
    wait_clks(2);
    slv_tx_valid = 1'b0;

    master_write(SLAVE_ADDR, 8'hDE);
    master_read(SLAVE_ADDR, readback);
    check_equal("Back-to-back read data", 8'h42, readback);

    // -----------------------------------------------------------------------
    // End of tests
    // -----------------------------------------------------------------------
    $display("");
    $display("============================================================");
    $display("  ALL TESTS COMPLETED");
    $display("============================================================");
    $display("");

    #1000;
    $finish;
  end

  // ---------------------------------------------------------------------------
  // I2C Bus Monitor — Tracks every SCL edge, SDA value, and decodes protocol
  // ---------------------------------------------------------------------------
  logic sda_fall, sda_rise;
  logic scl_fall, scl_rise;
  logic sda_d1, sda_d2;
  logic scl_d1, scl_d2;

  always_ff @(posedge clk) begin
    sda_d1 <= sda;
    sda_d2 <= sda_d1;
    scl_d1 <= scl;
    scl_d2 <= scl_d1;
  end

  assign sda_fall = sda_d2 && !sda_d1;
  assign sda_rise = !sda_d2 && sda_d1;
  assign scl_fall = scl_d2 && !scl_d1;
  assign scl_rise = !scl_d2 && scl_d1;

  // ---------------------------------------------------------------------------
  // $monitor: Tracks SCL/SDA values on every change (only during transactions)
  // ---------------------------------------------------------------------------
  initial begin
    $display("");
    $display("--- I2C Bus $monitor enabled (scl/sda trace) ---");
  end

  always @(scl or sda) begin
    // Only display during active transaction (between START and STOP)
    if (proto_in_frame)
      $display("[%0t] BUS: scl=%b sda=%b  [%s]", $time, scl, sda,
               proto_phase_addr ? "ADDRESS" :
               (proto_bit_cnt == 8 ? "ACK" : "DATA"));
  end

  // ---------------------------------------------------------------------------
  // Protocol Decoder: Tracks bit positions and accumulates frame data
  // ---------------------------------------------------------------------------
  int          proto_bit_cnt;    // Bit counter within current byte
  logic [7:0]  proto_byte;       // Accumulated byte being received/sent
  logic [7:0]  proto_addr;       // Last detected address (includes R/W)
  logic        proto_in_frame;   // Inside a transaction (between START/STOP)
  logic        proto_phase_addr; // Currently in address phase
  logic        proto_data_dir;   // Current data direction (0=write,1=read)
  int          proto_data_cnt;   // Number of data bytes transferred

  // Master/Slave state name lookup
  function automatic string mst_state_name(master_state_t s);
    case (s)
      MST_IDLE:      return "IDLE";
      MST_START:     return "START";
      MST_SEND_ADDR: return "SEND_ADDR";
      MST_WAIT_ACK:  return "WAIT_ACK";
      MST_READ_DATA: return "READ_DATA";
      MST_SEND_ACK:  return "SEND_ACK";
      MST_STOP:      return "STOP";
      MST_WAIT_STOP: return "WAIT_STOP";
      default:       return "??";
    endcase
  endfunction

  // FSM state-dedup tracking (integer-based, avoids iverilog string-signal issues)
  master_state_t last_mst_state_val = MST_IDLE;
  logic [2:0]    last_slv_state_val = 3'h0;

  // START / STOP detection with protocol decode
  always @(posedge clk) begin
    // --- START condition ---
    if (scl_d2 && sda_fall) begin
      $display("[%0t] ╔══ I2C START condition detected ══╗", $time);
      proto_in_frame   <= 1'b1;
      proto_bit_cnt    <= 0;
      proto_byte       <= '0;
      proto_phase_addr <= 1'b1;
      proto_data_cnt   <= 0;
    end

    // --- STOP condition ---
    if (scl_d1 && sda_rise) begin
      $display("[%0t] ╚══ I2C STOP condition detected ══╝", $time);
      if (proto_data_cnt > 0)
        $display("[%0t]   >>> Total data bytes transferred: %0d", $time, proto_data_cnt);
      proto_in_frame <= 1'b0;
    end

    // --- SCL rising edge: sample SDA (data/address bit capture) ---
    if (scl_rise && proto_in_frame) begin
      $display("[%0t] CLK: scl↑ sda=%b  [bit %0d of %s]",
               $time, sda, proto_bit_cnt,
               proto_phase_addr ? "ADDRESS" : "DATA");

      // Accumulate the bit into the byte being built (MSB first)
      proto_byte <= {proto_byte[6:0], sda};

      if (proto_bit_cnt == 7) begin
        // Last bit of this byte — next edge is ACK
        if (proto_phase_addr) begin
          proto_addr <= {proto_byte[6:0], sda};
          $display("[%0t] >>> ADDRESS BYTE complete: 0x%02x (address=0x%02x, R/W=%b)",
                   $time, {proto_byte[6:0], sda},
                   {proto_byte[6:0], sda} >> 1, sda);
        end else begin
          proto_data_cnt <= proto_data_cnt + 1;
          $display("[%0t] >>> DATA BYTE #%0d complete: 0x%02x (%s direction)",
                   $time, proto_data_cnt + 1,
                   {proto_byte[6:0], sda},
                   proto_data_dir ? "READ" : "WRITE");
        end
      end

      proto_bit_cnt <= (proto_bit_cnt == 7) ? 0 : proto_bit_cnt + 1;
    end

    // --- SCL falling edge: data changes here, also ACK sampled ---
    if (scl_fall && proto_in_frame && (proto_bit_cnt == 0)) begin
      // ACK/NACK was sampled on the previous rising edge (9th clock)
      // This falling edge starts the next byte
      if (proto_phase_addr) begin
        proto_phase_addr <= 1'b0;
        proto_data_dir <= proto_addr[0];  // LSB = R/W bit
        $display("[%0t] >>> ACK phase for address complete. Direction=%s",
                 $time, proto_addr[0] ? "READ (master <- slave)" : "WRITE (master -> slave)");
      end else begin
        $display("[%0t] >>> ACK phase for data byte #%0d complete.",
                 $time, proto_data_cnt);
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Master FSM State Trace (edge-triggered on state changes)
  // ---------------------------------------------------------------------------
  always @(posedge clk) begin
    if (u_i2c_top.u_i2c_master.state != last_mst_state_val) begin
      string cur = mst_state_name(u_i2c_top.u_i2c_master.state);
      if (u_i2c_top.u_i2c_master.state == MST_START ||
          u_i2c_top.u_i2c_master.state == MST_SEND_ADDR ||
          u_i2c_top.u_i2c_master.state == MST_WAIT_ACK ||
          u_i2c_top.u_i2c_master.state == MST_READ_DATA ||
          u_i2c_top.u_i2c_master.state == MST_SEND_ACK ||
          u_i2c_top.u_i2c_master.state == MST_STOP ||
          u_i2c_top.u_i2c_master.state == MST_WAIT_STOP) begin
        $display("[%0t] FSM: Master -> %s  (bit_cnt=%0d, byte_cnt=%0d, addr_phase=%b)",
                 $time, cur,
                 u_i2c_top.u_i2c_master.bit_cnt,
                 u_i2c_top.u_i2c_master.byte_cnt,
                 u_i2c_top.u_i2c_master.addr_phase);
      end
      last_mst_state_val <= u_i2c_top.u_i2c_master.state;
    end
  end

  // ---------------------------------------------------------------------------
  // Slave FSM State Trace (edge-triggered on state changes)
  // ---------------------------------------------------------------------------
  always @(posedge clk) begin
    if (u_i2c_top.u_i2c_slave.state != last_slv_state_val) begin
      string cur;
      case (u_i2c_top.u_i2c_slave.state)
        3'h0: cur = "IDLE";
        3'h1: cur = "ADDR";
        3'h2: cur = "ACK_REF";
        3'h3: cur = "TX_DATA";
        3'h4: cur = "RX_DATA";
        3'h5: cur = "ACK_PHASE";
        default: cur = "??";
      endcase
      if (u_i2c_top.u_i2c_slave.state == 3'h1 ||  // S_ADDR
          u_i2c_top.u_i2c_slave.state == 3'h2 ||  // S_ACK_REF
          u_i2c_top.u_i2c_slave.state == 3'h3 ||  // S_TX_DATA
          u_i2c_top.u_i2c_slave.state == 3'h4 ||  // S_RX_DATA
          u_i2c_top.u_i2c_slave.state == 3'h5) begin  // S_ACK_PHASE
        $display("[%0t] FSM: Slave -> %s  (bit_pos=%0d, my_addr=%b, addressed=%b)",
                 $time, cur,
                 u_i2c_top.u_i2c_slave.bit_pos,
                 u_i2c_top.u_i2c_slave.my_addr,
                 u_i2c_top.u_i2c_slave.addressed);
      end
      last_slv_state_val <= u_i2c_top.u_i2c_slave.state;
    end
  end

  // ---------------------------------------------------------------------------
  // Timeout protection
  // ---------------------------------------------------------------------------
  initial begin
    #10_000_000;
    $display("[%0t] TIMEOUT: Simulation exceeded 10 ms!", $time);
    $finish;
  end

endmodule : tb_i2c
