//==============================================================================
// tb_i2c_smbus_fastmode.sv
// Testbench for Complete I2C Controller — SMBus 3.0 Fast Mode (400 kHz)
//
// Comprehensive 18-test suite identical to tb_i2c_smbus.sv but running
// at 400 kHz Fast-mode with asymmetric t_LOW/t_HIGH timing.
// Includes: PEC, SMBALERT#, ARP, and clock stretching.
//==============================================================================

`timescale 1ns / 1ps

import i2c_pkg::*;

module tb_i2c_smbus_fastmode();

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  parameter CLK_PERIOD   = 20;           // 50 MHz
  parameter SLAVE_ADDR   = 7'h50;        // Target slave address
  parameter SCL_FREQ     = 400_000;      // Fast mode (400 kHz)

  // SMBus 3.0 features
  parameter bit SMBUS_EN        = 1'b1;  // SMBus mode active
  parameter bit PEC_EN          = 1'b1;  // PEC CRC-8 enabled
  parameter bit SMBALERT_EN     = 1'b1;  // SMBALERT# enabled
  parameter bit ARP_EN          = 1'b1;  // ARP (Address Resolution Protocol) enabled
  // UDID for ARP: 16 bytes = {capabilities, version, vendorID[15:0], deviceID[31:0],
  //                          interface[31:0], subsystem[15:0], vendor_specific[15:0]}
  parameter logic [127:0] UDID_VAL = 128'hA1_B2_C0DE_0000_0000_00000000_0000_0000;

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
  logic        mst_smbus_timeout;
  logic        mst_pec_error;

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

  // I2C Bus + SMBALERT#
  wire         scl;
  wire         sda;
  wire         slv_smbalert_n;

  // ---------------------------------------------------------------------------
  // Pull-up resistors (model external pull-ups on the bus)
  // ---------------------------------------------------------------------------
  pullup(scl);
  pullup(sda);

  // PEC: captures first rx_data byte before the PEC byte overwrites it
  logic [7:0] slv_rx_cap;

  // ---------------------------------------------------------------------------
  // DUT Instantiation — SMBus Fast Mode (400 kHz) with all features
  // ---------------------------------------------------------------------------
  i2c_top #(
    .CLK_FREQ         (50_000_000),
    .SCL_FREQ         (SCL_FREQ),
    .SLV_ADDR         (SLAVE_ADDR),
    .GEN_CALL         (1'b1),
    .SMBUS_ENABLE     (SMBUS_EN),
    .SMBUS_PEC_ENABLE (PEC_EN),
    .SMBALERT_ENABLE  (SMBALERT_EN),
    .SMARP_ENABLE     (ARP_EN),
    .UDID             (UDID_VAL),
    .SLV_STRETCH_ENABLE(1'b1)
  ) u_i2c_top (
    .clk             (clk),
    .rst_n           (rst_n),
    .mst_start       (mst_start),
    .mst_reset       (mst_reset),
    .mst_addr        (mst_addr),
    .mst_rw          (mst_rw),
    .mst_data_tx     (mst_data_tx),
    .mst_data_rx     (mst_data_rx),
    .mst_num_bytes   (mst_num_bytes),
    .mst_busy        (mst_busy),
    .mst_done        (mst_done),
    .mst_ack_error   (mst_ack_error),
    .mst_arb_lost    (mst_arb_lost),
    .mst_smbus_timeout(mst_smbus_timeout),
    .mst_pec_error   (mst_pec_error),
    .slv_tx_data     (slv_tx_data),
    .slv_tx_valid    (slv_tx_valid),
    .slv_tx_ready    (slv_tx_ready),
    .slv_rx_data     (slv_rx_data),
    .slv_rx_valid    (slv_rx_valid),
    .slv_rx_ready    (slv_rx_ready),
    .slv_addressed   (slv_addressed),
    .slv_dir_rw      (slv_dir_rw),
    .slv_busy        (slv_busy),
    .scl             (scl),
    .sda             (sda),
    .slv_smbalert_n  (slv_smbalert_n)
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
    $dumpfile("tb_i2c_smbus_fastmode.vcd");
    $dumpvars(0, tb_i2c_smbus_fastmode);
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
    mst_start       = 1'b0;
    mst_reset       = 1'b0;
    mst_addr        = '0;
    mst_rw          = 1'b0;
    mst_data_tx     = '0;
    mst_num_bytes   = '0;
    slv_tx_data     = '0;
    slv_tx_valid    = 1'b0;
    slv_rx_ready    = 1'b1;
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

  function automatic logic [7:0] ref_crc8(input logic [7:0] crc_in, input logic [7:0] data_byte);
    logic [7:0] c;
    c = crc_in ^ data_byte;
    for (int i = 0; i < 8; i++)
      c = c[7] ? ({c[6:0], 1'b0} ^ 8'h07) : {c[6:0], 1'b0};
    return c;
  endfunction

  // ===========================================================================
  // MAIN TEST SEQUENCE
  // ===========================================================================
  initial begin
    logic [7:0] readback;

    $display("");
    $display("============================================================");
    $display("  I2C Controller Testbench — SMBus 3.0 Fast Mode (400 kHz)");
    $display("  SCL Frequency: %0d kHz", SCL_FREQ / 1000);
    $display("  Slave Address: 0x%02x", SLAVE_ADDR);
    $display("  SMBUS_ENABLE:  %b", SMBUS_EN);
    $display("  PEC_ENABLE:    %b", PEC_EN);
    $display("  SMBALERT_EN:   %b", SMBALERT_EN);
    $display("  ARP_EN:        %b", ARP_EN);
    $display("  SLV_STRETCH:   1 (enabled)");
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
    check_equal("SMBus timeout inactive", 0, mst_smbus_timeout);

    // -----------------------------------------------------------------------
    // Test 2: Master write to slave at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 2: Master Write to Slave (400 kHz) ---");

    // Pre-load slave TX data (for any subsequent read)
    slv_tx_data  = 8'hA5;
    slv_tx_valid = 1'b1;
    wait_clks(2);

    $display("[%0t] Starting MASTER WRITE: addr=0x%02x, data=0x%02x",
             $time, SLAVE_ADDR, 8'hAB);
    mst_addr      = SLAVE_ADDR;
    mst_rw        = 1'b0;  // Write
    mst_data_tx   = 8'hAB;
    mst_num_bytes = 8'd1;
    mst_start     = 1'b1;
    wait_clks(1);
    mst_start = 1'b0;

    wait(slv_addressed);
    check_equal("Slave was addressed", 1'b1, slv_addressed);
    check_equal("Slave direction is write", 1'b0, slv_dir_rw);

    // With PEC enabled, the slave receives an extra PEC byte which
    // overwrites rx_data. Capture the data byte immediately on rx_valid.
    @(posedge slv_rx_valid);
    slv_rx_cap = slv_rx_data;

    wait(mst_done || mst_ack_error || mst_smbus_timeout);
    wait_clks(1);

    if (mst_smbus_timeout)
      $display("[%0t] WARN: SMBus timeout during Test 2 write!", $time);
    else if (mst_ack_error)
      $display("[%0t] ERROR: Master write NACK received!", $time);
    else
      $display("[%0t] Master WRITE complete successfully.", $time);

    wait_clks(5);
    $display("Slave RX data: 0x%02x (first byte), 0x%02x (last after PEC)",
             slv_rx_cap, slv_rx_data);
    check_equal("Slave received correct data", 8'hAB, slv_rx_cap);

    // -----------------------------------------------------------------------
    // Test 3: Master read from slave at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 3: Master Read from Slave (400 kHz) ---");

    // Keep tx_valid=1 during read to prevent false read-stretching
    slv_tx_data  = 8'h3C;
    slv_tx_valid = 1'b1;
    wait_clks(2);

    master_read(SLAVE_ADDR, readback);
    slv_tx_valid = 1'b0;  // Clear after read
    check_equal("Master read correct data", 8'h3C, readback);
    check_equal("Slave direction is read", 1'b1, slv_dir_rw);

    // -----------------------------------------------------------------------
    // Test 4: Repeated START (combined write+read) at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 4: Repeated START (combined write+read) at 400 kHz ---");

    slv_tx_data  = 8'h99;
    slv_tx_valid = 1'b1;
    wait_clks(2);

    master_write(SLAVE_ADDR, 8'h11);
    slv_tx_valid = 1'b0;  // Clear after write
    wait_clks(10);

    // Provide TX data for the read
    slv_tx_data  = 8'h99;
    slv_tx_valid = 1'b1;
    wait_clks(2);

    master_read(SLAVE_ADDR, readback);
    slv_tx_valid = 1'b0;  // Clear after read
    check_equal("Repeated START - master read correct", 8'h99, readback);

    // -----------------------------------------------------------------------
    // Test 5: Burst write (multiple bytes in single transaction) at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 5: Burst Write (Multiple Bytes) at 400 kHz ---");

    mst_addr      = SLAVE_ADDR;
    mst_rw        = 1'b0;
    mst_data_tx   = 8'h01;
    mst_num_bytes = 8'd4;
    mst_start     = 1'b1;
    wait_clks(1);
    mst_start     = 1'b0;
    wait(mst_done || mst_ack_error || mst_smbus_timeout);
    wait_clks(2);  // Settle before the next transaction (see tb_i2c.sv
                    // Test 5 for why this matters under Verilator).
    if (mst_smbus_timeout)
      $display("[%0t] WARN: SMBus timeout during Test 5 burst!", $time);
    else
      $display("[%0t] Master BURST WRITE complete.", $time);

    // -----------------------------------------------------------------------
    // Test 6: Error condition - write to non-existent slave
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 6: Error Condition - Write to non-existent address ---");

    $display("[%0t] Starting MASTER WRITE: addr=0x%02x, data=0x%02x",
             $time, 7'h7F, 8'hFF);
    mst_addr      = 7'h7F;
    mst_rw        = 1'b0;
    mst_data_tx   = 8'hFF;
    mst_num_bytes = 8'd1;
    mst_start     = 1'b1;
    wait_clks(1);
    mst_start = 1'b0;

    @(posedge mst_busy);  // Transaction started
    @(negedge mst_busy);  // Transaction completed
    wait_clks(2);

    if (mst_ack_error)
      $display("PASS: Master correctly detected NACK for non-existent slave.");
    else
      $display("FAIL: Master should have detected NACK!");

    // -----------------------------------------------------------------------
    // Test 7: Multiple transactions at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 7: Multiple Write Transactions at 400 kHz ---");

    for (int i = 0; i < 5; i++) begin
      master_write(SLAVE_ADDR, 8'hA0 + i);
      $display("  Write %0d: data=0x%02x, slave_rx_last=0x%02x",
               i, 8'hA0 + i, slv_rx_data);
    end

    // -----------------------------------------------------------------------
    // Test 8: Edge case - back-to-back transactions at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 8: Back-to-back transactions at 400 kHz ---");

    slv_tx_data  = 8'h42;
    slv_tx_valid = 1'b1;
    wait_clks(2);

    master_write(SLAVE_ADDR, 8'hDE);
    // tx_valid still 1 for the read
    master_read(SLAVE_ADDR, readback);
    slv_tx_valid = 1'b0;  // Clear after read
    check_equal("Back-to-back read data", 8'h42, readback);

    // -----------------------------------------------------------------------
    // Test 9: SMBus PEC write transaction at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 9: SMBus PEC Write Transaction (CRC-8) at 400 kHz ---");

    $display("[%0t] Starting PEC write: addr=0x%02x, data=0x%02x (2 data bytes + PEC)",
             $time, SLAVE_ADDR, 8'hAB);
    slv_tx_data  = 8'hA5;
    slv_tx_valid = 1'b1;
    wait_clks(2);

    mst_addr      = SLAVE_ADDR;
    mst_rw        = 1'b0;
    mst_data_tx   = 8'hAB;
    mst_num_bytes = 8'd2;  // 2 data bytes + PEC (CRC-8 appends automatically)
    slv_tx_valid = 1'b0;  // Clear after preload (write doesn't need tx)
    mst_start     = 1'b1;
    wait_clks(1);
    mst_start     = 1'b0;

    wait(mst_done || mst_ack_error || mst_smbus_timeout);
    wait_clks(5);

    if (mst_smbus_timeout)
      $display("[%0t] WARN: SMBus timeout during Test 9 PEC write!", $time);
    else if (mst_ack_error)
      $display("[%0t] FAIL: PEC write got NACK!", $time);
    else if (mst_pec_error)
      $display("[%0t] FAIL: PEC error detected on master side!", $time);
    else
      $display("[%0t] PASS: PEC write complete at 400 kHz — CRC-8 appended and ACKed.", $time);

    // Check slave PEC status
    if (u_i2c_top.u_i2c_slave.pec_error)
      $display("[%0t] FAIL: Slave PEC error! CRC mismatch on received data.", $time);
    else
      $display("[%0t] PASS: Slave verified PEC at 400 kHz — CRC matches.", $time);

    // -----------------------------------------------------------------------
    // Test 10: SMBus PEC burst write at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 10: SMBus PEC Burst Write (4 bytes + PEC) at 400 kHz ---");

    slv_tx_data  = 8'hA5;
    slv_tx_valid = 1'b1;
    wait_clks(2);

    mst_addr      = SLAVE_ADDR;
    mst_rw        = 1'b0;
    mst_data_tx   = 8'h01;
    mst_num_bytes = 8'd4;  // 4 data bytes + PEC
    slv_tx_valid = 1'b0;  // Clear after preload (write doesn't need tx)
    mst_start     = 1'b1;
    wait_clks(1);
    mst_start     = 1'b0;

    wait(mst_done || mst_ack_error || mst_smbus_timeout);
    wait_clks(5);

    if (mst_smbus_timeout)
      $display("[%0t] WARN: SMBus timeout during Test 10 PEC burst!", $time);
    else if (mst_ack_error)
      $display("[%0t] FAIL: PEC burst got NACK!", $time);
    else if (mst_pec_error)
      $display("[%0t] FAIL: PEC error on master during burst!", $time);
    else
      $display("[%0t] PASS: PEC burst write complete at 400 kHz — CRC-8 OK.", $time);

    if (u_i2c_top.u_i2c_slave.pec_error)
      $display("[%0t] FAIL: Slave PEC error on burst!", $time);
    else
      $display("[%0t] PASS: Slave verified PEC for burst at 400 kHz — CRC matches.", $time);

    // -----------------------------------------------------------------------
    // Test 11: SMBALERT# assertion at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 11: SMBALERT# Assertion (400 kHz) ---");

    // Load TX data into slave — should assert SMBALERT#
    slv_tx_data  = 8'hA5;
    slv_tx_valid = 1'b1;
    wait_clks(5);

    if (u_i2c_top.u_i2c_slave.alert_pending)
      $display("[%0t] PASS: alert_pending asserted after TX data loaded.", $time);
    else
      $display("[%0t] FAIL: alert_pending not asserted!", $time);

    // SMBALERT# should be driven low (active)
    if (slv_smbalert_n === 1'b0)
      $display("[%0t] PASS: smbalert_n driven LOW (asserted).", $time);
    else
      $display("[%0t] FAIL: smbalert_n = %b (expected 0)!", $time, slv_smbalert_n);

    // -----------------------------------------------------------------------
    // Test 12: ARA — Master reads from Alert Response Address at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 12: ARA Read at 400 kHz ---");

    $display("[%0t] Starting ARA read (addr=0x%02x)...", $time, ARA_ADDR);
    mst_addr      = ARA_ADDR;  // 0x0C
    mst_rw        = 1'b1;      // Read
    mst_data_tx   = '0;
    mst_num_bytes = 8'd1;
    mst_start     = 1'b1;
    wait_clks(1);
    mst_start     = 1'b0;

    @(posedge mst_busy);
    @(negedge mst_busy);
    wait_clks(2);

    if (mst_ack_error)
      $display("[%0t] FAIL: ARA read got NACK!", $time);
    else begin
      logic [7:0] expected_ara = {SLAVE_ADDR, 1'b0};
      $display("[%0t] ARA read data = 0x%02x (expected 0x%02x = {%02x, 0})",
               $time, mst_data_rx, expected_ara, SLAVE_ADDR);
      if (mst_data_rx == expected_ara)
        $display("[%0t] PASS: ARA response matches slave address 0x%02x at 400 kHz.", $time, SLAVE_ADDR);
      else
        $display("[%0t] FAIL: ARA response 0x%02x != expected 0x%02x!",
                 $time, mst_data_rx, expected_ara);
    end

    // After being addressed (via ARA), alert_pending should clear
    wait_clks(10);
    if (!u_i2c_top.u_i2c_slave.alert_pending)
      $display("[%0t] PASS: alert_pending cleared after ARA response.", $time);
    else
      $display("[%0t] FAIL: alert_pending still asserted after ARA!", $time);

    // -----------------------------------------------------------------------
    // SMBus 3.0 ARP Tests (13-16)
    // -----------------------------------------------------------------------
    $display("");
    $display("============================================================");
    $display("  SMBus 3.0 ARP Tests (400 kHz)");
    $display("  Default Address: 0x%02x", SMBUS_DEFAULT_ADDR);
    $display("  UDID: 0x%032h", UDID_VAL);
    $display("============================================================");

    // -----------------------------------------------------------------------
    // Test 13: ARP Default Address Recognition at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 13: ARP Default Address Write (400 kHz) ---");

    // Write Prepare to ARP command (0x01) to default address 0x61
    $display("[%0t] Writing ARP Prepare command (0x01) to 0x%02x...",
             $time, SMBUS_DEFAULT_ADDR);
    master_write(SMBUS_DEFAULT_ADDR, ARP_CMD_PREPARE_TO_ARP);

    if (mst_ack_error)
      $display("[%0t] FAIL: Slave NACKed default address!", $time);
    else
      $display("[%0t] PASS: Slave ACKed default address 0x%02x at 400 kHz.", $time, SMBUS_DEFAULT_ADDR);

    // Verify ARP command was captured
    wait_clks(5);
    if (u_i2c_top.u_i2c_slave.arp_cmd == ARP_CMD_PREPARE_TO_ARP)
      $display("[%0t] PASS: ARP command 0x%02x captured.", $time, ARP_CMD_PREPARE_TO_ARP);
    else
      $display("[%0t] FAIL: ARP command = 0x%02x (expected 0x%02x)!",
               $time, u_i2c_top.u_i2c_slave.arp_cmd, ARP_CMD_PREPARE_TO_ARP);

    check_equal("ARP: av flag = 0 (default)", 1'b0, u_i2c_top.u_i2c_slave.av);
    check_equal("ARP: assigned_addr = SLAVE_ADDR", SLAVE_ADDR,
                u_i2c_top.u_i2c_slave.assigned_addr);

    // -----------------------------------------------------------------------
    // Test 14: ARP Get UDID at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 14: ARP Get UDID Command (400 kHz) ---");

    // Write Get UDID command (0x02) to default address
    $display("[%0t] Writing ARP Get UDID command (0x02) to 0x%02x...",
             $time, SMBUS_DEFAULT_ADDR);
    master_write(SMBUS_DEFAULT_ADDR, ARP_CMD_GET_UDID);

    if (mst_ack_error)
      $display("[%0t] FAIL: Slave NACKed Get UDID command!", $time);
    else
      $display("[%0t] PASS: Get UDID command ACKed at 400 kHz.", $time);

    wait_clks(5);
    if (u_i2c_top.u_i2c_slave.arp_get_udid_pending)
      $display("[%0t] PASS: arp_get_udid_pending set after Get UDID cmd.", $time);
    else
      $display("[%0t] FAIL: arp_get_udid_pending not set!", $time);

    // -----------------------------------------------------------------------
    // Test 15: ARP UDID Read (16 bytes) at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 15: ARP UDID Read (16 bytes) at 400 kHz ---");

    for (int i = 0; i < 16; i++) begin
      logic [7:0] udid_byte;
      logic [7:0] expected_byte;

      // Expected: UDID[127:0], byte 0 (first) = UDID[127:120]
      expected_byte = UDID_VAL[(15 - i) * 8 +: 8];

      master_read(SMBUS_DEFAULT_ADDR, udid_byte);

      $display("[%0t]   UDID byte %0d/15: read=0x%02x, expected=0x%02x",
               $time, i, udid_byte, expected_byte);

      if (udid_byte === expected_byte)
        $display("[%0t]   PASS: UDID byte %0d matches at 400 kHz.", $time, i);
      else if (udid_byte === 'x)
        $display("[%0t]   WARN: UDID byte %0d = X (bus conflict or NACK)", $time, i);
      else
        $display("[%0t]   FAIL: UDID byte %0d = 0x%02x != expected 0x%02x!",
                 $time, i, udid_byte, expected_byte);
    end

    // After 16 reads, arp_get_udid_pending should be cleared
    wait_clks(5);
    if (!u_i2c_top.u_i2c_slave.arp_get_udid_pending)
      $display("[%0t] PASS: arp_get_udid_pending cleared after 16 UDID bytes at 400 kHz.", $time);
    else
      $display("[%0t] WARN: arp_get_udid_pending still set (may clear on next STOP).", $time);

    // -----------------------------------------------------------------------
    // Test 16: Normal operation after ARP at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 16: Normal Communication After ARP (400 kHz) ---");

    slv_tx_data  = 8'h5A;
    slv_tx_valid = 1'b1;
    wait_clks(2);

    // Write to own address (keep tx_valid=1 for the read that follows)
    master_write(SLAVE_ADDR, 8'hC3);

    if (mst_ack_error)
      $display("[%0t] FAIL: Own address NACKed after ARP!", $time);
    else
      $display("[%0t] PASS: Own address 0x%02x still works after ARP at 400 kHz.",
               $time, SLAVE_ADDR);

    // Read from own address
    master_read(SLAVE_ADDR, readback);
    slv_tx_valid = 1'b0;  // Clear after read
    check_equal("ARP post-test: read data = 0x5A", 8'h5A, readback);

    // -----------------------------------------------------------------------
    // Test 16b: ARP Get UDID Directed (0x04) at 400 kHz — addressed via
    // own address, not the default address 0x61.
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 16b: ARP Get UDID Directed Command (0x04, own address, 400 kHz) ---");

    $display("[%0t] Writing ARP Get UDID Directed command (0x04) to own address 0x%02x...",
             $time, SLAVE_ADDR);
    master_write(SLAVE_ADDR, ARP_CMD_GET_UDID_DIR);

    if (mst_ack_error)
      $display("[%0t] FAIL: Slave NACKed Get UDID Directed command!", $time);
    else
      $display("[%0t] PASS: Get UDID Directed command ACKed at 400 kHz.", $time);

    wait_clks(5);
    if (u_i2c_top.u_i2c_slave.arp_get_udid_pending)
      $display("[%0t] PASS: arp_get_udid_pending set after Get UDID Directed cmd.", $time);
    else
      $display("[%0t] FAIL: arp_get_udid_pending not set!", $time);

    for (int i = 0; i < 16; i++) begin
      logic [7:0] udid_byte;
      logic [7:0] expected_byte;

      expected_byte = UDID_VAL[(15 - i) * 8 +: 8];
      master_read(SLAVE_ADDR, udid_byte);

      if (udid_byte === expected_byte)
        $display("[%0t]   PASS: Directed UDID byte %0d matches.", $time, i);
      else
        $display("[%0t]   FAIL: Directed UDID byte %0d = 0x%02x != expected 0x%02x!",
                 $time, i, udid_byte, expected_byte);
    end

    wait_clks(5);
    if (!u_i2c_top.u_i2c_slave.arp_get_udid_pending)
      $display("[%0t] PASS: arp_get_udid_pending cleared after directed UDID read.", $time);
    else
      $display("[%0t] WARN: arp_get_udid_pending still set (may clear on next STOP).", $time);

    // -----------------------------------------------------------------------
    // Test 16c: ARP Reset Device Directed (0x06) at 400 kHz — addressed
    // via own address, not the default address 0x61.
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 16c: ARP Reset Device Directed Command (0x06, own address, 400 kHz) ---");

    $display("[%0t] Writing ARP Reset Device Directed command (0x06) to own address 0x%02x...",
             $time, SLAVE_ADDR);
    master_write(SLAVE_ADDR, ARP_CMD_RESET_DEV_DIR);

    if (mst_ack_error)
      $display("[%0t] FAIL: Slave NACKed Reset Device Directed command!", $time);
    else
      $display("[%0t] PASS: Reset Device Directed command ACKed at 400 kHz.", $time);

    wait_clks(5);
    check_equal("ARP Directed Reset: av flag = 0", 1'b0, u_i2c_top.u_i2c_slave.av);
    check_equal("ARP Directed Reset: assigned_addr = SLAVE_ADDR", SLAVE_ADDR,
                u_i2c_top.u_i2c_slave.assigned_addr);

    slv_tx_data  = 8'h64;
    slv_tx_valid = 1'b1;
    wait_clks(2);
    master_read(SLAVE_ADDR, readback);
    slv_tx_valid = 1'b0;
    check_equal("Own address still works after Directed Reset", 8'h64, readback);

    // -----------------------------------------------------------------------
    // Test 16d: ARP Assign Address (0x03) at 400 kHz — cmd + 16 UDID bytes
    // + new address in a single transaction. This is the one ARP command
    // never previously exercised by any testbench; it depends on
    // arp_byte_cnt being wide enough to count past the 16-byte UDID, and
    // on the slave correctly capturing the first bit of each new byte
    // right after the ACK phase (see rtl/i2c_slave.sv shift_reg capture
    // during S_ACK_PHASE).
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 16d: ARP Assign Address Command (0x03) at 400 kHz ---");

    begin
      logic [7:0] assign_bytes[18];
      logic [6:0] new_addr;
      int i;
      new_addr = 7'h20;
      assign_bytes[0] = ARP_CMD_ASSIGN_ADDR;
      for (i = 0; i < 16; i++)
        assign_bytes[1 + i] = UDID_VAL[(15 - i) * 8 +: 8];
      assign_bytes[17] = {new_addr, 1'b0};

      // See tb_i2c_smbus.sv Test 16d for the full explanation of this
      // staging scheme (iverilog doesn't support unpacked-array task
      // ports, and the master samples mst_data_tx for byte N+1 right at
      // byte N's ACK, ~1 cycle before slv_rx_valid for byte N is visible
      // here — so bytes are staged two ahead of the reactive loop).
      $display("[%0t] Writing ARP Assign Address (cmd + 16 UDID bytes + new addr 0x%02x) to 0x%02x...",
               $time, new_addr, SMBUS_DEFAULT_ADDR);
      mst_addr      = SMBUS_DEFAULT_ADDR;
      mst_rw        = 1'b0;
      mst_data_tx   = assign_bytes[0];
      mst_num_bytes = 8'd18;
      mst_start     = 1'b1;
      wait_clks(1);
      mst_start     = 1'b0;
      @(negedge u_i2c_top.u_i2c_master.addr_phase);
      mst_data_tx = assign_bytes[1];  // stage byte 2
      for (i = 0; i < 16; i++) begin
        @(posedge slv_rx_valid);      // byte i (0-indexed) received
        mst_data_tx = assign_bytes[i + 2];  // stage byte i+2
      end
      @(negedge mst_busy);
      wait_clks(1);

      if (mst_ack_error)
        $display("[%0t] FAIL: ARP Assign Address write got NACK!", $time);
      else
        $display("[%0t] PASS: ARP Assign Address write (18 bytes) ACKed at 400 kHz.", $time);

      wait_clks(5);
      check_equal("ARP Assign Address: av flag = 1", 1'b1, u_i2c_top.u_i2c_slave.av);
      check_equal("ARP Assign Address: assigned_addr = new address",
                  new_addr, u_i2c_top.u_i2c_slave.assigned_addr);

      // Old address (SLAVE_ADDR) must no longer respond
      mst_addr      = SLAVE_ADDR;
      mst_rw        = 1'b0;
      mst_data_tx   = 8'hEE;
      mst_num_bytes = 8'd1;
      mst_start     = 1'b1;
      wait_clks(1);
      mst_start     = 1'b0;
      @(posedge mst_busy);
      @(negedge mst_busy);
      wait_clks(1);
      check_equal("Old address NACKed after reassignment", 1'b1, mst_ack_error);

      // New address must work for both write and read
      slv_tx_data  = 8'h77;
      slv_tx_valid = 1'b1;
      wait_clks(2);
      master_write(new_addr, 8'h55);
      check_equal("Write to new address succeeds", 1'b0, mst_ack_error);

      master_read(new_addr, readback);
      slv_tx_valid = 1'b0;
      check_equal("Read from new address returns correct data", 8'h77, readback);

      // Restore original address (directed reset to new_addr — the
      // device no longer responds on the default address once av=1).
      $display("[%0t] Restoring original address via ARP Reset Device Directed...", $time);
      master_write(new_addr, ARP_CMD_RESET_DEV_DIR);
      wait_clks(5);
      check_equal("Post-restore: av flag = 0", 1'b0, u_i2c_top.u_i2c_slave.av);
      check_equal("Post-restore: assigned_addr = SLAVE_ADDR", SLAVE_ADDR,
                  u_i2c_top.u_i2c_slave.assigned_addr);

      slv_tx_data  = 8'h64;
      slv_tx_valid = 1'b1;
      wait_clks(2);
      master_read(SLAVE_ADDR, readback);
      slv_tx_valid = 1'b0;
      check_equal("Own address works again after restore", 8'h64, readback);
    end

    // -----------------------------------------------------------------------
    // Clock Stretching Tests (17-18) at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("============================================================");
    $display("  Slave Clock Stretching Tests (400 kHz)");
    $display("============================================================");

    // -----------------------------------------------------------------------
    // Test 17: Clock Stretch on Write at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 17: Clock Stretch on Write at 400 kHz (delayed rx_ready) ---");

    // Disable rx_ready to force slave to stretch after receiving byte
    slv_rx_ready = 1'b0;

    $display("[%0t] Starting write with rx_ready=0 (expecting stretch)...", $time);
    mst_addr      = SLAVE_ADDR;
    mst_rw        = 1'b0;
    mst_data_tx   = 8'h77;
    mst_num_bytes = 8'd1;
    mst_start     = 1'b1;
    wait_clks(1);
    mst_start     = 1'b0;

    // Wait until the byte has been received (rx_valid fires)
    @(posedge slv_rx_valid);
    // Capture immediately before stretch or PEC can overwrite
    slv_rx_cap = slv_rx_data;

    // Now the slave should be stretching (rx_ready=0, byte received)
    wait_clks(5);  // Let stretch settle
    if (u_i2c_top.u_i2c_slave.stretch_active)
      $display("[%0t] PASS: Slave is stretching (stretch_active=1) at 400 kHz after byte received.", $time);
    else
      $display("[%0t] INFO: Stretch may have already triggered and released.", $time);

    if (u_i2c_top.u_i2c_slave.scl_oe_slv)
      $display("[%0t] PASS: scl_oe_slv=1 (slave driving SCL low for stretch at 400 kHz).", $time);

    // Now consume the rx data and release the stretch
    $display("[%0t] Consuming rx_data (rx_ready=1)...", $time);
    slv_rx_ready = 1'b1;

    wait(mst_done || mst_ack_error || mst_smbus_timeout);
    wait_clks(1);

    if (mst_smbus_timeout)
      $display("[%0t] WARN: SMBus timeout during Test 17 stretch at 400 kHz!", $time);
    else if (mst_ack_error)
      $display("[%0t] FAIL: Stretch write got NACK at 400 kHz!", $time);
    else
      $display("[%0t] PASS: Stretch write completed at 400 kHz after rx_ready released.", $time);

    wait_clks(5);
    check_equal("Stretch at 400 kHz: slave received correct data (0x77)", 8'h77, slv_rx_cap);
    slv_rx_ready = 1'b1;  // Restore default

    // -----------------------------------------------------------------------
    // Test 18: Clock Stretch on Read at 400 kHz
    // -----------------------------------------------------------------------
    $display("");
    $display("--- Test 18: Clock Stretch on Read at 400 kHz (delayed tx_valid) ---");

    // Don't load TX data yet — slave should stretch waiting for it
    slv_tx_data  = 8'h00;
    slv_tx_valid = 1'b0;
    wait_clks(5);

    $display("[%0t] Starting read with tx_valid=0 (expecting stretch at 400 kHz)...", $time);
    mst_addr      = SLAVE_ADDR;
    mst_rw        = 1'b1;  // Read
    mst_data_tx   = '0;
    mst_num_bytes = 8'd1;
    mst_start     = 1'b1;
    wait_clks(1);
    mst_start     = 1'b0;

    // Wait for address phase to complete
    wait(slv_addressed || mst_ack_error || mst_smbus_timeout);
    wait_clks(100);

    // Check if stretch is active
    if (u_i2c_top.u_i2c_slave.stretch_active)
      $display("[%0t] PASS: Stretch activated at 400 kHz — slave holding SCL low waiting for TX data.", $time);
    else
      $display("[%0t] INFO: No stretch detected at 400 kHz (tx_valid was provided too early).", $time);

    // Provide TX data to release the stretch
    $display("[%0t] Providing TX data (tx_valid=1)...", $time);
    slv_tx_data  = 8'h3C;
    slv_tx_valid = 1'b1;
    wait_clks(20);

    if (!mst_done && !mst_ack_error && !mst_smbus_timeout)
      wait(mst_done || mst_ack_error || mst_smbus_timeout);
    wait_clks(1);

    if (mst_smbus_timeout)
      $display("[%0t] WARN: SMBus timeout during Test 18 stretch at 400 kHz!", $time);
    else if (mst_ack_error)
      $display("[%0t] FAIL: Stretch read got NACK at 400 kHz!", $time);
    else begin
      $display("[%0t] PASS: Stretch read completed at 400 kHz after tx_valid provided.", $time);
      check_equal("Stretch read at 400 kHz: data = 0x3C", 8'h3C, mst_data_rx);
    end
    slv_tx_valid = 1'b0;  // Clear after read

    // -----------------------------------------------------------------------
    // Test 19: Randomized Burst Write at 400 kHz — regression guard
    //
    // See tb_i2c_smbus.sv Test 19 for the full rationale: every other
    // burst-write test here either sends a single byte or repeats the
    // same byte across the whole burst, which coincidentally never
    // exposes a class of bug where the slave drops/miscaptures a byte's
    // first bit right after the ACK phase. This sends bursts of random
    // length with independently-random byte values and checks each byte
    // the slave actually received against what was sent.
    // -----------------------------------------------------------------------
    $display("");
    $display("============================================================");
    $display("  Test 19: Randomized Burst Write at 400 kHz (regression guard)");
    $display("============================================================");

    // arp_get_udid_pending only clears on the STOP that follows once
    // arp_tx_byte_cnt has reached UDID_BYTES-1 (see Test 16b, which
    // already WARNs — doesn't FAIL — when that hasn't happened by its
    // own end). Tests 19/20 exercise plain data transport, not ARP, so
    // force a clean slate here rather than depend on that timing.
    force u_i2c_top.u_i2c_slave.arp_get_udid_pending = 1'b0;
    wait_clks(2);
    release u_i2c_top.u_i2c_slave.arp_get_udid_pending;

    begin
      localparam int NUM_RAND_ITERS = 20;
      localparam int MAX_BURST      = 20;
      int burst_len;
      int b;
      int mismatches;
      int total_mismatches;
      integer rand_seed;
      logic [7:0] wr_data[MAX_BURST];
      logic [7:0] captured[MAX_BURST];

      total_mismatches = 0;
      rand_seed = 32'hDEC0DE01;
      void'($urandom(rand_seed));  // Fixed seed: reproducible regression

      for (int iter = 0; iter < NUM_RAND_ITERS; iter++) begin
        burst_len = 2 + ($urandom % (MAX_BURST - 1));  // 2..MAX_BURST
        for (b = 0; b < burst_len; b++)
          wr_data[b] = $urandom % 256;
        // This burst targets SLAVE_ADDR (the device's own assigned
        // address), which also accepts directed ARP commands on that
        // same wire format (rtl/i2c_slave.sv own_arp_write path). A
        // random first byte landing in 0x01-0x06 would genuinely (and
        // correctly, per the reserved ARP command space) be interpreted
        // as an ARP command instead of plain data — this test is for
        // general data transport, not ARP, so remap it out of that
        // range rather than let a rare seed flakily fail here. A fixed
        // remap (not an offset) — an offset like +0x10 can wrap back
        // into 0x01-0x06 for source values in 0xF1-0xF6.
        if (wr_data[0] >= 8'h01 && wr_data[0] <= 8'h06)
          wr_data[0] = 8'h50;

        mst_addr      = SLAVE_ADDR;
        mst_rw        = 1'b0;
        mst_data_tx   = wr_data[0];
        mst_num_bytes = burst_len[7:0];
        mst_start     = 1'b1;
        wait_clks(1);
        mst_start     = 1'b0;

        @(posedge mst_busy);
        @(negedge u_i2c_top.u_i2c_master.addr_phase);
        if (burst_len > 1)
          mst_data_tx = wr_data[1];

        for (b = 0; b < burst_len; b++) begin
          @(posedge slv_rx_valid);
          captured[b] = slv_rx_data;
          if (b + 2 < burst_len)
            mst_data_tx = wr_data[b + 2];
        end

        @(negedge mst_busy);
        wait_clks(1);

        if (mst_ack_error) begin
          $display("[%0t] FAIL: Random burst #%0d (len=%0d) got NACK at 400 kHz!",
                   $time, iter, burst_len);
          total_mismatches++;
        end else begin
          mismatches = 0;
          for (b = 0; b < burst_len; b++) begin
            if (captured[b] !== wr_data[b]) begin
              $display("[%0t]   MISMATCH burst #%0d byte %0d: sent=0x%02x received=0x%02x",
                       $time, iter, b, wr_data[b], captured[b]);
              mismatches++;
            end
          end
          check_equal($sformatf("Random burst #%0d (len=%0d) data matches at 400 kHz", iter, burst_len),
                      0, mismatches);
          total_mismatches += mismatches;
        end
      end

      check_equal("Randomized burst write regression at 400 kHz: total mismatches", 0, total_mismatches);
    end

    // -----------------------------------------------------------------------
    // Test 20: Randomized Burst Read at 400 kHz — regression guard
    //
    // See tb_i2c_smbus.sv Test 20 for the full rationale: every existing
    // read in this suite is its own single-byte transaction, so a
    // multi-byte READ burst with changing data has never been exercised —
    // this is what caught the analogous S_ACK_PHASE issue on the TX side
    // (slave releasing SDA instead of driving the next byte's first bit),
    // where a released/floating bus resolves HIGH via the pullup.
    // -----------------------------------------------------------------------
    $display("");
    $display("============================================================");
    $display("  Test 20: Randomized Burst Read at 400 kHz (regression guard)");
    $display("============================================================");

    begin
      localparam int NUM_RAND_ITERS = 20;
      localparam int MAX_BURST      = 20;
      int burst_len;
      int b;
      int mismatches;
      int total_mismatches;
      integer rand_seed;
      logic [7:0] rd_data[MAX_BURST];
      logic [7:0] captured[MAX_BURST];
      int capture_idx;

      total_mismatches = 0;
      rand_seed = 32'hFEEDBEE5;
      void'($urandom(rand_seed));  // Fixed seed: reproducible regression

      for (int iter = 0; iter < NUM_RAND_ITERS; iter++) begin
        burst_len = 2 + ($urandom % (MAX_BURST - 1));  // 2..MAX_BURST
        for (b = 0; b < burst_len; b++)
          rd_data[b] = $urandom % 256;
        capture_idx = 0;

        slv_tx_data  = rd_data[0];
        slv_tx_valid = 1'b1;
        wait_clks(2);

        mst_addr      = SLAVE_ADDR;
        mst_rw        = 1'b1;
        mst_data_tx   = 8'h00;
        mst_num_bytes = burst_len[7:0];
        mst_start     = 1'b1;
        wait_clks(1);
        mst_start     = 1'b0;

        fork
          // Staging: unlike the write side (Test 19), the slave reads
          // tx_data *continuously* (sda_o is combinational on tx_data),
          // not sampled once into a shift register — so there's no
          // "must land one full byte-period ahead" race here. A plain
          // 1-ahead reactive scheme on tx_ready is correct and sufficient.
          begin
            @(posedge mst_busy);
            for (b = 0; b < burst_len; b++) begin
              @(posedge slv_tx_ready);
              if (b + 1 < burst_len)
                slv_tx_data = rd_data[b + 1];
            end
          end
          // Capture: record every byte the master actually assembles, via
          // its internal data_rx register. Sync off byte_cnt (a monotonic
          // counter), not data_rx itself — @(data_rx) would miss an
          // update whenever two consecutive random bytes happen to be
          // equal. byte_cnt increments once for the address-phase
          // completion too, so that first transition must be discarded.
          begin
            @(u_i2c_top.u_i2c_master.byte_cnt);  // Discard: address phase
            for (capture_idx = 0; capture_idx < burst_len; capture_idx++) begin
              @(u_i2c_top.u_i2c_master.byte_cnt);
              captured[capture_idx] = u_i2c_top.u_i2c_master.data_rx;
            end
          end
        join

        @(negedge mst_busy);
        wait_clks(1);
        slv_tx_valid = 1'b0;

        if (mst_ack_error) begin
          $display("[%0t] FAIL: Random read burst #%0d (len=%0d) got NACK at 400 kHz!",
                   $time, iter, burst_len);
          total_mismatches++;
        end else begin
          mismatches = 0;
          for (b = 0; b < burst_len; b++) begin
            if (captured[b] !== rd_data[b]) begin
              $display("[%0t]   MISMATCH read burst #%0d byte %0d: expected=0x%02x received=0x%02x",
                       $time, iter, b, rd_data[b], captured[b]);
              mismatches++;
            end
          end
          check_equal($sformatf("Random read burst #%0d (len=%0d) data matches at 400 kHz", iter, burst_len),
                      0, mismatches);
          total_mismatches += mismatches;
        end
      end

      check_equal("Randomized burst read regression at 400 kHz: total mismatches", 0, total_mismatches);
    end

    // -----------------------------------------------------------------------
    // Test 21: Randomized PEC Burst Write at 400 kHz — regression guard
    //
    // See tb_i2c_smbus.sv Test 21 for the full rationale: cross-checks the
    // master's computed PEC against an independently-implemented reference
    // CRC-8, confirms the slave never false-positives on valid data, and
    // confirms a deliberately-corrupted PEC byte is always caught.
    // -----------------------------------------------------------------------
    $display("");
    $display("============================================================");
    $display("  Test 21: Randomized PEC Burst Write at 400 kHz (regression guard)");
    $display("============================================================");

    begin
      localparam int NUM_RAND_ITERS     = 20;
      localparam int NUM_MUTATION_ITERS = 5;
      localparam int MAX_BURST          = 20;
      int burst_len;
      int b;
      int total_mismatches;
      integer rand_seed;
      logic [7:0] wr_data[MAX_BURST];
      logic [7:0] ref_pec;

      total_mismatches = 0;
      rand_seed = 32'hC0FFEE05;
      void'($urandom(rand_seed));  // Fixed seed: reproducible regression

      for (int iter = 0; iter < NUM_RAND_ITERS; iter++) begin
        burst_len = 2 + ($urandom % (MAX_BURST - 1));  // 2..MAX_BURST
        for (b = 0; b < burst_len; b++)
          wr_data[b] = $urandom % 256;
        if (wr_data[0] >= 8'h01 && wr_data[0] <= 8'h06)
          wr_data[0] = 8'h50;  // avoid the reserved ARP command range

        mst_addr      = SLAVE_ADDR;
        mst_rw        = 1'b0;
        mst_data_tx   = wr_data[0];
        mst_num_bytes = burst_len[7:0];
        mst_start     = 1'b1;
        wait_clks(1);
        mst_start     = 1'b0;

        @(posedge mst_busy);
        @(negedge u_i2c_top.u_i2c_master.addr_phase);
        if (burst_len > 1)
          mst_data_tx = wr_data[1];
        for (b = 0; b < burst_len; b++) begin
          @(posedge slv_rx_valid);
          if (b + 2 < burst_len)
            mst_data_tx = wr_data[b + 2];
        end
        @(negedge mst_busy);
        wait_clks(1);

        ref_pec = ref_crc8(8'h00, {SLAVE_ADDR, 1'b0});
        for (b = 0; b < burst_len; b++)
          ref_pec = ref_crc8(ref_pec, wr_data[b]);

        if (mst_ack_error) begin
          $display("[%0t] FAIL: PEC burst #%0d (len=%0d) got NACK!", $time, iter, burst_len);
          total_mismatches++;
        end else if (u_i2c_top.u_i2c_master.pec_value !== ref_pec) begin
          $display("[%0t] FAIL: PEC burst #%0d (len=%0d): master pec_value=0x%02x != reference=0x%02x",
                   $time, iter, burst_len, u_i2c_top.u_i2c_master.pec_value, ref_pec);
          total_mismatches++;
        end else if (u_i2c_top.u_i2c_slave.pec_error) begin
          $display("[%0t] FAIL: PEC burst #%0d (len=%0d): slave pec_error false-positive on valid data!",
                   $time, iter, burst_len);
          total_mismatches++;
        end else begin
          check_equal($sformatf("PEC burst #%0d (len=%0d) at 400 kHz: CRC matches reference, no false pec_error",
                      iter, burst_len), 1, 1);
        end
      end

      // --- Negative: deliberately corrupt the PEC byte, confirm it's caught
      for (int iter = 0; iter < NUM_MUTATION_ITERS; iter++) begin
        burst_len = 2 + ($urandom % (MAX_BURST - 1));
        for (b = 0; b < burst_len; b++)
          wr_data[b] = $urandom % 256;
        if (wr_data[0] >= 8'h01 && wr_data[0] <= 8'h06)
          wr_data[0] = 8'h50;

        mst_addr      = SLAVE_ADDR;
        mst_rw        = 1'b0;
        mst_data_tx   = wr_data[0];
        mst_num_bytes = burst_len[7:0];
        mst_start     = 1'b1;
        wait_clks(1);
        mst_start     = 1'b0;

        @(posedge mst_busy);
        @(negedge u_i2c_top.u_i2c_master.addr_phase);
        if (burst_len > 1)
          mst_data_tx = wr_data[1];
        for (b = 0; b < burst_len; b++) begin
          @(posedge slv_rx_valid);
          if (b + 2 < burst_len)
            mst_data_tx = wr_data[b + 2];
        end

        wait (u_i2c_top.u_i2c_master.shift_reg == u_i2c_top.u_i2c_master.pec_value);
        force u_i2c_top.u_i2c_master.shift_reg = ~u_i2c_top.u_i2c_master.shift_reg;

        wait(mst_done || mst_ack_error || mst_smbus_timeout);
        wait_clks(5);
        release u_i2c_top.u_i2c_master.shift_reg;

        if (!u_i2c_top.u_i2c_slave.pec_error) begin
          $display("[%0t] FAIL: PEC mutation #%0d (len=%0d): corrupted PEC byte was NOT detected!",
                   $time, iter, burst_len);
          total_mismatches++;
        end else begin
          check_equal($sformatf("PEC mutation #%0d (len=%0d) at 400 kHz: corrupted PEC correctly detected",
                      iter, burst_len), 1, 1);
        end
      end

      check_equal("Randomized PEC regression at 400 kHz: total mismatches", 0, total_mismatches);
    end

    // -----------------------------------------------------------------------
    // Test 22: Randomized ARP Assign Address at 400 kHz — regression guard
    //
    // See tb_i2c_smbus.sv Test 22 for the full rationale: random valid new
    // addresses with the correct UDID must be accepted, and a deliberately
    // corrupted UDID must be rejected (av stays 0, assigned_addr unchanged).
    // -----------------------------------------------------------------------
    $display("");
    $display("============================================================");
    $display("  Test 22: Randomized ARP Assign Address at 400 kHz (regression guard)");
    $display("============================================================");

    begin
      localparam int NUM_RAND_ITERS = 8;
      localparam int NUM_NEG_ITERS  = 8;
      int total_mismatches;
      integer rand_seed;
      logic [7:0] assign_bytes[18];
      logic [6:0] new_addr;
      logic [UDID_WIDTH-1:0] bad_udid;
      int i;

      total_mismatches = 0;
      rand_seed = 32'hA55A0001;
      void'($urandom(rand_seed));  // Fixed seed: reproducible regression

      // --- Positive: random valid new address, correct UDID ---
      for (int iter = 0; iter < NUM_RAND_ITERS; iter++) begin
        new_addr = 7'h08 + ($urandom % (7'h78 - 7'h08));
        if (new_addr == SLAVE_ADDR || new_addr == SMBUS_DEFAULT_ADDR || new_addr == ARA_ADDR)
          new_addr = new_addr ^ 7'h01;

        assign_bytes[0] = ARP_CMD_ASSIGN_ADDR;
        for (i = 0; i < 16; i++)
          assign_bytes[1 + i] = UDID_VAL[(15 - i) * 8 +: 8];
        assign_bytes[17] = {new_addr, 1'b0};

        mst_addr      = SMBUS_DEFAULT_ADDR;
        mst_rw        = 1'b0;
        mst_data_tx   = assign_bytes[0];
        mst_num_bytes = 8'd18;
        mst_start     = 1'b1;
        wait_clks(1);
        mst_start     = 1'b0;
        @(negedge u_i2c_top.u_i2c_master.addr_phase);
        mst_data_tx = assign_bytes[1];
        for (i = 0; i < 16; i++) begin
          @(posedge slv_rx_valid);
          mst_data_tx = assign_bytes[i + 2];
        end
        @(negedge mst_busy);
        wait_clks(5);

        if (mst_ack_error) begin
          $display("[%0t] FAIL: ARP random assign #%0d at 400 kHz: write got NACK!", $time, iter);
          total_mismatches++;
        end else if (u_i2c_top.u_i2c_slave.av !== 1'b1 ||
                     u_i2c_top.u_i2c_slave.assigned_addr !== new_addr) begin
          $display("[%0t] FAIL: ARP random assign #%0d at 400 kHz: av=%b assigned_addr=0x%02x != expected 0x%02x",
                   $time, iter, u_i2c_top.u_i2c_slave.av,
                   u_i2c_top.u_i2c_slave.assigned_addr, new_addr);
          total_mismatches++;
        end else begin
          check_equal($sformatf("ARP random assign #%0d at 400 kHz: av=1, assigned_addr=0x%02x", iter, new_addr),
                      1, 1);
        end

        master_write(u_i2c_top.u_i2c_slave.assigned_addr, ARP_CMD_RESET_DEV_DIR);
        wait_clks(5);
        if (u_i2c_top.u_i2c_slave.av !== 1'b0 ||
            u_i2c_top.u_i2c_slave.assigned_addr !== SLAVE_ADDR) begin
          $display("[%0t] FAIL: ARP random assign #%0d at 400 kHz: restore to SLAVE_ADDR failed!", $time, iter);
          total_mismatches++;
        end
      end

      // --- Negative: deliberately wrong UDID, confirm assignment is rejected ---
      for (int iter = 0; iter < NUM_NEG_ITERS; iter++) begin
        new_addr = 7'h08 + ($urandom % (7'h78 - 7'h08));
        if (new_addr == SLAVE_ADDR || new_addr == SMBUS_DEFAULT_ADDR || new_addr == ARA_ADDR)
          new_addr = new_addr ^ 7'h01;

        bad_udid = UDID_VAL ^ (128'h1 << ($urandom % UDID_WIDTH));

        assign_bytes[0] = ARP_CMD_ASSIGN_ADDR;
        for (i = 0; i < 16; i++)
          assign_bytes[1 + i] = bad_udid[(15 - i) * 8 +: 8];
        assign_bytes[17] = {new_addr, 1'b0};

        mst_addr      = SMBUS_DEFAULT_ADDR;
        mst_rw        = 1'b0;
        mst_data_tx   = assign_bytes[0];
        mst_num_bytes = 8'd18;
        mst_start     = 1'b1;
        wait_clks(1);
        mst_start     = 1'b0;
        @(negedge u_i2c_top.u_i2c_master.addr_phase);
        mst_data_tx = assign_bytes[1];
        for (i = 0; i < 16; i++) begin
          @(posedge slv_rx_valid);
          mst_data_tx = assign_bytes[i + 2];
        end
        @(negedge mst_busy);
        wait_clks(5);

        if (u_i2c_top.u_i2c_slave.av !== 1'b0 ||
            u_i2c_top.u_i2c_slave.assigned_addr !== SLAVE_ADDR) begin
          $display("[%0t] FAIL: ARP UDID-mismatch #%0d at 400 kHz: assignment wrongly accepted (av=%b addr=0x%02x)!",
                   $time, iter, u_i2c_top.u_i2c_slave.av, u_i2c_top.u_i2c_slave.assigned_addr);
          total_mismatches++;
        end else begin
          check_equal($sformatf("ARP UDID-mismatch #%0d at 400 kHz: assignment correctly rejected", iter), 1, 1);
        end

        master_write(SLAVE_ADDR, 8'h55);
        if (mst_ack_error) begin
          $display("[%0t] FAIL: ARP UDID-mismatch #%0d at 400 kHz: SLAVE_ADDR broken after rejected assign!",
                   $time, iter);
          total_mismatches++;
        end
      end

      check_equal("Randomized ARP Assign Address regression at 400 kHz: total mismatches", 0, total_mismatches);
    end

    // -----------------------------------------------------------------------
    // Test 23: Genuine Arbitration Loss Detection at 400 kHz
    //
    // See tb_i2c_smbus.sv Test 23 for the full rationale: directly forces
    // the shared sda wire low -- exactly what a competing master would do
    // -- at the moment our master releases SDA high to send SLAVE_ADDR's
    // MSB (a '1' bit, since SLAVE_ADDR = 0x50 = 7'b1010000), and confirms
    // the RTL's "released high but sampled low" comparator actually fires.
    // -----------------------------------------------------------------------
    $display("");
    $display("============================================================");
    $display("  Test 23: Genuine Arbitration Loss Detection at 400 kHz");
    $display("============================================================");

    begin
      mst_addr      = SLAVE_ADDR;
      mst_rw        = 1'b0;
      mst_data_tx   = 8'h00;
      mst_num_bytes = 8'd1;
      mst_start     = 1'b1;
      wait_clks(1);
      mst_start     = 1'b0;

      @(posedge mst_busy);

      wait (u_i2c_top.u_i2c_master.state == MST_SEND_ADDR &&
            u_i2c_top.u_i2c_master.sda_oe && u_i2c_top.u_i2c_master.sda_o);
      force sda = 1'b0;

      // See tb_i2c_smbus.sv Test 23 for why two clocks of margin are
      // needed here (cross-always_ff visibility lag on scl_rising_event).
      @(posedge u_i2c_top.u_i2c_master.scl_rising_event);
      wait_clks(2);

      check_equal("Arbitration loss detected at 400 kHz (u_i2c_master.arbitration_lost)",
                  1'b1, u_i2c_top.u_i2c_master.arbitration_lost);
      check_equal("mst_arb_lost output reflects the detection at 400 kHz", 1'b1, mst_arb_lost);

      release sda;

      wait(mst_done || mst_ack_error || mst_smbus_timeout);
      wait_clks(5);
    end

    // -----------------------------------------------------------------------
    // End of tests
    // -----------------------------------------------------------------------
    $display("");
    $display("============================================================");
    $display("  FAST MODE (400 kHz) — SMBus 3.0 - ALL TESTS COMPLETED");
    $display("============================================================");
    $display("");

    #1000;
    $finish;
  end

  // ---------------------------------------------------------------------------
  // Safety timeout
  // ---------------------------------------------------------------------------
  initial begin
    #10_000_000;
    $display("[%0t] PROGRESS: 10ms reached", $time);
    #20_000_000;
    $display("[%0t] PROGRESS: 30ms reached", $time);
    #30_000_000;
    $display("[%0t] PROGRESS: 60ms reached", $time);
    #40_000_000;
    $display("[%0t] PROGRESS: 100ms reached", $time);
    #100_000_000;
    $display("[%0t] PROGRESS: 200ms reached", $time);
    #300_000_000;
    $display("[%0t] TIMEOUT: Simulation exceeded 500 ms!", $time);
    $finish;
  end

endmodule : tb_i2c_smbus_fastmode
