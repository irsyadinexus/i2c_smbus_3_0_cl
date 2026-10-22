`timescale 1ns / 1ps

module tb_i2c_glitch_filter();
  logic clk = 0;
  logic rst_n = 0;
  logic pin_in = 1;
  wire  pin_out;

  always #10 clk = ~clk; // 50 MHz (20 ns period)

  i2c_glitch_filter #(.GLITCH_CYCLES(3), .ENABLE(1'b1)) dut (
    .clk, .rst_n, .pin_in, .pin_out
  );

  initial begin
    #100 rst_n = 1;
    #40;
    // Inject a 1-clock (20 ns) glitch low
    @(posedge clk);
    pin_in = 0;
    @(posedge clk);
    pin_in = 1;

    // Wait and verify pin_out stayed 1
    #60;
    if (pin_out !== 1'b1) begin
      $display("FAIL: Glitch was NOT suppressed!");
      $fatal;
    end else begin
      $display("PASS: Glitch (20 ns) successfully suppressed.");
    end

    // Now inject stable low (4 clocks = 80 ns)
    @(posedge clk);
    pin_in = 0;
    repeat (4) @(posedge clk);
    if (pin_out !== 1'b0) begin
      $display("FAIL: Valid signal was suppressed!");
      $fatal;
    end else begin
      $display("PASS: Valid transition propagated correctly.");
    end

    // Now inject 1-clock (20 ns) glitch high while stable low
    @(posedge clk);
    pin_in = 1;
    @(posedge clk);
    pin_in = 0;

    #60;
    if (pin_out !== 1'b0) begin
      $display("FAIL: High glitch while low was NOT suppressed!");
      $fatal;
    end else begin
      $display("PASS: High glitch (20 ns) successfully suppressed.");
    end

    // Now inject stable high (4 clocks)
    @(posedge clk);
    pin_in = 1;
    repeat (4) @(posedge clk);
    if (pin_out !== 1'b1) begin
      $display("FAIL: Valid high transition was suppressed!");
      $fatal;
    end else begin
      $display("PASS: Valid high transition propagated correctly.");
    end

    $display("ALL GLITCH FILTER TESTS PASSED");
    $finish;
  end
endmodule
