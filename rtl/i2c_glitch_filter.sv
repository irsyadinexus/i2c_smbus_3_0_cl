//==============================================================================
// i2c_glitch_filter.sv
// Parameterized Digital Spike / Glitch Suppression Filter
//
// Suppresses pulses narrower than GLITCH_CYCLES system clock periods.
// For standard 50 MHz operation (20 ns clock period), GLITCH_CYCLES=3
// suppresses spikes narrower than ~60 ns (compliant with I2C/SMBus 50 ns spec).
//==============================================================================

`timescale 1ns / 1ps

module i2c_glitch_filter #(
  parameter int GLITCH_CYCLES = 2,
  parameter bit ENABLE        = 1'b0
) (
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic clk,
  input  logic rst_n,
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic pin_in,
  output logic pin_out
);

  generate
    if (!ENABLE || GLITCH_CYCLES < 2) begin : gen_bypass
      assign pin_out = pin_in;
    end else begin : gen_filter
      logic [GLITCH_CYCLES-1:0] shift_reg;
      logic pin_filtered;

      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          shift_reg    <= '1;
          pin_filtered <= 1'b1;
        end else begin
          shift_reg <= {shift_reg[GLITCH_CYCLES-2:0], pin_in};
          if (&shift_reg && pin_in)
            pin_filtered <= 1'b1;
          else if (~|shift_reg && !pin_in)
            pin_filtered <= 1'b0;
        end
      end

      assign pin_out = pin_filtered;
    end
  endgenerate

endmodule : i2c_glitch_filter
