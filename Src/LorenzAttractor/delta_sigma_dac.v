	

`timescale 1ns / 1ps


module delta_sigma_dac(clk, PWM_in, PWM_out);
input wire clk;
input wire [15:0] PWM_in;
output reg PWM_out = 0;

reg [16:0] PWM_accumulator = 0;
always @(posedge clk) 
begin
     PWM_accumulator = PWM_accumulator + PWM_in;
     PWM_out = PWM_accumulator[16];
     PWM_accumulator[16] = 0;
end 
endmodule