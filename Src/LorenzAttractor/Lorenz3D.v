`timescale 1ns / 1ps


module Lorenz3D(
    clk,
    pinX,
    pinY,
    pinZ,
	serial_data
	 
    );
	 
	 
	input wire clk;
    output wire pinX;
    output wire pinY;
    output wire pinZ;
    output wire serial_data;
	 
	
    reg [2:0] i = 0;
    wire [7:0] inputdata;
    wire       done_bit;
    reg        start = 0;
    wire       active;
    wire       out_serial;
    reg        triger = 1;
    reg [3:0] count = 0;
	 

	wire signed [31:0] x , y , z , xm , ym , zm;
	wire signed [31:0] scale = {7'b0001001 , 25'b0000001111010111000010100};
	 
	 
	reg [31:0] key; 
	 
	 
	 
	wire slowclock;
	 
	Slowclock clock ( .clk(clk), .slowclock(slowclock));
	 
	 
	 
	 
	 
    Lorenz  L1(
     .clk(slowclock),
     .x(x),
     .y(y),
     .z(z)
    );
	 

	UART_Tx1  #(435) U1 ( .clk(clk), .Tx_s(start), .Tx_Data(inputdata), .Tx_Active(active),  .Tx_serial_out(out_serial), .Tx_Done(done_bit));
	 
	 
	always@(posedge clk)
    begin
        
        if(done_bit == 1'b1)
        begin
            i = i + 1;
        end
        
        if( i == 3'b000 && triger == 1'b1)
        begin
        
          
            key = x;            
            start = 1;
            triger = 0;
            
        end
        

        
        if(i == 3'b100)
        begin
            
            i = 0;
            triger = 1;
            
        end
        
        
    end
    
	 
	 
	delta_sigma_dac d1 (.clk(clk), .PWM_in(x[31:9]), .PWM_out(pinX));
    delta_sigma_dac d2 (.clk(clk), .PWM_in(y[31:9]), .PWM_out(pinY));
	delta_sigma_dac d3 (.clk(clk), .PWM_in(z[31:9]), .PWM_out(pinZ));
	
	signed_mult sm1 (.a(x) , .b(scale) , .out(xm));
	signed_mult sm2 (.a(y) , .b(scale) , .out(ym));
	signed_mult sm3 (.a(z) , .b(scale) , .out(zm));
	 
	assign serial_data = out_serial;
    assign inputdata = key[((i + 1) * 8 - 1) -: 8];
    
endmodule
