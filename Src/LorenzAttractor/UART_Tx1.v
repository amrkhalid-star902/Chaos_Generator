`timescale 1ns / 1ps



module UART_Tx1
    #(parameter CLKS_PER_BIT = 435)
    (
        input clk,
        input Tx_s,
        input [7:0] Tx_Data,
        output Tx_Active,
        output reg Tx_serial_out,
        output Tx_Done
    );
    
    // States of finite state machine
    localparam TX_IDLE = 3'b000;
    localparam TX_START_BIT = 3'b001;
    localparam TX_DATA_SEND = 3'b010;
    localparam TX_STOP = 3'b011;
    localparam TX_CLEAN = 3'b100;
    
    reg [2:0] MAIN_STATE = 0;
    reg [9:0] clk_count = 0;
    reg [2:0] Bit_Index = 0;
    reg [7:0] Send_Data = 0;
    reg       Done = 0;
    reg       Active = 0;
    
    
    always@(posedge clk)
    begin
    
        case(MAIN_STATE)
        
            TX_IDLE:
                begin
                
                    // make output line high to indicate idle state
                    Tx_serial_out <= 1'b1;
                    Done <= 1'b0;
                    clk_count <= 1'b0;
                    Bit_Index <= 1'b0;
                    
                    if(Tx_s == 1'b1)
                        begin
                        
                            Active <= 1'b1;
                            Send_Data <= Tx_Data;
                            MAIN_STATE <= TX_START_BIT;
                            
                        end
                        
                     else
                        MAIN_STATE <= TX_IDLE;
                end
            
            TX_START_BIT:
                begin
                
                    //generate start bit by pulling output line to low
                    Tx_serial_out <= 1'b0;
                    
                    //Wait for period equal to CLK_PER_PERIOD to finish send start bit
                    if(clk_count < CLKS_PER_BIT - 1)
                        begin
                        
                            clk_count <= clk_count + 1;
                            MAIN_STATE <= TX_START_BIT;
                        end
                    
                    else
                        begin    
                            
                            clk_count <= 1'b0;
                            MAIN_STATE <= TX_DATA_SEND;
                        end
                end       
        
            TX_DATA_SEND:
                begin
                
                    // Wait CLKS_PER_BIT-1 clock cycles for data bits to finish 
                    Tx_serial_out <= Send_Data[Bit_Index];
                    
                    if(clk_count < CLKS_PER_BIT - 1)
                        begin
                            
                            clk_count <= clk_count + 1;
                            MAIN_STATE <= TX_DATA_SEND;
                        
                        end
                    
                    else
                        begin
                            
                            clk_count <= 1'b0;
                            
                            // Check if we have sent out all bits
                            if(Bit_Index < 7)
                                begin
                                
                                    Bit_Index <= Bit_Index + 1;
                                    MAIN_STATE <= TX_DATA_SEND;
                                    
                                end
                                
                            else
                                begin
                                    
                                    Bit_Index <= 1'b0;
                                    MAIN_STATE <= TX_STOP;
                                
                                end
                                
                        end
                    
                end
                
            TX_STOP:
                begin
                    
                    // Send out Stop bit.  By pulling ouput line to high
                    Tx_serial_out <= 1'b1;
                    
                    // Wait CLKS_PER_BIT-1 clock cycles for Stop bit to finish
                    if(clk_count < CLKS_PER_BIT - 1)
                        begin
                        
                            clk_count <= clk_count + 1;
                            MAIN_STATE <= TX_STOP;
                           
                        end
                        
                    else
                        begin
                            
                            Done <= 1'b1;
                            clk_count <= 1'b0;
                            MAIN_STATE <= TX_CLEAN;
                            Active <= 1'b0;
                            
                        end    
                end
                
            TX_CLEAN:
                begin
                
                    Done <= 1'b1;
                    MAIN_STATE <= TX_IDLE;
                    
                end
                
            default:
                MAIN_STATE <= TX_IDLE;    
        
        endcase
    
    end
    
    
    assign Tx_Active = Active;
    assign Tx_Done = Done;
    
   
endmodule
