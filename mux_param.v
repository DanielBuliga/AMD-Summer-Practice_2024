`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/01/2024 04:57:41 PM
// Design Name: 
// Module Name: mux_param
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module mux_param_behavioral(in1, in2, sel, out);
    parameter N=8;
    
    input [N-1:0] in1;
    input [N-1:0] in2;
    input sel;
    
    output reg [N-1:0] out;
    
    always @ (in1 or in2 or sel)
    begin
        if(!sel)
        begin
            out <= in1;
        end
        
        if(sel)
        begin
            out <= in2;
        end 
    end  
     
endmodule


module test;
    parameter N=8;
    
    reg [N-1:0] in1, in2;
    reg sel;
    
    wire [N-1:0] out;
    
    mux_param_behavioral #(8) M1(in1, in2, sel, out);
    //mux_param_behavioral #32 M2(in1, in2, sel, out);
    
    initial 
    begin   
        #0 in1=8'b0; in2=8'b0; sel=0;
        #10 in1=8'b1000_0000;
        #10 in2=8'b0000_1001;
        #10 sel=1'b1;
        #10 sel=1'b0;
        #10 in1=8'b0000_0000;
        #10 in2=8'b0000_0010;
        #10 sel=1;
    end
    
    initial
        #180 $finish; 
       
endmodule 
