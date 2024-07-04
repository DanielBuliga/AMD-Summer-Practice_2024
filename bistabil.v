`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/04/2024 09:35:36 AM
// Design Name: 
// Module Name: bistabil
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


module bistabil(d,clk,q,qneg);
    input d, clk;
    //input reset; ??
    output q, qneg;
    
    assign q = d;
    assign qneg = ~q;
    
     //always @(posedge clk)
     //begin
            //q <= d;
            //qneg <= ~q; 
endmodule

module test;
    reg d, clk;
    wire q, qneg;
    
    bistabil b(d,clk,q,qneg);
    
    initial
    begin
        #0 clk=1'b0;
        forever #5 clk=~clk;
    end
    
    initial
    begin
        #0 d=1'b0; clk=1'b0; 
        #10 d=1'b1;
        #20 d=1'b1;
        #10 d=1'b0;
        #20 d=1'b0;
        #20 d=1'b1;
        #150 $finish;
    end

endmodule 


