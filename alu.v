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

module alu(a, b, o, op, of, uf,  err, zero);
    parameter N = 4;
    input [N-1:0] a,b;
    input [3:0] op;
    
    output reg [N-1:0] o;
    output reg of, uf, err, zero;
    
    
    always @(a or b or op)
    begin
        of <= 0;
        uf <= 0;
        err <= 0;
        zero <= 0;
        
        casex(op)
            4'b0000: {of,o} <= a + b;
            4'b0001: {uf,o} <= a - b;
            4'b0010: o <= a << b;
            4'b0011: o <= a >> b;
            4'b0100: zero <= a==b;
            4'b0101: zero <= a > b;
            4'b0110: zero <= a < b;
            4'b0111, 4'b1xxx: err <= 1;
        
        endcase
    
    end
    
endmodule




module test;
    parameter N = 4;
    reg [N-1:0] a,b;
    reg [3:0] op;
    
    wire [N-1:0] o;
    wire of,uf, err, zero;
    
    reg [N-1:0] rez;
    
   
    alu #(N) alu_inst(a,b,o,op,of,uf,err,zero);
    
    task multiply(input integer at, input integer bt, output integer rez);
    begin
        rez = 0;

        while(bt)
            begin
                op=4'b0000;
                a = rez;
                b = at;
                #10
                rez = o;
                bt = bt - 1; 
            end
        
            $display("Rezultat = %0d", rez);

    end       
    endtask
    
    initial
    begin
        
        a=0; b=0; 
        multiply(2,3,rez);
        multiply(3,3,rez);
       
    end
    
    initial
        #100 $finish;
        
endmodule 