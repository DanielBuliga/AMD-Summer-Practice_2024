`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/01/2024 06:07:34 PM
// Design Name: 
// Module Name: mux_param_structural
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
/*
module not_gate(in1, out1);
    parameter N=1;
    input  in1;
    output  out1;
    
    assign out1 = ~ in1;
    
endmodule

 
module and_gate(i1, i2, out2);
    parameter N=32;
    
    input [N-1:0] i1, i2;
    output [N-1:0] out2;
    
    assign out2 = i1 & i2;
    
endmodule


module or_gate(inp1, inp2, out3);
    parameter N=32;
    
    input [N-1:0] inp1, inp2;
    output [N-1:0] out3;
    
    assign out3 = inp1 | inp2;
    
endmodule


module mux_param_structural(in1, in2, sel, outp);
    parameter N=32;
    
    input [N-1:0] in1;
    input [N-1:0] in2;
    input sel;
    
    output [N-1:0] outp;
    
    wire [N-1:0] and1_out, and2_out, not_out;
    
    and_gate #(32)and_1(in2, {N{sel}}, and2_out);
    not_gate not_G(sel, not_out);
    and_gate #(32) and_2(in1, {N{not_out}}, and1_out);
    or_gate #(32) or_G(and1_out, and2_out, outp); 
     
endmodule



module tb;
    parameter N=32;
    
    reg [N-1:0] in1, in2;
    reg sel;
    //wire [N-1:0] and1_out, and2_out, not_out;
    
    wire [N-1:0] out;
    
    
    //not_gate  not_G(sel, not_out); 
    //and_gate #(32)and_1(in2, {31'b0, sel}, and2_out);
    //and_gate #(32) and_2(in1, {31'b0, not_out}, and1_out);
    //r_gate #(32) or_G(and1_out, and2_out, out); 
    mux_param_structural #(32) M2(in1, in2, sel, out);
    //mux_param_behavioral #32 M2(in1, in2, sel, out);
    
    initial 
    begin   
        #0 in1=32'h0000_0000; in2=32'h0000_0000; sel=1'b0;
        #10 in1=32'h0000_0001; in2=32'h0000_1000;
        #10 sel=1'b1;
        #10 sel=1'b0;
        #10 in1=32'h0000_0017;
        #10 in2=32'h0000_0001;
        #10 sel=1;
    end
    
    initial
        #180 $finish; 
       
endmodule 

*/
/*
module not_gate #(parameter N = 1)(input [N-1:0] in1, output [N-1:0] out1);
    assign out1 = ~in1;
endmodule

module and_gate #(parameter N = 1)(input [N-1:0] i1, i2, output [N-1:0] out2);
    assign out2 = i1 & i2;
endmodule

module or_gate #(parameter N = 1)(input [N-1:0] inp1, inp2, output [N-1:0] out3);
    assign out3 = inp1 | inp2;
endmodule

module mux_param_structural #(parameter N = 32)(
    input [N-1:0] in1,
    input [N-1:0] in2,
    input sel,
    output [N-1:0] outp
);
    wire [N-1:0] and1_out, and2_out, not_out;

    and_gate #(N) and_1(in2, {N{sel}}, and2_out);
    not_gate #(1) not_G({sel}, not_out);
    and_gate #(N) and_2(in1, {N{not_out[0]}}, and1_out);
    or_gate #(N) or_G(and1_out, and2_out, outp); 
endmodule

module tb;
    parameter N = 32;
    
    reg [N-1:0] in1, in2;
    reg sel;
    
    wire [N-1:0] out;
    //wire [N-1:0] and1_out, and2_out;
    //wire [0:0] not_out;

    mux_param_structural #(N) M2(in1, in2, sel, out);
    
    initial 
    begin   
        #0 in1 = 32'h0000_0000; in2 = 32'h0000_0000; sel = 1'b0;
        #10 in1 = 32'h0000_0001; in2 = 32'h0000_1000;
        #10 sel = 1'b1;
        #10 sel = 1'b0;
        #10 in1 = 32'h0000_0017;
        #10 in2 = 32'h0000_0001;
        #10 sel = 1'b1;
    end
    
    initial
        #180 $finish; 
       
endmodule

*/

module not_gate(in1, out1);
    //parameter N=1;
    input  in1;
    output  out1;
    
    assign out1 = ~ in1;
    
endmodule

module and_gate(i1, i2, out2);
    parameter N=32;
    
    input [N-1:0] i1, i2;
    output [N-1:0] out2;
    
    assign out2 = i1 & i2;
    
endmodule


module or_gate(inp1, inp2, out3);
    parameter N=32;
    
    input [N-1:0] inp1, inp2;
    output [N-1:0] out3;
    
    assign out3 = inp1 | inp2;
    
endmodule


module tb;
    parameter N = 32;
    
    reg [N-1:0] in1, in2;
    reg sel;
    wire [N-1:0] and1_out, and2_out;
    wire not_out;
    wire [N-1:0] out;
    
    and_gate #(32)and_1(in2, {N{sel}}, and2_out);
    not_gate #(1) not_G(sel, not_out);
    and_gate #(32) and_2(in1, {N{not_out}}, and1_out);
    or_gate #(32) or_G(and1_out, and2_out, out); 
    
    initial 
    begin   
        #0 in1 = 32'h0000_0000; in2 = 32'h0000_0000; sel = 1'b0;
        #10 in1 = 32'h0000_0001; in2 = 32'h0000_1000;
        #20 sel = 1'b1;
        #20 sel = 1'b0;
        #10 in1 = 32'h0000_0017;
        #10 in2 = 32'h0000_0001;
        #20 sel = 1'b1;
        #20 sel = 1'b0;
    end
    
    initial
        #180 $finish; 
endmodule
