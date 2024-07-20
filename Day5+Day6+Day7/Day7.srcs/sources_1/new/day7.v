`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/12/2024 08:35:52 AM
// Design Name: 
// Module Name: day7
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

module digit_dec(din,a,b,c,d,e,f,g);
    input [3:0] din;
    output reg  a,b,c,d,e,f,g;
    
    always @(din)
    begin
        case(din)
            4'b1010: {a,b,c,d,e,f,g} = ~7'b1110111;
            4'b1011: {a,b,c,d,e,f,g} = ~7'b0011111;
            4'b1100: {a,b,c,d,e,f,g} = ~7'b1001110;
            4'b1101: {a,b,c,d,e,f,g} = ~7'b0111101;
            4'b1110: {a,b,c,d,e,f,g} = ~7'b1001111;
            4'b1111: {a,b,c,d,e,f,g} = ~7'b1000111;
        
            4'b0000: {a,b,c,d,e,f,g} = ~7'b1111110;
            4'b0001: {a,b,c,d,e,f,g} = ~7'b0110000;
            4'b0010: {a,b,c,d,e,f,g} = ~7'b1101101;
            4'b0011: {a,b,c,d,e,f,g} = ~7'b1111001;
            4'b0100: {a,b,c,d,e,f,g} = ~7'b0110011;
            4'b0101: {a,b,c,d,e,f,g} = ~7'b1011011;
            4'b0110: {a,b,c,d,e,f,g} = ~7'b1011111;
            4'b0111: {a,b,c,d,e,f,g} = ~7'b1110000;
            4'b1000: {a,b,c,d,e,f,g} = ~7'b1111111;
            4'b1001: {a,b,c,d,e,f,g} = ~7'b1111011;
             
        endcase
    end
endmodule

module decod(ain, aout);
    input [1:0] ain;
    output reg [3:0] aout;
    
    always @(ain)
    begin
        case(ain)
            2'b00: aout = ~4'b0001;
            2'b01: aout = ~4'b0010;
            2'b10: aout = ~4'b0100;
            2'b11: aout = ~4'b1000;
        endcase
    end
    
endmodule  

module cnt(clk,out);
    input clk;
    output reg [1:0] out;    
    
    always @(posedge clk)
    begin
  
        if(out < 2'b11)
            out <= out + 1;
        else
            out <= 2'b00;
         
    end
    
endmodule 
/*
module mux1(in,out,sel);
    input [15:0] in;
    input [1:0] sel;
    output reg [3:0] out;
    
    always @(in or sel)
    begin
        case(sel)
            2'b00: out = in[3:0];
            2'b01: out = in[7:4];
            2'b10: out = in[11:8];
            2'b11: out = in[15:12];
        endcase
    end
   
endmodule 
*/


module mux(in3, in2, in1, in0, out, sel);
	parameter SIZE = 4;
    input [SIZE-1:0] in0, in1, in2, in3;
    input [1:0] sel;
    output reg [SIZE-1:0] out;

    always@(sel)
        case(sel)
            2'b00: out = in0;
            2'b01: out = in1;
            2'b10: out = in2;
            2'b11: out = in3;
        endcase

endmodule


module divizor(clk, clk_out);
    input clk;
    output reg clk_out;
    
    parameter d = 138_875; //2; //138_875;
    
    reg [31:0] cntr;
    
    always @(posedge clk)
    begin
        if(cntr >= d - 1)
            begin
                cntr <= 0;
                clk_out <= ~clk_out;
            end
        else
            cntr <= cntr + 1;
    end
    
endmodule


module alu(a, b, op, o, of, uf, err, zero);
    parameter N = 12;
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
        
        case(op)
            4'b0000: begin {of,o} <= a + b; zero <= 0; end
            4'b0001: begin {uf,o} <= a - b; zero <= 0; end
            4'b0010: begin o <= a << b; zero <= 0; of <= 0; uf <= 0; end
            4'b0011: o <= a >> b;
            4'b0100: zero <= (a==b) ? 1'b1 : 1'b0;
            4'b0101: zero <= (a > b) ? 1'b1 : 1'b0;
            4'b0110: zero <= (a < b) ? 1'b1 : 1'b0;
            //4'b0111, 4'b1xxx: err <= 1;
            default: err <= 1;
        
        endcase
    
    end
    
endmodule


module register(clk, pl, din, dout);
    input clk, pl;
    input[11:0] din;
    output reg[11:0] dout;
    
    always@(posedge clk)
        if(pl)
            dout <= din;
            
endmodule


module dec_errf(in, out); 

    input[3:0] in;
    output reg[3:0] out;
    //{ERR, UF, OF, ZERO}
    always@(in)
        case(in)
            4'b0001: out = 4'b0010; //2 -> zero
            4'b0010: out = 4'b1000; // 8 -> ov
            4'b0100: out = 4'b1000; // 8 -> uf
            4'b1000: out = 4'b1110; // E -> err
            default: out = 4'b0000; // 0 
        endcase
endmodule


module dec_bt(in, out);
    input[2:0] in;
    output reg[1:0] out;
    // BT2 BT1 BT0
    always@(in)
        case(in)
            3'b000: out = 2'b00;
            3'b001: out = 2'b01;
            3'b010: out = 2'b10;
            3'b100: out = 2'b11;
        endcase
endmodule

module top(clk, in, pl, BT0, BT1, BT2, a, b, c, d, e, f, g, A);
    
    input clk, pl, BT0, BT1, BT2;
    input [11:0] in;
    output a,b,c,d,e,f,g;
    output [3:0] A;

    wire [3:0] mux_out;
    wire [1:0] cnt_out;
    wire div_out;
    wire[11:0] r1_out, r2_out, regop;
    //wire[3:0] regop;
    
    wire ERR, OF, UF, ZERO;
    wire [11:0] alu_out, mux2_out;
    
    wire[1:0] debug_cmd;
    wire[3:0] dec_errf_out;
    /*
    divizor dvz(clk, div_out);
    cnt ct(div_out, cnt_out);
    decod dec(cnt_out, A);
    */
    divizor div(clk, div_out);
    cnt count(div_out, cnt_out);
    //mux m(in, mux_out, cnt_out);
    digit_dec dd(mux_out,a,b,c,d,e,f,g);
    decod dec(cnt_out, A);
    
    
    
    register r1(div_out, pl & BT1, in, r1_out); 
    register r2(div_out, pl & BT0, in, r2_out); 
    register rop(div_out, pl & BT2, in, regop);
    
    alu alui(r1_out, r2_out, regop[3:0], alu_out, OF, UF, ERR, ZERO);
    
    mux #(12) mux2(regop, r1_out, r2_out, alu_out, mux2_out, debug_cmd);
    
    dec_bt decbt({BT2,BT1,BT0}, debug_cmd);
    
    dec_errf derr({ERR, UF, OF, ZERO}, dec_errf_out);
    
    mux mux1(dec_errf_out, mux2_out[11:8], mux2_out[7:4], mux2_out[3:0], mux_out, cnt_out);

endmodule