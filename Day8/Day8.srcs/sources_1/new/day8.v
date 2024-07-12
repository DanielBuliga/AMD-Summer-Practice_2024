`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12.07.2024 11:34:22
// Design Name: 
// Module Name: day8
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

module reg_pc(clk, in, out);
    input clk;
    input [31:0] in;
    output reg [31:0] out;
    
    always @ (posedge clk)
        out <= in;
    
endmodule 


module alu_sum(in, out);
    input [31:0] in;
    output reg [31:0] out;
    
    always @(in)
        out <= in + 4;
        
endmodule 


module im(addr, instr);
    input [31:0] addr; //input 
    output reg [31:0] instr; //output
    
    reg [7:0] data [31:0];
    
    initial
        $readmemb("im.mem", data);
    
    always @(addr)
    begin
        instr[7:0] <= data[addr];
        instr[15:8] <= data[addr + 1];
        instr[23:16] <= data[addr + 2];
        instr[31:24] <= data[addr + 3];
    end
    
endmodule


module alu(a, b, alu_op, out, zero);
    input [31:0] a,b; 
    input [3:0] alu_op;
    output reg zero;
    output reg [31:0] out;
    
    always @( a or b or alu_op)
    begin
        case(alu_op)
            4'b0000: begin out <= a & b;  zero <= (~out) ? 1 : 0; end
            4'b0001: begin out <= a | b;  zero <= (~out) ? 1 : 0; end
            4'b0010: begin out <= a + b;  zero <= (~out) ? 1 : 0; end
            4'b0110: begin out <= a - b;  zero <= (~out) ? 1 : 0; end
            4'b0111: begin out <= (a < b) ? 1 : 0;  zero <= (~out) ? 1 : 0; end
            4'b1100: begin out <= ~(a | b);  zero <= (~out) ? 1 : 0; end
            default: begin out <= 32'b0;  zero <= (~out) ? 1 : 0; end
        endcase
    end
    
endmodule


module mux2_1(in1, in2, sel, out);
    parameter N = 32;
    input [N-1:0] in1, in2;
    input  sel;
    output reg [N-1:0] out;
    
    always @ (in1 or in2 or sel)
    begin
        case(sel)
            1'b0: out <= in1;
            1'b1: out <= in2;
        endcase
    end
    
endmodule


module ext_sign(in, ext_op, out);
    input [15:0] in;
    input ext_op;
    output reg [31:0] out;
    
    //always @(in or ext_op)
    //    out <= { {16{ext_op}}, in};
    
    always @ (in or ext_op)
    begin
        if(ext_op)
            out <= {{16{in[15]}}, in};
        else
            out <= in;
    end
    
endmodule 


module register_bank(clk, ra1, ra2, wa, wd, reg_write, rd1, rd2);
    input clk, reg_write;
    input [4:0] ra1, ra2, wa;
    input [31:0] wd;
    output reg [31:0] rd1, rd2;
    
    reg [31:0] rg [31:0];
    
    //initializare toate reg cu 0
    //reg [4:0] i;
    // for (i=0; i<32; i=i+1)
       //rg[i]=32'b0;
    
    
    always @(posedge  clk)
    begin
        if(reg_write)
            rg[wa] = wd;
    end
    
    always @(negedge clk)
    begin
        rd1 = rg[ra1];
        rd2 = rg[ra2];
    end
    
    
endmodule 


module dm(clk, address, wd, mem_write, rd);
    input clk, mem_write;
    input [31:0] address, wd;
    output reg [31:0] rd;
    
    reg [7:0] mem [31:0] ;
    
    always @(posedge clk)
    begin
        if(mem_write)
        begin
            
        end
            
            
            
    end
    
    always @(negedge clk)
    begin
        //wd = {};
    end
    
endmodule


module main_control();
    
endmodule

