`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/15/2024 02:14:32 PM
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
    
        rd1 = rg[ra1];
        rd2 = rg[ra2];
        
    end
    
    /*end
    
    always @(negedge clk)
    begin
        rd1 = rg[ra1];
        rd2 = rg[ra2];
    end
    */
endmodule 


module dm(clk, address, wd, mem_write, rd);
    input clk, mem_write;
    input [31:0] address, wd;
    output reg [31:0] rd;
    
    reg [7:0] mem [31:0] ;
    
    always @(posedge clk)
    begin
       
        if(mem_write)
            mem[address] = wd; 
        
        rd = mem[address];
        
        //else
            //rd = mem[address];           
    end
    /*
    always @(negedge clk)
    begin
        //wd = {};
        //rd = mem[address];
    end
    */
endmodule


module main_control(func, opcode, zero, reg_dst, reg_write, ext_op, alu_src, alu_op, mem_write, mem_to_reg);
    input [5:0] func, opcode;
    input zero;
    output reg reg_dst, reg_write, ext_op, alu_src, mem_write, mem_to_reg;
    output reg [3:0] alu_op;
    
    always @(func or opcode)
    begin 
        case(opcode)
            6'b000000:              // R-TYPE
                begin            
                    reg_dst = 1;
                    alu_src = 0;
                    mem_to_reg = 0;
                    reg_write = 1;
                    mem_write = 0;
                    ext_op = 0;   //0 sau 1 ???
                    
                    case(func)
                        6'b100000: alu_op = 4'b0010;    //ADD
                        6'b100010: alu_op = 4'b0110;    //SUB
                        6'b100100: alu_op = 4'b0000;    //AND
                        6'b100101: alu_op = 4'b0001;    //OR
                        6'b101010: alu_op = 4'b0111;    //SLT
                        default: alu_op = 4'b1100;      //NOR
                    endcase
                    
                end
        
            6'b001000:              // I-TYPE -> ADDI
                begin
                    reg_dst = 0;
                    alu_src = 1;
                    mem_to_reg = 0;
                    reg_write = 1;
                    mem_write = 0;
                    ext_op = 0;
                    alu_op = 4'b0010;    
                end
                
            6'b011001:              // I-TYPE -> LHI  ????
                begin
                    reg_dst = 0;
                    alu_src = 1;
                    mem_to_reg = 0;
                    reg_write = 1;
                    mem_write = 0;
                    ext_op = 0;
                    alu_op = 4'b0010; 
                    
                end 
                
            6'b011000:              // I-TYPE -> LOI  ????
                begin
                    reg_dst = 0;
                    alu_src = 1;
                    mem_to_reg = 0;
                    reg_write = 1;
                    mem_write = 0;
                    ext_op = 0;
                    alu_op = 4'b0010; 
                    
                end
                
            6'b100011:              // ---------LW 
                begin
                    reg_dst = 0;
                    alu_src = 1;
                    mem_to_reg = 1;
                    reg_write = 1;
                    mem_write = 0;
                    ext_op = 0;
                    alu_op = 4'b0010;  // ???
                    
                end           
        
            6'b101011:              // ---------SW
                begin
                    reg_dst = 0;
                    alu_src = 1;
                    mem_to_reg = 0;
                    reg_write = 0;
                    mem_write = 1;
                    ext_op = 0;
                    alu_op = 4'b0010;   // ???
                    
                end 
            
            default:
                begin
                    reg_dst = 0;
                    alu_src = 0;
                    mem_to_reg = 0;
                    reg_write = 0;
                    mem_write = 0;
                    ext_op = 0;
                    alu_op = 4'b1111;    
                end 
        
        endcase                      
    end
    
endmodule


module top(clk);
    input clk;
    
    wire [31:0]  pc_out, sum_out, im_out, alu_out, rb1_out, rb2_out, dm_out, mux2_out, mux3_out, ext_sign_out;
    wire [4:0] mux1_out; 
    wire [3:0] alu_op;
    wire reg_write, reg_dst, alu_src, mem_write, mem_to_reg, ext_op, zero;
    
    reg_pc pc_inst(clk, sum_out, pc_out); 
    //reg_pc(clk, in, out);
    
    alu_sum sum_inst(pc_out, sum_out);
    //alu_sum(in, out);
    
    im im_inst(pc_out, im_out); 
    //im(addr, instr);
    
    mux2_1 #(5) mux1(im_out[20:16], im_out[15:11], reg_dst, mux1_out);
    //mux2_1(in1, in2, sel, out);
    
    register_bank rb_inst(clk, im_out[25:21], im_out[20:16], mux1_out, mux3_out, reg_write, rb1_out, rb2_out);
    //register_bank(clk, ra1, ra2, wa, wd, reg_write, rd1, rd2);
    
    ext_sign(im_out[15:0], ext_op, ext_sign_out);
    //ext_sign(in, ext_op, out);
    
    mux2_1 mux2(rb2_out, ext_sign_out, alu_src, mux2_out);
    
    alu(rb1_out, mux2_out, alu_op, alu_out, zero);
    //alu(a, b, alu_op, out, zero);
    
    dm(clk, alu_out, rb2_out, mem_write, dm_out);
    //dm(clk, address, wd, mem_write, rd);
    
    mux2_1 mux3(dm_out, alu_out, mem_to_reg, mux3_out);
    
    main_control(im_out[5:0], im_out[31:26], zero, reg_dst, reg_write, ext_op, alu_src, alu_op, mem_write, mem_to_reg);
    //main_control(func, opcode, zero, reg_dst, reg_write, ext_op, alu_src, alu_op, mem_write, mem_to_reg);
    
    
endmodule


module test;
    reg clk;
    
    top top_inst(clk);
    
    initial
    begin
        #0 clk = 0;
        forever #5 clk = ~clk;
    end
    
    initial 
        #200 $finish;
    
    initial
    begin   
        // Initialize Inputs
        clk = 0;
        //pc_out = 0;

        // Wait for the global reset
        #100;
        
        // Apply different test cases
        $display("Starting test cases...");
        
        // Test case 1: Addi
        // Instruction: addi $t0, $t0, 10
        //pc_out = 0;
        #10;
        $display("Instruction at PC=0: %h", im_out);
        
        // Test case 2: LW
        // Instruction: lw $t1, 0($t0)
        //pc_out = 4;
        #10;
        $display("Instruction at PC=4: %h", im_out);

        // Test case 3: SW
        // Instruction: sw $t1, 4($t0)
       // pc_out = 8;
        #10;
        $display("Instruction at PC=8: %h", im_out);
     
    end

endmodule

