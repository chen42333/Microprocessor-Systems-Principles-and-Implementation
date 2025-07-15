`timescale 1ns / 1ps

module profiler#(
    parameter HART_ID       = 0,
    parameter XLEN          = 32
)
(
    input clk_i,
    input [XLEN-1:0] instr_i,
    input [XLEN-1:0] pc_addr_i,
    input stall_i
    );

reg [XLEN-1:0] START [0:4];
reg [XLEN-1:0] END [0:4];
(* dont_touch = "true", mark_debug = "true" *) reg [XLEN-1:0] ls [0:4];
(* dont_touch = "true", mark_debug = "true" *) reg [XLEN-1:0] alu [0:4];
(* dont_touch = "true", mark_debug = "true" *) reg [XLEN-1:0] stall [0:4];
(* dont_touch = "true", mark_debug = "true" *) reg [XLEN-1:0] total_stall;
(* dont_touch = "true", mark_debug = "true" *) reg [XLEN-1:0] count;
integer i;
initial begin
    START[0] = 'h1d80; END[0] = 'h1da4; //core_list_reverse
    START[1] = 'h1d28; END[1] = 'h1d80; //core_list_find
    START[2] = 'h2670; END[2] = 'h2730; //matrix_mul_matrix_bitextract
    START[3] = 'h2a14; END[3] = 'h2d10; //core_state_transition
    START[4] = 'h19e8; END[4] = 'h1a2c; //crcu8
    count = 0; total_stall = 0;
    for (i = 0; i < 5; i = i + 1) begin
        ls[i] = 0; alu[i] = 0; stall[i] = 0;
    end
end

always @(posedge clk_i) begin
    if (pc_addr_i >= 'h1000 && pc_addr_i < 'h6d44) begin 
        count = count + 1;
        if (stall_i) total_stall <= total_stall + 1;
    end
    for(i = 0; i < 5; i = i + 1) begin
        if (pc_addr_i >= START[i] && pc_addr_i < END[i]) begin
            if(stall_i) stall[i] <= stall[i] + 1;
            else if(instr_i[6:0] == 7'b0000011 || instr_i[6:0] == 7'b0100011 || instr_i[6:0] == 7'b0110111) ls[i] <= ls[i] + 1;
            else alu[i] <= alu[i] + 1;
        end
    end
end


endmodule
