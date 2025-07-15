`timescale 1ns / 1ps
// =============================================================================
//  Program : bpu.v
//  Author  : Jin-you Wu
//  Date    : Jan/19/2019
// -----------------------------------------------------------------------------
//  Description:
//  This is the Branch Prediction Unit (BPU) of the Aquila core (A RISC-V core).
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Aug/15/2020, by Chun-Jen Tsai:
//    Hanlding of JAL in this BPU. In the original code, an additional
//    Unconditional Branch Prediction Unit (UC-BPU) was used to handle
//    the JAL instruction, which seemed redundant.
//
// Aug/16/2023, by Chun-Jen Tsai:
//    Replace the fully associative BHT by the standard Bimodal BHT table.
//    The performance drops a little (1.0 DMIPS -> 0.97 DMIPS), but the resource 
//    usage drops significantly.
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2019,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Chiao Tung Uniersity
//                    Hsinchu, Taiwan.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// =============================================================================
`include "aquila_config.vh"

module bpu #( parameter ENTRY_NUM = 256, parameter XLEN = 32 )
(
    // System signals
    input               clk_i,
    input               rst_i,
    input               stall_i,

    // from Program_Counter
    input  [XLEN-1 : 0] pc_i, // Addr of the next instruction to be fetched.

    // from Decode
    input               is_jal_i,
    input               is_cond_branch_i,
    input  [XLEN-1 : 0] dec_pc_i, // Addr of the instr. just processed by decoder.

    // from Execute
    input               exe_is_branch_i,
    input               branch_taken_i,
    input               branch_misprediction_i,
    input  [XLEN-1 : 0] branch_target_addr_i,

    // to Program_Counter
    output              branch_hit_o,
    output              branch_decision_o,
    output [XLEN-1 : 0] branch_target_addr_o,
    
    //BPU statistics
    input [XLEN-1 : 0] instr_i
);

localparam NBITS = $clog2(ENTRY_NUM);

wire                    we;
wire [XLEN-1 : 0]       branch_inst_tag;
wire [NBITS-1 : 0]      read_addr;
wire [NBITS-1 : 0]      write_addr;
// two-bit saturating counter
parameter ENTRY_NUM_COUNTER = 2048;//ENTRY_NUM;
localparam NBITS_counter = $clog2(ENTRY_NUM_COUNTER);
wire [NBITS_counter-1 : 0]      write_addr_counter;
wire [NBITS_counter-1 : 0]      read_addr_counter;
reg  [1 : 0]            branch_likelihood[ENTRY_NUM_COUNTER-1 : 0];

// "we" is enabled to add a new entry to the BHT table when
// the decoder sees a branch instruction for the first time.
// CY Hsiang 0220_2020: added "~stall_i" to "we ="
//for BHT 
assign we = ~stall_i & (is_cond_branch_i | is_jal_i) & (branch_inst_tag != dec_pc_i); //bug, always TRUE (branch_inst_tag != dec_pc_i)
assign read_addr = pc_i[NBITS+1 : 2];
assign write_addr = dec_pc_i[NBITS+1 : 2];

integer idx;

always @(posedge clk_i)
begin
    if (rst_i)
    begin
        for (idx = 0; idx < ENTRY_NUM_COUNTER; idx = idx + 1)
            branch_likelihood[idx] <= 2'b0;
    end
    else if (stall_i)
    begin
        for (idx = 0; idx < ENTRY_NUM_COUNTER; idx = idx + 1)
            branch_likelihood[idx] <= branch_likelihood[idx];
    end
    else
    begin
        if (we) // Execute the branch instruction for the first time.
        begin
            branch_likelihood[write_addr_counter] <= {branch_taken_i, branch_taken_i};
        end
        else if (exe_is_branch_i)
        begin
            case (branch_likelihood[write_addr_counter])
                2'b00:  // strongly not taken
                    if (branch_taken_i)
                        branch_likelihood[write_addr_counter] <= 2'b01;
                    else
                        branch_likelihood[write_addr_counter] <= 2'b00;
                2'b01:  // weakly not taken
                    if (branch_taken_i)
                        branch_likelihood[write_addr_counter] <= 2'b11;
                    else
                        branch_likelihood[write_addr_counter] <= 2'b00;
                2'b10:  // weakly taken
                    if (branch_taken_i)
                        branch_likelihood[write_addr_counter] <= 2'b11;
                    else
                        branch_likelihood[write_addr_counter] <= 2'b00;
                2'b11:  // strongly taken
                    if (branch_taken_i)
                        branch_likelihood[write_addr_counter] <= 2'b11;
                    else
                        branch_likelihood[write_addr_counter] <= 2'b10;
            endcase
        end
    end
end
// ===========================================================================
//Local and global predictors
parameter GLOBAL_TRACK = 32, LOCAL_TRACK = 32, ENTRY_NUM_LOCAL = 256;
localparam NBITS_LOCAL = $clog2(ENTRY_NUM_LOCAL);
reg [GLOBAL_TRACK-1 : 0] GR;
reg [LOCAL_TRACK-1 : 0] history[ENTRY_NUM_LOCAL-1 : 0];
integer i;
initial begin
    for (i = 0; i < GLOBAL_TRACK; i = i + 1)  GR[i] = 0;
    for (i = 0; i < LOCAL_TRACK; i = i + 1)  history[i] = 0;
end

if(`BPU_TYPE == 1) begin //Global and PC
    parameter GLOBAL_BITS = 6; //Less than NBITS_counter
    parameter PC_BITS = NBITS_counter - GLOBAL_BITS;
    assign read_addr_counter = {GR[GLOBAL_BITS-1:0], pc_i[PC_BITS+1:2]};
    assign write_addr_counter = {GR[GLOBAL_BITS-1:0], pc_i[PC_BITS+1:2]};
end
else if(`BPU_TYPE == 2) begin //Glocal and local
    parameter GLOBAL_BITS = 1; //Less than NBITS_counter
    parameter LOCAL_BITS = NBITS_counter - GLOBAL_BITS;
    wire [NBITS_LOCAL-1 : 0] index_read, index_write;
    assign index_read = pc_i[NBITS_LOCAL+2 : 2];
    assign read_addr_counter = {GR[GLOBAL_BITS-1:0], history[index_read][LOCAL_BITS-1:0]};
    assign index_write = dec_pc_i[NBITS_LOCAL+2 : 2];
    assign write_addr_counter = {GR[GLOBAL_BITS-1:0], history[index_write][LOCAL_BITS-1:0]};
end
else if(`BPU_TYPE == 3) begin //Gshare
    assign read_addr_counter = GR[NBITS_counter-1:0] ^ pc_i[NBITS_counter-1:0];
    assign write_addr_counter = GR[NBITS_counter-1:0] ^ dec_pc_i[NBITS_counter-1:0];
end
else begin //Default bimodel preditor
    assign write_addr_counter = write_addr;
    assign read_addr_counter = read_addr;
end

reg [NBITS_LOCAL-1 : 0] local_index;
always @(posedge clk_i)
begin
    if (rst_i) begin
        for (i = 0; i < GLOBAL_TRACK; i = i + 1)  GR[i] = 0;
        for (i = 0; i < LOCAL_TRACK; i = i + 1)  history[i] = 0;
    end
    else if (stall_i)
    begin
        for (i = 0; i < GLOBAL_TRACK; i = i + 1)  GR[i] <= GR[i];
        for (i = 0; i < LOCAL_TRACK; i = i + 1)  history[i] <= history[i];
    end
    else
    begin
        if (exe_is_branch_i) begin
            if(`BPU_TYPE == 1 || `BPU_TYPE ==2 || `BPU_TYPE == 3) begin
                GR[GLOBAL_TRACK-1:1] = GR[GLOBAL_TRACK-2:0];
                GR[0] = branch_taken_i;
            end
            if(`BPU_TYPE == 2) begin
                local_index = dec_pc_i[NBITS_LOCAL+2 : 2];
                history[local_index][LOCAL_TRACK-1:1] = history[local_index][LOCAL_TRACK-2:0];
                history[local_index][0] = branch_taken_i;
            end
        end
    end
end
// ===========================================================================
//BPU staticstics
parameter BEQ_funct3 = 3'b000, BNE_funct3 = 3'b001, BLT_funct3 = 3'b100, BGE_funct3 = 3'b101, BLTU_funct3 = 3'b110, BGEU_funct3 = 3'b111;
(* dont_touch = "true", mark_debug = "true" *) reg [XLEN-1:0] BPU_taken_count [0:5];
(* dont_touch = "true", mark_debug = "true" *) reg [XLEN-1:0] BPU_ntaken_count [0:5];
(* dont_touch = "true", mark_debug = "true" *) reg [XLEN-1:0] BPU_misprediction_count [0:5];
(* dont_touch = "true", mark_debug = "true" *) reg [XLEN-1:0] BPU_correct_count [0:5];
initial begin
    for (i = 0; i < 6; i = i + 1) begin
        BPU_taken_count[i] = 0; BPU_ntaken_count [i] = 0;
        BPU_misprediction_count[i] = 0;
    end
end

always @(posedge clk_i) begin
    if (rst_i) begin
        for (i = 0; i < 6; i = i + 1) begin
            BPU_taken_count[i] = 0; BPU_ntaken_count[i] = 0;
            BPU_misprediction_count[i] = 0; BPU_correct_count[i] = 0;
        end
    end
    else if (stall_i) begin
        for (i = 0; i < 6; i = i + 1) begin
            BPU_taken_count[i] <= BPU_taken_count[i]; BPU_ntaken_count[i] <= BPU_ntaken_count[i];
            BPU_misprediction_count[i] <= BPU_misprediction_count[i];
        end
    end
    else
    begin
        if(is_cond_branch_i && pc_i >= 'h1000 && pc_i < 'h6d44) begin
        case(instr_i[14:12])
                BEQ_funct3: begin
                    if (branch_taken_i) BPU_taken_count[0] <= BPU_taken_count[0] + 1;
                    else BPU_ntaken_count[0] <= BPU_ntaken_count[0] + 1;
                    if (branch_misprediction_i) BPU_misprediction_count[0] <= BPU_misprediction_count[0] + 1;
                    else BPU_correct_count[0] <= BPU_correct_count[0] + 1;
                end
                BNE_funct3: begin
                    if (branch_taken_i) BPU_taken_count[1] <= BPU_taken_count[1] + 1;
                    else BPU_ntaken_count[1] <= BPU_ntaken_count[1] + 1;
                    if (branch_misprediction_i) BPU_misprediction_count[1] <= BPU_misprediction_count[1] + 1;
                    else BPU_correct_count[1] <= BPU_correct_count[1] + 1;
                end
                BLT_funct3: begin
                    if (branch_taken_i) BPU_taken_count[2] <= BPU_taken_count[2] + 1;
                    else BPU_ntaken_count[2] <= BPU_ntaken_count[2] + 1;
                    if (branch_misprediction_i) BPU_misprediction_count[2] <= BPU_misprediction_count[2] + 1;
                    else BPU_correct_count[2] <= BPU_correct_count[2] + 1;
                    
                end
                BGE_funct3: begin
                    if (branch_taken_i) BPU_taken_count[3] <= BPU_taken_count[3] + 1;
                    else BPU_ntaken_count[3] <= BPU_ntaken_count[3] + 1;
                    if (branch_misprediction_i) BPU_misprediction_count[3] <= BPU_misprediction_count[3] + 1;
                    else BPU_correct_count[3] <= BPU_correct_count[3] + 1;
                end
                BLTU_funct3: begin
                    if (branch_taken_i) BPU_taken_count[4] <= BPU_taken_count[4] + 1;
                    else BPU_ntaken_count[4] <= BPU_ntaken_count[4] + 1;
                    if (branch_misprediction_i) BPU_misprediction_count[4] <= BPU_misprediction_count[4] + 1;
                    else BPU_correct_count[4] <= BPU_correct_count[4] + 1;
                end
                BGEU_funct3: begin
                    if (branch_taken_i) BPU_taken_count[5] <= BPU_taken_count[5] + 1;
                    else BPU_ntaken_count[5] <= BPU_ntaken_count[5] + 1;
                    if (branch_misprediction_i) BPU_misprediction_count[5] <= BPU_misprediction_count[5] + 1;
                    else BPU_correct_count[5] <= BPU_correct_count[5] + 1;
                end
            endcase
        end
    end
end
// ===========================================================================
//  Branch History Table (BHT). Here, we use a direct-mapping cache table to
//  store branch history. Each entry of the table contains two fields:
//  the branch_target_addr and the PC of the branch instruction (as the tag).
//
distri_ram #(.ENTRY_NUM(ENTRY_NUM), .XLEN(XLEN*2))
BPU_BHT(
    .clk_i(clk_i),
    .we_i(we),                  // Write-enabled when the instruction at the Decode
                                //   is a branch and has never been executed before.
    .write_addr_i(write_addr),  // Direct-mapping index for the branch at Decode.
    .read_addr_i(read_addr),    // Direct-mapping Index for the next PC to be fetched.

    .data_i({branch_target_addr_i, dec_pc_i}), // Input is not used when 'we' is 0.
    .data_o({branch_target_addr_o, branch_inst_tag})
);

// ===========================================================================
//  Outputs signals
//
assign branch_hit_o = (branch_inst_tag == pc_i);
assign branch_decision_o = branch_likelihood[read_addr_counter][1];

endmodule
