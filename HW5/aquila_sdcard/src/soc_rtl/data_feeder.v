`timescale 1ns / 1ps

module data_feeder
#(parameter XLEN = 32, BRAM_SIZE = 1024)
(
    input   clk_i, rst_i,
    (* dont_touch = "true", mark_debug = "true" *) input   strobe_i,
    (* dont_touch = "true", mark_debug = "true" *) input [XLEN-1 : 0]   addr_i,
    (* dont_touch = "true", mark_debug = "true" *) input   rw_i, // 0 for read and 1 for write
    (* dont_touch = "true", mark_debug = "true" *) input [XLEN-1 : 0]   data_i,
    
    (* dont_touch = "true", mark_debug = "true" *) output  data_ready_o,
    (* dont_touch = "true", mark_debug = "true" *) output reg [XLEN-1 : 0]  data_o
);
/*
BRAM_A: store vector A (0xC400_0000 - 0xC400_0FFF)
BRAM_B1, BRAM_B2: store vector B (0xC400_1000 - 0xC400_1FFF)
Read result (0xC400_2000)
Write total number of neurons (0xC400_3000), and add 1 at the tail of vector a (for bias)
*/
parameter BASE_A = 'hC400_0000, BASE_B = 'hC400_1000;
(* dont_touch = "true", mark_debug = "true" *) wire bram_a_write, bram_b1_write, bram_b2_write;
(* dont_touch = "true", mark_debug = "true" *) reg ping_pong; // 0: compute with B2 and store in B1, 1: compute with B1 and store in B2
(* dont_touch = "true", mark_debug = "true" *) reg complete, ready; // the computation of the current neuron has nded / the result is prepared
(* dont_touch = "true", mark_debug = "true" *) reg compute; // the data is prepared and the IP can do computation
(* dont_touch = "true", mark_debug = "true" *) reg [XLEN-1 : 0] vec_size; // without bias
(* dont_touch = "true", mark_debug = "true" *) reg [XLEN-1 : 0] vec_ele_idx;
(* dont_touch = "true", mark_debug = "true" *) wire [XLEN-1 : 0] vec_ele_idx_now;
(* dont_touch = "true", mark_debug = "true" *) wire [XLEN-1 : 0] vec_a;
(* dont_touch = "true", mark_debug = "true" *) wire [XLEN-1 : 0] vec_b;
(* dont_touch = "true", mark_debug = "true" *) wire [XLEN-1 : 0] vec_b1;
(* dont_touch = "true", mark_debug = "true" *) wire [XLEN-1 : 0] vec_b2;
(* dont_touch = "true", mark_debug = "true" *) wire [XLEN-1 : 0] addr_a;
(* dont_touch = "true", mark_debug = "true" *) wire [XLEN-1 : 0] data_a_i;
(* dont_touch = "true", mark_debug = "true" *) wire [XLEN-1 : 0] result;
(* dont_touch = "true", mark_debug = "true" *) wire result_valid;


initial begin
    ping_pong <= 0; vec_ele_idx <= 0; vec_size <= 0; data_o <= 0; compute <= 0; ready <= 0; complete <= 0;
end

assign data_ready_o = rw_i ? 1 : ready;

assign bram_a_write = strobe_i && ((addr_i[13:12] == 2'b00) || (addr_i[13:12] == 2'b11));
assign bram_b1_write = strobe_i && (addr_i[13:12] == 2'b01) && ~ping_pong;
assign bram_b2_write = strobe_i && (addr_i[13:12] == 2'b01) && ping_pong;
assign vec_ele_idx_now = (strobe_i) ? ((vec_ele_idx == vec_size) ? 0 : (vec_ele_idx + 1)) : vec_ele_idx; // Let the read of vector A won't be delayed for 1 cycle (don't neet to wait vec_ele_idx)
assign addr_a = bram_a_write ? ((addr_i[13:12] == 2'b11) ? data_i : (addr_i - BASE_A) / (XLEN/8)) : (vec_ele_idx_now > 0) ? (vec_ele_idx_now - 1) : vec_size;
assign data_a_i = (addr_i[13:12] == 2'b11) ? 'h3f800000 : data_i; // write 1.0 as the last element of vector A to compute bias
assign vec_b = ping_pong ? vec_b1 : vec_b2;

generate
    sram #(.DATA_WIDTH(XLEN), .N_ENTRIES(BRAM_SIZE))
         BRAM_A(
             .clk_i(clk_i),
             .en_i(1'b1),
             .we_i(bram_a_write),
             .addr_i(addr_a[$clog2(BRAM_SIZE)-1 : 0]),
             .data_i(data_a_i),
             .data_o(vec_a)
         );
    sram #(.DATA_WIDTH(XLEN), .N_ENTRIES(4))
         BRAM_B1(
             .clk_i(clk_i),
             .en_i(1'b1),
             .we_i(bram_b1_write),
             .addr_i(0),
             .data_i(data_i),
             .data_o(vec_b1)
         );
    sram #(.DATA_WIDTH(XLEN), .N_ENTRIES(4))
         BRAM_B2(
             .clk_i(clk_i),
             .en_i(1'b1),
             .we_i(bram_b2_write),
             .addr_i(0),
             .data_i(data_i),
             .data_o(vec_b2)
         );
endgenerate

floating_point_0 FP_Operator(
    .aclk(clk_i),
    .aresetn(~rst_i),

    .s_axis_a_tvalid(1),
    .s_axis_a_tdata(vec_a),

    .s_axis_b_tvalid(1),
    .s_axis_b_tdata(vec_b),

    .s_axis_c_tvalid(1),
    .s_axis_c_tdata(data_o),

    .s_axis_operation_tvalid(compute),
    .s_axis_operation_tdata(6'b000000), // perform add

    .m_axis_result_tvalid(result_valid),
    .m_axis_result_tdata(result)
);

always @(posedge clk_i) begin
    if (rst_i) begin
        ping_pong <= 0; vec_ele_idx <= 0; vec_size <= 0; data_o <= 0; compute <= 0; ready <= 0; complete <= 0;
    end
    else begin
        if (strobe_i) begin
            if (addr_i == 'hC4003000) vec_size <= data_i;
            else if (addr_i == 'hC4001000) begin
                ping_pong <= ~ping_pong;
                compute <= 1;
                if (vec_ele_idx == 0) data_o <= 0;
                else data_o <= result;
                if (vec_ele_idx == vec_size) begin
                    vec_ele_idx <= 0;
                    complete <= 1;
                end
                else vec_ele_idx <= vec_ele_idx + 1;
                if (ready) ready <= 0;
            end
        end
        if (complete && result_valid) begin // the computation of the current neuron is complete
            ready <= 1;
            complete <= 0;
            data_o <= result;
        end
        if (compute) compute <= 0;
    end
end

endmodule
