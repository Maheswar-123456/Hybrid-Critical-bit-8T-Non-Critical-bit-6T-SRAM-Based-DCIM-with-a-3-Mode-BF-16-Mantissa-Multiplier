# Hybrid-Critical-bit-8T-Non-Critical-bit-6T-SRAM-Based-DCIM-with-a-3-Mode-BF-16-Mantissa-Multiplier
Hybrid 8T/6T SRAM-based (DCIM) architecture featuring significance-aware BF16 bit storage and a configurable 8×8, 6×6, and 4×4 mantissa multiplier. Implemented in Verilog HDL with RTL functional verification, Cadence Genus synthesis, Innovus physical implementation through routing, and transistor-level SRAM design and simulation using Virtuoso.
DESIGN CODE
`timescale 1ns/1ps

//======================================================================
// File: hybrid_bf16_dcim_all.v
//
// Contains:
// 1. hybrid_sram_macro
// 2. three_mode_mantissa_multiplier
// 3. bf16_compute_unit
// 4. bf16_accumulator
// 5. dcim_controller
// 6. hybrid_bf16_dcim_macro
//
// IMPORTANT:
// The SRAM below is a behavioral RTL model. It does not physically
// instantiate 6T and 8T SRAM bit-cells. The hybrid 6T/8T organization
// must be implemented and characterized at transistor/layout level.
//
// BF16 behavior used here:
// - Normal BF16 numbers are supported.
// - Subnormal inputs are flushed to zero.
// - Mantissa results are truncated, not rounded-to-nearest-even.
// - Infinity and NaN receive basic handling.
//======================================================================


//======================================================================
// 1. BEHAVIORAL HYBRID SRAM MODEL
//======================================================================

module hybrid_sram_macro #(
    parameter ADDR_WIDTH = 3,
    parameter DATA_WIDTH = 16,
    parameter DEPTH      = 8
) (
    input                       clk,
    input                       reset_n,

    input      [ADDR_WIDTH-1:0] address,
    input                       read_enable,
    input                       write_enable,
    input      [DATA_WIDTH-1:0] write_data,

    output reg [DATA_WIDTH-1:0] read_data,
    output reg                  read_valid
);

    reg [DATA_WIDTH-1:0] memory [0:DEPTH-1];
    integer index;

    // Synchronous, read-first behavior for a simultaneous read and write
    // to the same address because nonblocking assignments are used.
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            read_data  <= {DATA_WIDTH{1'b0}};
            read_valid <= 1'b0;

            for (index = 0; index < DEPTH; index = index + 1) begin
                memory[index] <= {DATA_WIDTH{1'b0}};
            end
        end
        else begin
            read_valid <= 1'b0;

            if (write_enable) begin
                memory[address] <= write_data;
            end

            if (read_enable) begin
                read_data  <= memory[address];
                read_valid <= 1'b1;
            end
        end
    end

endmodule


//======================================================================
// 2. THREE-MODE MANTISSA MULTIPLIER
//======================================================================

module three_mode_mantissa_multiplier (
    input      [7:0]  significand_a,
    input      [7:0]  significand_b,
    input      [1:0]  precision_mode,

    output     [15:0] product
);

    reg [7:0] masked_operand_a;
    reg [7:0] masked_operand_b;

    always @(*) begin
        masked_operand_a = significand_a;
        masked_operand_b = significand_b;

        case (precision_mode)
            // Full precision: 8 x 8 significand multiplication.
            2'b00: begin
                masked_operand_a = significand_a;
                masked_operand_b = significand_b;
            end

            // Medium precision: retain the upper six bits.
            2'b01: begin
                masked_operand_a = {significand_a[7:2], 2'b00};
                masked_operand_b = {significand_b[7:2], 2'b00};
            end

            // Low precision: retain the upper four bits.
            2'b10: begin
                masked_operand_a = {significand_a[7:4], 4'b0000};
                masked_operand_b = {significand_b[7:4], 4'b0000};
            end

            // Reserved mode falls back to full precision.
            default: begin
                masked_operand_a = significand_a;
                masked_operand_b = significand_b;
            end
        endcase
    end

    // Shared multiplier. For a stronger power claim, replace this with
    // explicit partial-product generation and precision-mode gating.
    assign product = masked_operand_a * masked_operand_b;

endmodule


//======================================================================
// 3. BF16 COMPUTE UNIT
//======================================================================

module bf16_compute_unit (
    input             clk,
    input             reset_n,
    input             start,

    input      [15:0] operand_a,
    input      [15:0] operand_b,
    input      [1:0]  precision_mode,

    output reg [15:0] result,
    output reg        valid
);

    wire       sign_a;
    wire       sign_b;
    wire       sign_result;

    wire [7:0] exponent_a;
    wire [7:0] exponent_b;
    wire [6:0] fraction_a;
    wire [6:0] fraction_b;

    wire [7:0] significand_a;
    wire [7:0] significand_b;
    wire [15:0] mantissa_product;

    reg [15:0] computed_result;
    reg [7:0]  normalized_exponent;
    reg [6:0]  normalized_fraction;
    reg signed [10:0] exponent_work;

    assign sign_a      = operand_a[15];
    assign exponent_a  = operand_a[14:7];
    assign fraction_a  = operand_a[6:0];

    assign sign_b      = operand_b[15];
    assign exponent_b  = operand_b[14:7];
    assign fraction_b  = operand_b[6:0];

    assign sign_result = sign_a ^ sign_b;

    // Exponent zero is treated as zero; BF16 subnormals are flushed.
    assign significand_a =
        (exponent_a == 8'h00) ? 8'h00 : {1'b1, fraction_a};

    assign significand_b =
        (exponent_b == 8'h00) ? 8'h00 : {1'b1, fraction_b};

    three_mode_mantissa_multiplier u_mantissa_multiplier (
        .significand_a  (significand_a),
        .significand_b  (significand_b),
        .precision_mode (precision_mode),
        .product        (mantissa_product)
    );

    always @(*) begin
        computed_result    = 16'h0000;
        normalized_exponent = 8'h00;
        normalized_fraction = 7'h00;
        exponent_work       = 11'sd0;

        // NaN input: return a canonical quiet NaN.
        if (((exponent_a == 8'hFF) && (fraction_a != 7'h00)) ||
            ((exponent_b == 8'hFF) && (fraction_b != 7'h00))) begin

            computed_result = 16'h7FC0;
        end
        // Infinity multiplied by zero is invalid.
        else if ((((exponent_a == 8'hFF) && (fraction_a == 7'h00)) &&
                  (exponent_b == 8'h00)) ||
                 (((exponent_b == 8'hFF) && (fraction_b == 7'h00)) &&
                  (exponent_a == 8'h00))) begin

            computed_result = 16'h7FC0;
        end
        // Infinity handling.
        else if ((exponent_a == 8'hFF) || (exponent_b == 8'hFF)) begin

            computed_result = {sign_result, 8'hFF, 7'h00};
        end
        // Zero and flushed-subnormal handling.
        else if ((exponent_a == 8'h00) || (exponent_b == 8'h00)) begin

            computed_result = {sign_result, 8'h00, 7'h00};
        end
        else begin
            exponent_work =
                $signed({3'b000, exponent_a}) +
                $signed({3'b000, exponent_b}) -
                11'sd127;

            // Product is in Q2.14 form. If bit 15 is one, the value
            // is at least 2.0 and requires a right normalization.
            if (mantissa_product[15]) begin
                exponent_work       = exponent_work + 11'sd1;
                normalized_fraction = mantissa_product[14:8];
            end
            else begin
                normalized_fraction = mantissa_product[13:7];
            end

            // Overflow becomes infinity.
            if (exponent_work >= 11'sd255) begin
                computed_result = {sign_result, 8'hFF, 7'h00};
            end
            // Underflow is flushed to zero in this implementation.
            else if (exponent_work <= 11'sd0) begin
                computed_result = {sign_result, 8'h00, 7'h00};
            end
            else begin
                normalized_exponent = exponent_work[7:0];
                computed_result = {
                    sign_result,
                    normalized_exponent,
                    normalized_fraction
                };
            end
        end
    end

    // One-cycle registered result. valid is asserted for one clock.
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            result <= 16'h0000;
            valid  <= 1'b0;
        end
        else begin
            valid <= 1'b0;

            if (start) begin
                result <= computed_result;
                valid  <= 1'b1;
            end
        end
    end

endmodule


//======================================================================
// 4. BF16 ACCUMULATOR
//======================================================================

module bf16_accumulator (
    input             clk,
    input             reset_n,

    input             clear,
    input             accumulate_enable,
    input      [15:0] product,

    output reg [15:0] accumulated_result,
    output reg        accumulator_valid
);

    // Combinational BF16 addition used by the running accumulator.
    // Subnormal operands/results are flushed to zero and the final
    // mantissa is truncated. This is synthesizable starter RTL, not a
    // fully IEEE-754-compliant rounded BF16 adder.
    function [15:0] bf16_add;
        input [15:0] value_a;
        input [15:0] value_b;

        reg          sign_a_local;
        reg          sign_b_local;
        reg          result_sign;
        reg [7:0]    exp_a_local;
        reg [7:0]    exp_b_local;
        reg [6:0]    frac_a_local;
        reg [6:0]    frac_b_local;
        reg [10:0]   mant_a_local;
        reg [10:0]   mant_b_local;
        reg [10:0]   mant_large;
        reg [10:0]   mant_small;
        reg [11:0]   mant_work;
        reg [15:0]   add_result;
        reg [8:0]    exp_work_local;
        reg [7:0]    shift_amount;
        integer      normalize_index;

        begin
            sign_a_local = value_a[15];
            sign_b_local = value_b[15];
            exp_a_local  = value_a[14:7];
            exp_b_local  = value_b[14:7];
            frac_a_local = value_a[6:0];
            frac_b_local = value_b[6:0];

            mant_a_local = {1'b1, frac_a_local, 3'b000};
            mant_b_local = {1'b1, frac_b_local, 3'b000};

            mant_large     = 11'd0;
            mant_small     = 11'd0;
            mant_work      = 12'd0;
            result_sign    = 1'b0;
            exp_work_local = 9'd0;
            shift_amount   = 8'd0;
            add_result     = 16'h0000;

            // NaN handling.
            if (((exp_a_local == 8'hFF) && (frac_a_local != 7'h00)) ||
                ((exp_b_local == 8'hFF) && (frac_b_local != 7'h00))) begin

                add_result = 16'h7FC0;
            end
            // Positive infinity plus negative infinity is invalid.
            else if ((exp_a_local == 8'hFF) &&
                     (exp_b_local == 8'hFF) &&
                     (sign_a_local != sign_b_local)) begin

                add_result = 16'h7FC0;
            end
            // Infinity propagation.
            else if (exp_a_local == 8'hFF) begin
                add_result = {sign_a_local, 8'hFF, 7'h00};
            end
            else if (exp_b_local == 8'hFF) begin
                add_result = {sign_b_local, 8'hFF, 7'h00};
            end
            // Exponent-zero operands are treated as zero.
            else if ((exp_a_local == 8'h00) &&
                     (exp_b_local == 8'h00)) begin
                add_result = 16'h0000;
            end
            else if (exp_a_local == 8'h00) begin
                add_result = value_b;
            end
            else if (exp_b_local == 8'h00) begin
                add_result = value_a;
            end
            else begin
                // Select the operand with the larger magnitude and
                // align the smaller significand to its exponent.
                if ((exp_a_local > exp_b_local) ||
                    ((exp_a_local == exp_b_local) &&
                     (mant_a_local >= mant_b_local))) begin

                    result_sign    = sign_a_local;
                    exp_work_local = {1'b0, exp_a_local};
                    mant_large     = mant_a_local;
                    shift_amount   = exp_a_local - exp_b_local;

                    if (shift_amount >= 8'd11)
                        mant_small = 11'd0;
                    else
                        mant_small = mant_b_local >> shift_amount;
                end
                else begin
                    result_sign    = sign_b_local;
                    exp_work_local = {1'b0, exp_b_local};
                    mant_large     = mant_b_local;
                    shift_amount   = exp_b_local - exp_a_local;

                    if (shift_amount >= 8'd11)
                        mant_small = 11'd0;
                    else
                        mant_small = mant_a_local >> shift_amount;
                end

                if (sign_a_local == sign_b_local) begin
                    mant_work = {1'b0, mant_large} +
                                {1'b0, mant_small};

                    // Addition overflow: shift right once.
                    if (mant_work[11]) begin
                        mant_work      = mant_work >> 1;
                        exp_work_local = exp_work_local + 9'd1;
                    end
                end
                else begin
                    mant_work = {1'b0, mant_large} -
                                {1'b0, mant_small};

                    // Normalize subtraction result to put the hidden
                    // one at bit 10.
                    for (normalize_index = 0;
                         normalize_index < 10;
                         normalize_index = normalize_index + 1) begin

                        if ((mant_work[10] == 1'b0) &&
                            (mant_work != 12'd0) &&
                            (exp_work_local > 9'd1)) begin

                            mant_work      = mant_work << 1;
                            exp_work_local = exp_work_local - 9'd1;
                        end
                    end
                end

                if (mant_work == 12'd0) begin
                    add_result = 16'h0000;
                end
                // A nonzero result without a normalized hidden bit at
                // the minimum exponent would be subnormal; flush it.
                else if ((exp_work_local <= 9'd1) &&
                         (mant_work[10] == 1'b0)) begin
                    add_result = {result_sign, 8'h00, 7'h00};
                end
                else if (exp_work_local >= 9'd255) begin
                    add_result = {result_sign, 8'hFF, 7'h00};
                end
                else if (exp_work_local == 9'd0) begin
                    add_result = {result_sign, 8'h00, 7'h00};
                end
                else begin
                    add_result = {
                        result_sign,
                        exp_work_local[7:0],
                        mant_work[9:3]
                    };
                end
            end

            bf16_add = add_result;
        end
    endfunction

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            accumulated_result <= 16'h0000;
            accumulator_valid  <= 1'b0;
        end
        else begin
            accumulator_valid <= 1'b0;

            if (clear) begin
                accumulated_result <= 16'h0000;
                accumulator_valid  <= 1'b1;
            end
            else if (accumulate_enable) begin
                accumulated_result <=
                    bf16_add(accumulated_result, product);
                accumulator_valid <= 1'b1;
            end
        end
    end

endmodule


//======================================================================
// 5. DCIM CONTROLLER
//======================================================================

module dcim_controller (
    input  clk,
    input  reset_n,

    input  write_request,
    input  read_request,
    input  compute_request,
    input  accumulate_enable,
    input  accumulator_clear_request,

    input  memory_data_valid,
    input  product_valid,

    output reg memory_read_enable,
    output reg memory_write_enable,
    output reg compute_start,

    output reg accumulator_clear,
    output reg accumulator_start,

    output reg read_valid,
    output reg busy
);

    localparam STATE_IDLE          = 3'd0;
    localparam STATE_WRITE         = 3'd1;
    localparam STATE_ISSUE_READ    = 3'd2;
    localparam STATE_WAIT_MEMORY   = 3'd3;
    localparam STATE_ISSUE_COMPUTE = 3'd4;
    localparam STATE_WAIT_PRODUCT  = 3'd5;
    localparam STATE_ACCUMULATE    = 3'd6;
    localparam STATE_CLEAR_ACCUM   = 3'd7;

    reg [2:0] current_state;
    reg [2:0] next_state;

    reg compute_pending;
    reg accumulation_pending;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            current_state        <= STATE_IDLE;
            compute_pending      <= 1'b0;
            accumulation_pending <= 1'b0;
        end
        else begin
            current_state <= next_state;

            // Capture only the request selected by the IDLE priority.
            if (current_state == STATE_IDLE) begin
                if (write_request) begin
                    compute_pending      <= 1'b0;
                    accumulation_pending <= 1'b0;
                end
                else if (accumulator_clear_request) begin
                    compute_pending      <= 1'b0;
                    accumulation_pending <= 1'b0;
                end
                else if (compute_request) begin
                    compute_pending      <= 1'b1;
                    accumulation_pending <= accumulate_enable;
                end
                else if (read_request) begin
                    compute_pending      <= 1'b0;
                    accumulation_pending <= 1'b0;
                end
            end

            if ((current_state == STATE_WAIT_PRODUCT) &&
                product_valid) begin
                compute_pending <= 1'b0;

                if (!accumulation_pending)
                    accumulation_pending <= 1'b0;
            end

            if (current_state == STATE_ACCUMULATE) begin
                accumulation_pending <= 1'b0;
            end
        end
    end

    always @(*) begin
        memory_read_enable  = 1'b0;
        memory_write_enable = 1'b0;
        compute_start       = 1'b0;
        accumulator_clear   = 1'b0;
        accumulator_start   = 1'b0;
        read_valid          = 1'b0;

        busy       = (current_state != STATE_IDLE);
        next_state = current_state;

        case (current_state)
            STATE_IDLE: begin
                if (write_request)
                    next_state = STATE_WRITE;
                else if (accumulator_clear_request)
                    next_state = STATE_CLEAR_ACCUM;
                else if (read_request || compute_request)
                    next_state = STATE_ISSUE_READ;
            end

            STATE_WRITE: begin
                memory_write_enable = 1'b1;
                next_state = STATE_IDLE;
            end

            STATE_ISSUE_READ: begin
                memory_read_enable = 1'b1;
                next_state = STATE_WAIT_MEMORY;
            end

            STATE_WAIT_MEMORY: begin
                if (memory_data_valid) begin
                    if (compute_pending)
                        next_state = STATE_ISSUE_COMPUTE;
                    else begin
                        read_valid = 1'b1;
                        next_state = STATE_IDLE;
                    end
                end
            end

            STATE_ISSUE_COMPUTE: begin
                compute_start = 1'b1;
                next_state = STATE_WAIT_PRODUCT;
            end

            STATE_WAIT_PRODUCT: begin
                if (product_valid) begin
                    if (accumulation_pending)
                        next_state = STATE_ACCUMULATE;
                    else
                        next_state = STATE_IDLE;
                end
            end

            STATE_ACCUMULATE: begin
                accumulator_start = 1'b1;
                next_state = STATE_IDLE;
            end

            STATE_CLEAR_ACCUM: begin
                accumulator_clear = 1'b1;
                next_state = STATE_IDLE;
            end

            default: begin
                next_state = STATE_IDLE;
            end
        endcase
    end

endmodule


//======================================================================
// 6. TOP-LEVEL HYBRID BF16 DCIM MACRO
//======================================================================

module hybrid_bf16_dcim_macro #(
    parameter ADDR_WIDTH = 3,
    parameter DEPTH      = 8
) (
    input                       clk,
    input                       reset_n,

    // Command requests. Assert a request when busy is low.
    input                       write_request,
    input                       read_request,
    input                       compute_request,
    input                       accumulate_enable,
    input                       accumulator_clear_request,

    // Memory command information.
    input      [ADDR_WIDTH-1:0] address,
    input      [15:0]           write_data,

    // The memory word is operand A; this is operand B.
    input      [15:0]           external_operand_b,
    input      [1:0]            precision_mode,

    // Read response.
    output     [15:0]           read_data,
    output                      read_valid,

    // Multiply response.
    output     [15:0]           compute_result,
    output                      compute_valid,

    // Accumulator response/state.
    output     [15:0]           accumulated_result,
    output                      accumulator_valid,

    // Controller status.
    output                      busy
);

    // Latched command fields prevent address/data changes at the top-level
    // pins from corrupting a command after it has been accepted.
    reg [ADDR_WIDTH-1:0] command_address;
    reg [15:0]           command_write_data;
    reg [15:0]           command_operand_b;
    reg [1:0]            command_precision_mode;

    wire [15:0] memory_read_data;
    wire        memory_data_valid;

    wire [15:0] product_result;
    wire        product_valid;

    wire memory_read_enable;
    wire memory_write_enable;
    wire compute_start;
    wire accumulator_clear;
    wire accumulator_start;
    wire controller_read_valid;

    // Capture the command selected by the same priority used in the
    // controller: write, clear, compute/read.
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            command_address        <= {ADDR_WIDTH{1'b0}};
            command_write_data     <= 16'h0000;
            command_operand_b      <= 16'h0000;
            command_precision_mode <= 2'b00;
        end
        else if (!busy) begin
            if (write_request) begin
                command_address    <= address;
                command_write_data <= write_data;
            end
            else if (read_request || compute_request) begin
                command_address        <= address;
                command_operand_b      <= external_operand_b;
                command_precision_mode <= precision_mode;
            end
        end
    end

    hybrid_sram_macro #(
        .ADDR_WIDTH (ADDR_WIDTH),
        .DATA_WIDTH (16),
        .DEPTH      (DEPTH)
    ) u_hybrid_sram_macro (
        .clk          (clk),
        .reset_n      (reset_n),
        .address      (command_address),
        .read_enable  (memory_read_enable),
        .write_enable (memory_write_enable),
        .write_data   (command_write_data),
        .read_data    (memory_read_data),
        .read_valid   (memory_data_valid)
    );

    bf16_compute_unit u_bf16_compute_unit (
        .clk            (clk),
        .reset_n        (reset_n),
        .start          (compute_start),
        .operand_a      (memory_read_data),
        .operand_b      (command_operand_b),
        .precision_mode (command_precision_mode),
        .result         (product_result),
        .valid          (product_valid)
    );

    bf16_accumulator u_bf16_accumulator (
        .clk                (clk),
        .reset_n            (reset_n),
        .clear              (accumulator_clear),
        .accumulate_enable  (accumulator_start),
        .product            (product_result),
        .accumulated_result (accumulated_result),
        .accumulator_valid  (accumulator_valid)
    );

    dcim_controller u_dcim_controller (
        .clk                       (clk),
        .reset_n                   (reset_n),
        .write_request             (write_request),
        .read_request              (read_request),
        .compute_request           (compute_request),
        .accumulate_enable         (accumulate_enable),
        .accumulator_clear_request (accumulator_clear_request),
        .memory_data_valid         (memory_data_valid),
        .product_valid             (product_valid),
        .memory_read_enable        (memory_read_enable),
        .memory_write_enable       (memory_write_enable),
        .compute_start             (compute_start),
        .accumulator_clear         (accumulator_clear),
        .accumulator_start         (accumulator_start),
        .read_valid                (controller_read_valid),
        .busy                      (busy)
    );

    assign read_data      = memory_read_data;
    assign read_valid     = controller_read_valid;
    assign compute_result = product_result;
    assign compute_valid  = product_valid;

endmodule
