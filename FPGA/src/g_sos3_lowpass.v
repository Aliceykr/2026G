/*
 * G 题 5 MSPS 低通：6 阶椭圆滤波器，3 个 SOS。
 *
 * 设计指标：
 * - Fs = 5 MHz
 * - 通带 0..500 kHz，Rp = 0.1 dB
 * - 750 kHz 起进入 60 dB 阻带
 *
 * 系数为有符号 18 bit Q2.16。采用转置直接 II 型：
 *   y  = b0*x + d1
 *   d1 = b1*x - a1*y + d2
 *   d2 = b2*x - a2*y
 *
 * 每个输入样本依次处理 3 个 SOS，每段分为前向和反馈两拍，
 * 接收样本后共占 6 个 50 MHz 处理周期。
 */
module g_sos3_lowpass(
    input                       clk,
    input                       rst_n,
    input      signed [17:0]    sample_in,
    input                       sample_valid,
    output reg signed [17:0]    sample_out,
    output reg                  output_valid
);

localparam [2:0] STATE_IDLE = 3'd0;
localparam [2:0] STATE_S0_A = 3'd1;
localparam [2:0] STATE_S0_B = 3'd2;
localparam [2:0] STATE_S1_A = 3'd3;
localparam [2:0] STATE_S1_B = 3'd4;
localparam [2:0] STATE_S2_A = 3'd5;
localparam [2:0] STATE_S2_B = 3'd6;

reg [2:0] state;
reg signed [17:0] x_register;
reg signed [17:0] y_register;
reg signed [39:0] b1x_register;
reg signed [39:0] b2x_register;

reg signed [39:0] d1_0;
reg signed [39:0] d2_0;
reg signed [39:0] d1_1;
reg signed [39:0] d2_1;
reg signed [39:0] d1_2;
reg signed [39:0] d2_2;

/*
 * 当前段的系数和状态在进入 A 拍前锁存，避免 state 译码和多路选择器
 * 落在“乘法 + 累加 + 舍入/饱和”的关键路径上。
 */
reg signed [17:0] b0_register;
reg signed [17:0] b1_register;
reg signed [17:0] b2_register;
reg signed [17:0] a1_register;
reg signed [17:0] a2_register;
reg signed [39:0] d1_register;
reg signed [39:0] d2_register;

function signed [17:0] coeff_b0;
    input [1:0] section;
    begin
        case (section)
            2'd0: coeff_b0 = 18'sd238;
            default: coeff_b0 = 18'sd65536;
        endcase
    end
endfunction

function signed [17:0] coeff_b1;
    input [1:0] section;
    begin
        case (section)
            2'd0: coeff_b1 = 18'sd236;
            2'd1: coeff_b1 = -18'sd50147;
            default: coeff_b1 = -18'sd75832;
        endcase
    end
endfunction

function signed [17:0] coeff_b2;
    input [1:0] section;
    begin
        case (section)
            2'd0: coeff_b2 = 18'sd238;
            default: coeff_b2 = 18'sd65536;
        endcase
    end
endfunction

function signed [17:0] coeff_a1;
    input [1:0] section;
    begin
        case (section)
            2'd0: coeff_a1 = -18'sd93425;
            2'd1: coeff_a1 = -18'sd94985;
            default: coeff_a1 = -18'sd99008;
        endcase
    end
endfunction

function signed [17:0] coeff_a2;
    input [1:0] section;
    begin
        case (section)
            2'd0: coeff_a2 = 18'sd35088;
            2'd1: coeff_a2 = 18'sd46648;
            default: coeff_a2 = 18'sd59505;
        endcase
    end
endfunction

function signed [39:0] round_q16_to_integer;
    input signed [39:0] value;
    reg signed [40:0] magnitude;
    reg signed [40:0] rounded;
    begin
        magnitude = 41'sd0;
        rounded = 41'sd0;
        if (value < 0) begin
            magnitude = -{{1{value[39]}}, value};
            rounded = -((magnitude + 41'sd32768) >>> 16);
        end
        else begin
            rounded = ({{1{value[39]}}, value} + 41'sd32768) >>> 16;
        end
        round_q16_to_integer = rounded[39:0];
    end
endfunction

function signed [17:0] saturate_18;
    input signed [39:0] value;
    begin
        if (value > 40'sd131071)
            saturate_18 = 18'sd131071;
        else if (value < -40'sd131072)
            saturate_18 = 18'sh20000;
        else
            saturate_18 = value[17:0];
    end
endfunction

/* A 拍：3 个前向乘法并行，寄存 y、b1*x、b2*x。 */
wire signed [35:0] product_b0 = x_register * b0_register;
wire signed [35:0] product_b1 = x_register * b1_register;
wire signed [35:0] product_b2 = x_register * b2_register;
wire signed [39:0] product_b0_ext = {{4{product_b0[35]}}, product_b0};
wire signed [39:0] product_b1_ext = {{4{product_b1[35]}}, product_b1};
wire signed [39:0] product_b2_ext = {{4{product_b2[35]}}, product_b2};
wire signed [39:0] y_accumulator = product_b0_ext + d1_register;
wire signed [17:0] y_phase_a =
    saturate_18(round_q16_to_integer(y_accumulator));

/* B 拍：2 个反馈乘法并行，更新当前段状态。 */
wire signed [35:0] product_a1 = y_register * a1_register;
wire signed [35:0] product_a2 = y_register * a2_register;
wire signed [39:0] product_a1_ext = {{4{product_a1[35]}}, product_a1};
wire signed [39:0] product_a2_ext = {{4{product_a2[35]}}, product_a2};
wire signed [39:0] next_d1 =
    b1x_register - product_a1_ext + d2_register;
wire signed [39:0] next_d2 =
    b2x_register - product_a2_ext;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= STATE_IDLE;
        x_register <= 18'sd0;
        y_register <= 18'sd0;
        b1x_register <= 40'sd0;
        b2x_register <= 40'sd0;
        d1_0 <= 40'sd0;
        d2_0 <= 40'sd0;
        d1_1 <= 40'sd0;
        d2_1 <= 40'sd0;
        d1_2 <= 40'sd0;
        d2_2 <= 40'sd0;
        b0_register <= 18'sd0;
        b1_register <= 18'sd0;
        b2_register <= 18'sd0;
        a1_register <= 18'sd0;
        a2_register <= 18'sd0;
        d1_register <= 40'sd0;
        d2_register <= 40'sd0;
        sample_out <= 18'sd0;
        output_valid <= 1'b0;
    end
    else begin
        output_valid <= 1'b0;

        case (state)
            STATE_IDLE: begin
                if (sample_valid) begin
                    x_register <= sample_in;
                    b0_register <= coeff_b0(2'd0);
                    b1_register <= coeff_b1(2'd0);
                    b2_register <= coeff_b2(2'd0);
                    a1_register <= coeff_a1(2'd0);
                    a2_register <= coeff_a2(2'd0);
                    d1_register <= d1_0;
                    d2_register <= d2_0;
                    state <= STATE_S0_A;
                end
            end

            STATE_S0_A,
            STATE_S1_A,
            STATE_S2_A: begin
                y_register <= y_phase_a;
                b1x_register <= product_b1_ext;
                b2x_register <= product_b2_ext;
                state <= state + 3'd1;
            end

            STATE_S0_B: begin
                d1_0 <= next_d1;
                d2_0 <= next_d2;
                x_register <= y_register;
                b0_register <= coeff_b0(2'd1);
                b1_register <= coeff_b1(2'd1);
                b2_register <= coeff_b2(2'd1);
                a1_register <= coeff_a1(2'd1);
                a2_register <= coeff_a2(2'd1);
                d1_register <= d1_1;
                d2_register <= d2_1;
                state <= STATE_S1_A;
            end

            STATE_S1_B: begin
                d1_1 <= next_d1;
                d2_1 <= next_d2;
                x_register <= y_register;
                b0_register <= coeff_b0(2'd2);
                b1_register <= coeff_b1(2'd2);
                b2_register <= coeff_b2(2'd2);
                a1_register <= coeff_a1(2'd2);
                a2_register <= coeff_a2(2'd2);
                d1_register <= d1_2;
                d2_register <= d2_2;
                state <= STATE_S2_A;
            end

            STATE_S2_B: begin
                d1_2 <= next_d1;
                d2_2 <= next_d2;
                sample_out <= y_register;
                output_valid <= 1'b1;
                state <= STATE_IDLE;
            end

            default: begin
                state <= STATE_IDLE;
            end
        endcase
    end
end

endmodule
