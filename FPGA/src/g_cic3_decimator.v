/*
 * 3 阶 CIC 抽取器，R=10。
 *
 * 输入为以 0 为中心的 13 bit ADC 码，输入速率 50 MSPS。
 * 输出速率 5 MSPS。CIC 直流增益为 10^3，输出用 /1024 近似归一化，
 * 剩余固定增益统一留给 STM32 标定层处理。
 */
module g_cic3_decimator(
    input                       clk,
    input                       rst_n,
    input      signed [12:0]    sample_in,
    output reg signed [17:0]    sample_out,
    output reg                  sample_valid
);

reg signed [23:0] integrator1;
reg signed [23:0] integrator2;
reg signed [23:0] integrator3;

reg signed [23:0] comb1_delay;
reg signed [23:0] comb2_delay;
reg signed [23:0] comb3_delay;
reg signed [23:0] comb1;
reg signed [23:0] comb2;
reg signed [23:0] comb3;

reg [3:0] dec_count;

function signed [17:0] normalize_and_saturate;
    input signed [23:0] value;
    reg signed [24:0] magnitude;
    reg signed [24:0] rounded;
    begin
        magnitude = 25'sd0;
        rounded = 25'sd0;
        if (value < 0) begin
            magnitude = -{{1{value[23]}}, value};
            rounded = -((magnitude + 25'sd512) >>> 10);
        end
        else begin
            rounded = (value + 25'sd512) >>> 10;
        end

        if (rounded > 25'sd131071)
            normalize_and_saturate = 18'sd131071;
        else if (rounded < -25'sd131072)
            normalize_and_saturate = 18'sh20000;
        else
            normalize_and_saturate = rounded[17:0];
    end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        integrator1 <= 24'sd0;
        integrator2 <= 24'sd0;
        integrator3 <= 24'sd0;
        comb1_delay <= 24'sd0;
        comb2_delay <= 24'sd0;
        comb3_delay <= 24'sd0;
        comb1 <= 24'sd0;
        comb2 <= 24'sd0;
        comb3 <= 24'sd0;
        dec_count <= 4'd0;
        sample_out <= 18'sd0;
        sample_valid <= 1'b0;
    end
    else begin
        integrator1 <= integrator1 + sample_in;
        integrator2 <= integrator2 + integrator1;
        integrator3 <= integrator3 + integrator2;
        sample_valid <= 1'b0;

        if (dec_count == 4'd9) begin
            dec_count <= 4'd0;

            comb1 <= integrator3 - comb1_delay;
            comb1_delay <= integrator3;

            comb2 <= comb1 - comb2_delay;
            comb2_delay <= comb1;

            comb3 <= comb2 - comb3_delay;
            comb3_delay <= comb2;

            sample_out <= normalize_and_saturate(comb3);
            sample_valid <= 1'b1;
        end
        else begin
            dec_count <= dec_count + 4'd1;
        end
    end
end

endmodule
