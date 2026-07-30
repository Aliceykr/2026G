/*
 * 2026 G 题最小数据闭环顶层。
 *
 * 本 Revision 只保留：
 *   AD9238 CH1 -> CIC3(R=10) -> Q4 精度扩展 -> 3 SOS 低通
 *   -> 1.25 MSPS 分析帧或 5 MSPS 波形帧
 *   -> 4096 x 16 bit 双 RAM bank
 *   -> SPI1 32 bit 命令/响应
 *
 * 交付工程不包含DDS、旧IIR和数字回放模块。
 */
module g_measure_top(
    input            sys_clk,
    input            sys_rst_n,

    input  [11:0]    ad1_in,
    input  [11:0]    ad2_in,
    output           ad1_clk,
    output           ad2_clk,

    output [13:0]    dac_dataA,
    output           dac_clkA,
    output           dac_wrtA,
    output [13:0]    dac_dataB,
    output           dac_clkB,
    output           dac_wrtB,

    input            SPI_CS_R,
    input            SCLK_R,
    input            MOSI,
    output           MISO
);

localparam [7:0] CMD_NOP            = 8'h00;
localparam [7:0] CMD_READ_ID        = 8'h20;
localparam [7:0] CMD_READ_STATUS    = 8'h21;
localparam [7:0] CMD_READ_SAMPLE    = 8'h22;
localparam [7:0] CMD_START_ANALYSIS = 8'h30;
localparam [7:0] CMD_START_WAVEFORM = 8'h31;

localparam [31:0] FPGA_ID = 32'h000004D2;
localparam [15:0] SAMPLE_TAG = 16'h5341;

wire adc_clk;
wire clk_125M;

my_pll my_pll_inst(
    .inclk0(sys_clk),
    .c0(adc_clk),
    .c1(clk_125M)
);

assign ad1_clk = adc_clk;
assign ad2_clk = adc_clk;

/* G 题采集 Revision 不使用 DAC，保持中码和连续时钟，避免输出悬空。 */
assign dac_dataA = 14'd8192;
assign dac_dataB = 14'd8192;
assign dac_clkA = clk_125M;
assign dac_clkB = clk_125M;
assign dac_wrtA = clk_125M;
assign dac_wrtB = clk_125M;

/* ADC 通道 1 由 offset-binary 转为以零为中心的有符号码。 */
reg signed [12:0] adc_sample;

always @(posedge adc_clk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        adc_sample <= 13'sd0;
    else
        adc_sample <= $signed({1'b0, ad1_in}) - 13'sd2048;
end

wire signed [17:0] cic_sample;
wire cic_valid;

g_cic3_decimator cic_inst(
    .clk(adc_clk),
    .rst_n(sys_rst_n),
    .sample_in(adc_sample),
    .sample_out(cic_sample),
    .sample_valid(cic_valid)
);

/*
 * CIC 输出的有效范围仍接近 12 bit。进入 SOS 前左移 4 位，利用 18 bit
 * 通路保留 4 位额外小数精度，避免每个 SOS 的整数舍入在阻带形成限环杂散。
 * 最终滤波结果直接以 Q4 扩展码写入 16 bit RAM，电压标定层统一换算。
 */
wire signed [17:0] cic_sample_q4 = cic_sample <<< 4;

wire signed [17:0] filtered_sample;
wire filtered_valid;

g_sos3_lowpass lowpass_inst(
    .clk(adc_clk),
    .rst_n(sys_rst_n),
    .sample_in(cic_sample_q4),
    .sample_valid(cic_valid),
    .sample_out(filtered_sample),
    .output_valid(filtered_valid)
);

function signed [15:0] saturate_16;
    input signed [17:0] value;
    begin
        if (value > 18'sd32767)
            saturate_16 = 16'sd32767;
        else if (value < -18'sd32768)
            saturate_16 = 16'sh8000;
        else
            saturate_16 = value[15:0];
    end
endfunction

/* ------------------------------------------------------------------------- */
/* sys_clk -> adc_clk：采集启动 toggle 和模式同步。                           */
/* mode=0：分析帧，低通输出再 /4，得到 1.25 MSPS。                           */
/* mode=1：波形帧，直接保存 5 MSPS 低通输出。                                */
/* ------------------------------------------------------------------------- */
reg start_toggle_sys;
reg capture_mode_sys;

reg start_toggle_adc_meta;
reg start_toggle_adc;
reg start_toggle_adc_seen;
reg capture_mode_adc_meta;
reg capture_mode_adc;

reg capture_active_adc;
reg [11:0] capture_addr_adc;
reg [1:0] analysis_decim_count;
reg done_toggle_adc;

wire analysis_take;
wire capture_take;
wire signed [15:0] capture_data;

assign analysis_take = (analysis_decim_count == 2'd3);
assign capture_take = capture_active_adc && filtered_valid &&
                      (capture_mode_adc || analysis_take);
assign capture_data = saturate_16(filtered_sample);

always @(posedge adc_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        start_toggle_adc_meta <= 1'b0;
        start_toggle_adc <= 1'b0;
        capture_mode_adc_meta <= 1'b0;
        capture_mode_adc <= 1'b0;
    end
    else begin
        start_toggle_adc_meta <= start_toggle_sys;
        start_toggle_adc <= start_toggle_adc_meta;
        capture_mode_adc_meta <= capture_mode_sys;
        capture_mode_adc <= capture_mode_adc_meta;
    end
end

always @(posedge adc_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        start_toggle_adc_seen <= 1'b0;
        capture_active_adc <= 1'b0;
        capture_addr_adc <= 12'd0;
        analysis_decim_count <= 2'd0;
        done_toggle_adc <= 1'b0;
    end
    else begin
        if (start_toggle_adc != start_toggle_adc_seen) begin
            start_toggle_adc_seen <= start_toggle_adc;
            capture_active_adc <= 1'b1;
            capture_addr_adc <= 12'd0;
            analysis_decim_count <= 2'd0;
        end
        else if (capture_active_adc && filtered_valid) begin
            if (capture_mode_adc)
                analysis_decim_count <= 2'd0;
            else if (analysis_decim_count == 2'd3)
                analysis_decim_count <= 2'd0;
            else
                analysis_decim_count <= analysis_decim_count + 2'd1;

            if (capture_take) begin
                if (capture_addr_adc == 12'd4095) begin
                    capture_active_adc <= 1'b0;
                    done_toggle_adc <= ~done_toggle_adc;
                end
                else begin
                    capture_addr_adc <= capture_addr_adc + 12'd1;
                end
            end
        end
    end
end

/* ------------------------------------------------------------------------- */
/* 两个 2048 x 16 RAM 作为一个逻辑 4096 x 16 单通道帧。                      */
/* ------------------------------------------------------------------------- */
reg [11:0] ram_read_addr_sys;
wire [15:0] ram_bank0_q;
wire [15:0] ram_bank1_q;
wire ram_bank0_wren;
wire ram_bank1_wren;

assign ram_bank0_wren = capture_take && !capture_addr_adc[11];
assign ram_bank1_wren = capture_take && capture_addr_adc[11];

RAM ram_bank0(
    .data(capture_data),
    .rdaddress(ram_read_addr_sys[10:0]),
    .rdclock(sys_clk),
    .rden(1'b1),
    .wraddress(capture_addr_adc[10:0]),
    .wrclock(adc_clk),
    .wren(ram_bank0_wren),
    .q(ram_bank0_q)
);

RAM2 ram_bank1(
    .data(capture_data),
    .rdaddress(ram_read_addr_sys[10:0]),
    .rdclock(sys_clk),
    .rden(1'b1),
    .wraddress(capture_addr_adc[10:0]),
    .wrclock(adc_clk),
    .wren(ram_bank1_wren),
    .q(ram_bank1_q)
);

/* ------------------------------------------------------------------------- */
/* SPI 命令与 sys_clk 域状态。                                                */
/* 响应采用一帧流水：本帧提交命令，下一帧读回该命令产生的响应。              */
/* ------------------------------------------------------------------------- */
reg [31:0] spi_tx_word;
wire [31:0] spi_rx_word;
wire spi_rx_valid;

g_spi_slave spi_inst(
    .clk(sys_clk),
    .rst_n(sys_rst_n),
    .spi_cs(SPI_CS_R),
    .spi_sclk(SCLK_R),
    .spi_mosi(MOSI),
    .spi_miso(MISO),
    .tx_word(spi_tx_word),
    .rx_word(spi_rx_word),
    .rx_valid(spi_rx_valid)
);

reg done_toggle_sys_meta;
reg done_toggle_sys;
reg done_toggle_sys_seen;
reg capture_busy_sys;
reg capture_done_sys;

reg read_bank_pending;
reg [2:0] read_wait_count;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        done_toggle_sys_meta <= 1'b0;
        done_toggle_sys <= 1'b0;
    end
    else begin
        done_toggle_sys_meta <= done_toggle_adc;
        done_toggle_sys <= done_toggle_sys_meta;
    end
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        start_toggle_sys <= 1'b0;
        capture_mode_sys <= 1'b0;
        done_toggle_sys_seen <= 1'b0;
        capture_busy_sys <= 1'b0;
        capture_done_sys <= 1'b0;
        ram_read_addr_sys <= 12'd0;
        read_bank_pending <= 1'b0;
        read_wait_count <= 3'd0;
        spi_tx_word <= 32'h47303031;
    end
    else begin
        if (done_toggle_sys != done_toggle_sys_seen) begin
            done_toggle_sys_seen <= done_toggle_sys;
            capture_busy_sys <= 1'b0;
            capture_done_sys <= 1'b1;
        end

        if (read_wait_count != 3'd0) begin
            read_wait_count <= read_wait_count - 3'd1;
            if (read_wait_count == 3'd1) begin
                spi_tx_word <= {
                    SAMPLE_TAG,
                    (read_bank_pending ? ram_bank1_q : ram_bank0_q)
                };
            end
        end

        if (spi_rx_valid) begin
            case (spi_rx_word[31:24])
                CMD_NOP: begin
                    /* 保持上一条命令产生的响应。 */
                end

                CMD_READ_ID: begin
                    spi_tx_word <= FPGA_ID;
                end

                CMD_READ_STATUS: begin
                    spi_tx_word <= {
                        16'd4096,
                        12'd0,
                        1'b0,
                        capture_mode_sys,
                        capture_busy_sys,
                        capture_done_sys
                    };
                end

                CMD_READ_SAMPLE: begin
                    ram_read_addr_sys <= spi_rx_word[11:0];
                    read_bank_pending <= spi_rx_word[11];
                    read_wait_count <= 3'd3;
                end

                CMD_START_ANALYSIS: begin
                    capture_mode_sys <= 1'b0;
                    start_toggle_sys <= ~start_toggle_sys;
                    capture_busy_sys <= 1'b1;
                    capture_done_sys <= 1'b0;
                    spi_tx_word <= 32'h414E4C59;
                end

                CMD_START_WAVEFORM: begin
                    capture_mode_sys <= 1'b1;
                    start_toggle_sys <= ~start_toggle_sys;
                    capture_busy_sys <= 1'b1;
                    capture_done_sys <= 1'b0;
                    spi_tx_word <= 32'h57415645;
                end

                default: begin
                    spi_tx_word <= 32'hBAD00000 | {24'd0, spi_rx_word[31:24]};
                end
            endcase
        end
    end
end

/* 通道 2 在最小闭环阶段不进入算法，显式引用以避免误解为漏接。 */
wire unused_ad2_bit = ^ad2_in;

endmodule
