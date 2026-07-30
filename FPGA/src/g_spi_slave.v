/*
 * G 题最小闭环 SPI 从机
 *
 * - SPI Mode 0，MSB first，固定 32 bit 一帧。
 * - SCLK/CS/MOSI 先同步到 50 MHz sys_clk 域，再做边沿检测。
 * - tx_word 可在两帧之间更新；CS 下降沿锁存本帧返回值。
 * - 当前闭环按“一次 HAL 事务对应一帧、每帧切换一次 CS”使用。
 */
module g_spi_slave(
    input             clk,
    input             rst_n,
    input             spi_cs,
    input             spi_sclk,
    input             spi_mosi,
    output reg        spi_miso,
    input      [31:0] tx_word,
    output reg [31:0] rx_word,
    output reg        rx_valid
);

reg [2:0] cs_sync;
reg [2:0] sclk_sync;
reg [1:0] mosi_sync;
reg [31:0] tx_shift;
reg [31:0] rx_shift;
reg [5:0] bit_count;

wire cs_fall;
wire sclk_rise;
wire sclk_fall;
wire selected;

assign cs_fall   = (cs_sync[2:1] == 2'b10);
assign sclk_rise = (sclk_sync[2:1] == 2'b01);
assign sclk_fall = (sclk_sync[2:1] == 2'b10);
assign selected  = ~cs_sync[2];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cs_sync   <= 3'b111;
        sclk_sync <= 3'b000;
        mosi_sync <= 2'b00;
    end
    else begin
        cs_sync   <= {cs_sync[1:0], spi_cs};
        sclk_sync <= {sclk_sync[1:0], spi_sclk};
        mosi_sync <= {mosi_sync[0], spi_mosi};
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_shift <= 32'd0;
        rx_shift <= 32'd0;
        rx_word  <= 32'd0;
        rx_valid <= 1'b0;
        bit_count <= 6'd0;
        spi_miso <= 1'b0;
    end
    else begin
        rx_valid <= 1'b0;

        if (cs_fall) begin
            tx_shift <= tx_word;
            rx_shift <= 32'd0;
            bit_count <= 6'd0;
            spi_miso <= tx_word[31];
        end
        else if (!selected) begin
            bit_count <= 6'd0;
            spi_miso <= 1'b0;
        end
        else begin
            if (sclk_rise) begin
                rx_shift <= {rx_shift[30:0], mosi_sync[1]};
                if (bit_count == 6'd31) begin
                    rx_word <= {rx_shift[30:0], mosi_sync[1]};
                    rx_valid <= 1'b1;
                end
                bit_count <= bit_count + 6'd1;
            end

            if (sclk_fall) begin
                tx_shift <= {tx_shift[30:0], 1'b0};
                spi_miso <= tx_shift[30];
            end
        end
    end
end

endmodule
