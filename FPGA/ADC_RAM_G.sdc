# 2026 G 题最小闭环时序约束
# sys_clk 为板载 50 MHz；PLL 派生出 50 MHz ADC 与 125 MHz DAC 时钟。
create_clock -name sys_clk -period 20.000 [get_ports {sys_clk}]
derive_pll_clocks
derive_clock_uncertainty

# SPI SCLK 来自 STM32，与 FPGA 系统时钟异步。当前闭环最高按 5 MHz 约束，
# SPI 输入先经过同步器后再做边沿检测。
create_clock -name spi_sclk -period 200.000 [get_ports {SCLK_R}]
set_clock_groups -asynchronous \
    -group [get_clocks {sys_clk}] \
    -group [get_clocks {spi_sclk}]

set_false_path -from [get_ports {sys_rst_n}]
