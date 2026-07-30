# 2026 G题 FPGA采集与滤波工程

最后更新：`2026-07-29 19:21:30 +08:00`

器件为Cyclone IV E `EP4CE6E22C8`，唯一Quartus Revision为
`ADC_RAM_G`，顶层为`g_measure_top`。

## 数据链

```text
AD9238 CH1
  -> CIC3十倍抽取
  -> Q4精度扩展
  -> 三节SOS低通
  -> 1.25MSPS分析帧 / 5MSPS波形帧
  -> 4096 x 16bit双RAM
  -> SPI 32bit命令/响应
```

FPGA ID固定为`0x000004D2`。AD9238 CH2端口继续保留以兼容板卡引脚；
DAC A/B保持中码`8192`和连续时钟，避免未使用输出悬空。

## 保留模块

- `src/g_measure_top.v`
- `src/g_spi_slave.v`
- `src/g_cic3_decimator.v`
- `src/g_sos3_lowpass.v`
- `ip/my_pll`
- `ip/RAM`
- `ip/RAM2`
- `Tests/RTL/tb_g_filter_precision.sv`

工程不包含旧DDS、旧IIR、ROM波形和数字回放Revision。

## 构建结果

使用Quartus Prime 18.0完整编译成功，`0 errors`。所有已分析时序组
`TNS=0.000ns`，最小setup裕量`2.753ns`，最小hold裕量`0.112ns`。

SOF位于：

```text
output_files_g/ADC_RAM_G.sof
```

ModelSim测试输出`PASS: g_filter_precision`。Quartus仍报告未用CH2、
DAC固定中码和板级I/O时序未完全约束等预期警告，详见交付验证记录。
