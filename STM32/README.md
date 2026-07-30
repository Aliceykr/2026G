# 2026 G题 STM32测量工程

最后更新：`2026-07-29 19:21:30 +08:00`

本目录是周期信号测量分析装置的STM32F407ZG最小交付工程，仅保留：

- FPGA SPI通信与4096点采集；
- CMSIS-DSP频谱分析、谱峰插值和谐波识别；
- 幅值标定、`Upp`、真`Urms`和波形重建；
- USART1 ASCII调试协议；
- UART4淘晶驰屏幕显示、开始测量及1/3周期切换。

## 入口与接口

主循环固定为：

```text
HAL_Init
  -> SystemClock_Config
  -> MX_GPIO_Init
  -> MX_SPI1_Init
  -> MX_USART1_UART_Init
  -> GSerial_Init
  -> GSignalFlow_Init
  -> GSignalFlow_Process
```

引脚：

| 功能 | STM32引脚 |
|---|---|
| FPGA片选 | PA4 |
| SPI1 SCK/MISO/MOSI | PA5/PA6/PA7 |
| USART1 TX/RX | PA9/PA10 |
| 淘晶驰UART4 TX/RX | PA0/PA1 |
| SWDIO/SWCLK | PA13/PA14 |

USART1为`115200 8N1`，继续输出`G_FLOW`、`G_MEAS`、`G_WAVE`和
`G_WAVE_DATA`调试协议。淘晶驰屏独占UART4（`921600 8N1`），串口、
GPIO、NVIC和接收队列均封装在`tjc_usart_hmi.c`中。
淘晶驰屏按钮弹起事件也可发送以下二进制帧：

```text
55 01 00 00 FF FF FF  -> 开始测量
55 02 00 00 FF FF FF  -> 在1周期和3周期之间切换
```

## 构建

Keil工程：

```text
MDK-ARM/stm32_FilterShell_2026GA.uvprojx
```

已验证使用Keil MDK 5、ARM Compiler 5.06 update 7构建，结果为
`0 Error(s), 0 Warning(s)`。生成的HEX位于：

```text
Build/stm32_FilterShell_2026GA.hex
```

详细命令和测试方法见交付根目录的`构建与烧录说明.md`。

## 目录边界

- `App`：G题业务算法；
- `BSP`：FPGA通信；
- `Core`：CubeMX初始化和`GSerial`；
- `Drivers`：完整HAL、CMSIS和CMSIS-DSP；
- `Tests`：Windows主机算法测试；
- `Docs`：无屏联调、标定和实测记录。

本工程不包含旧LCD、矩阵键盘、DAC7811、ADS1256、MCU ADC自检和旧IIR
滤波器业务。

## 当前串口屏联调模式

按下开始测量后执行一次真实FPGA分析帧采集，计算并显示频率、Vpp、Urms
及H1～H3，随后保持结果。当前暂不采集第二个波形帧，也不向`s0`、`s1`
发送时域波形或频谱图。
