# 2026 G题 STM32测量工程

最后更新：`2026-07-29 19:21:30 +08:00`

本目录是周期信号测量分析装置的STM32F407ZG最小交付工程，仅保留：

- FPGA SPI通信与4096点采集；
- CMSIS-DSP频谱分析、谱峰插值和谐波识别；
- 幅值标定、`Upp`、真`Urms`和波形重建；
- USART1 ASCII调试协议；
- UART4淘晶驰屏幕显示、时域/频域测量及1/3周期切换。

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

USART1为`921600 8N1`，继续输出`G_FLOW`、`G_MEAS`、`G_WAVE`和
`G_WAVE_DATA`调试协议。淘晶驰屏独占UART4（`921600 8N1`），串口、
GPIO、NVIC和接收队列均封装在`tjc_usart_hmi.c`中。
淘晶驰屏按钮弹起事件也可发送以下二进制帧：

```text
55 01 00 00 FF FF FF  -> 时域测量
55 02 00 00 FF FF FF  -> 在1周期和3周期之间切换
55 03 00 00 FF FF FF  -> 频域测量
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

`b0`执行时域测量：采集分析帧并只更新`t0/t1/t2/t6`，随后采集
5 MSPS波形帧并只更新`s0`。波形下限
在压缩为原高度1/3后整体上移20个纵坐标单位。原始5 MSPS波形使用四点
三次插值生成平滑的256点周期数据，再映射为`s0.w`返回的实际控件宽度，
使一个完整周期正好铺满控件；宽度读取失败时才回退为512点。透明传输
严格等待屏幕`FE`就绪和`FD`完成应答，不再依赖固定延时。不进行水平循环移位。
发送到`s0`前会将整组波形点左右镜像，以匹配当前波形控件的实际显示方向。

`b2`执行频域测量：更新`t3/t4/t5`、`s1`和`t6`状态，不改动时域参数
或`s0`。`s1`绘制检测到的最多三个实际频率分量及相对幅值。0.5mV
量化上电默认开启，`b3`可切换；`t6`文字前的`1`表示量化开启、`0`
表示关闭，例如“1状态：测量中”和“0状态：就绪”。

`b1`只负责`s0`在1周期和3周期之间切换，与`s1`无关。b0时域测量进行中
切换只更新待应用值；其他时刻直接使用最近一次b0缓存的5 MSPS帧重画，
不重新采集。三个按钮分别具有120ms防抖，采集忙时新的测量请求会被忽略。
