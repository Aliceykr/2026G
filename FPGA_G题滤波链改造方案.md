# FPGA G题滤波链改造方案

## 1. 项目目标

基于仓库 `Dearmoments/FPGAProTwo`，新增一条用于 G 题的数字采集与滤波链：

```text
模拟低通
   ↓
AD9238：50MSPS
   ↓
FPGA降采样：50MSPS → 5MSPS
   ↓
FPGA多级IIR低通
   ↓
波形缓存 / FFT缓存
   ↓
STM32读取分析
```

有用信号范围：

```text
10kHz～500kHz
```

需要抑制的干扰：

```text
fJ ≥ 1MHz
```

---

## 2. 当前代码情况

### ADC链路

顶层 `top.v` 中：

- FPGA 系统时钟为 50MHz；
- AD9238 采样时钟为 50MHz；
- ADC 原始数据连续进入 FPGA。

### 旧IIR不能作为G题主链

`legacy_iir_filter.v` 先产生约 1MSPS 的采样触发，再将抽取后的数据送进旧 IIR。

旧 IIR 又采用多个二阶节串行状态机计算，并由 3 个乘法器分时处理。

因此不要直接修改：

```text
legacy_iir_filter.v
legacy_iir/IIRFilterMod.v
```

这部分继续保留，用于兼容原程控滤波器功能。

### 可参考的模块

`replay_biquad.v` 已包含：

- ADC 无符号数据转有符号；
- S2.30 系数；
- 转置直接Ⅱ型 biquad；
- 64 位乘加；
- 输出饱和；
- ADC 时钟域实时处理。

可以复用其数值格式和饱和思路，但不要直接复制多个组合 biquad。

---

## 3. 推荐架构

### 第一级：模拟抗混叠低通

ADC 前增加模拟低通：

```text
0～500kHz：基本平坦
1MHz：开始明显衰减
2.5MHz以上：较强衰减
```

原因是最终要将采样率降到 5MSPS，其奈奎斯特频率为 2.5MHz。高于 2.5MHz 的输入必须在降采样之前被压低。

因此整体不依赖 FPGA 单独解决所有抗混叠问题。

---

### 第二级：CIC降采样

新增：

```text
src/g_cic_decimator.v
```

参数初步设定：

```text
输入采样率：50MSPS
抽取倍数：10
输出采样率：5MSPS
CIC阶数：先比较3阶和4阶
差分延迟：1
```

结构：

```text
50MSPS ADC
   ↓
CIC积分器
   ↓
每10点输出一次
   ↓
CIC梳状器
   ↓
5MSPS
```

选择 CIC 的原因：

- 50MSPS 部分只使用加法器和寄存器；
- 不需要大量乘法器；
- 适合 EP4CE6 这类资源较小的 FPGA；
- 能在降采样前提供一定的数字抗混叠能力。

需要仿真比较 3 阶和 4 阶 CIC 在 500kHz 处的幅值下垂，并决定：

- 通过标定补偿；
- 或在 5MSPS 域增加一个小型补偿 FIR。

---

### 第三级：5MSPS多级SOS-IIR

新增：

```text
src/g_sos_engine.v
src/g_iir_filter.v
```

滤波指标初步设定：

```text
采样率：5MSPS
通带边缘：500kHz
阻带边缘：1MHz
通带纹波：≤0.1～0.2dB
阻带衰减：初步目标≥40dB
结构：4～5个二阶节
系数：S2.30或根据资源优化
```

#### 推荐计算结构

不要复制 5 份完整 biquad，而是设计一个时分复用 SOS 计算引擎。

5MSPS 下每个输入样本之间有：

\[
50\text{MHz}/5\text{MHz}=10
\]

个时钟周期。

设计目标：

```text
每个二阶节占2个50MHz时钟
最多5个二阶节
一个采样点共10个时钟完成
```

每节采用转置直接Ⅱ型：

```text
y  = b0·x + s1
s1 = b1·x - a1·y + s2
s2 = b2·x - a2·y
```

建议使用 3 个乘法器。

第 1 个时钟同时计算：

```text
b0·x
b1·x
b2·x
```

并得到：

```text
y = b0·x + s1
```

第 2 个时钟同时计算：

```text
a1·y
a2·y
```

随后更新：

```text
s1、s2
```

这样一个 SOS 两拍完成，5 个 SOS 正好需要 10 拍。

---

## 4. 定点实现要求

不要继续使用旧 IIR 中简单截位、默认不会溢出的处理方式。

新模块要求：

```text
输入样本：有符号格式
系数：S2.30
乘法结果：至少64位
累加结果：至少64位
缩放：算术右移30位
截位：使用舍入
越界：使用饱和，不允许回卷
```

每个 SOS 必须独立检查：

- 状态最大值；
- 输出最大值；
- 乘加是否越界；
- 量化后极点是否仍在单位圆内；
- 不同 SOS 排列顺序的内部动态范围。

系数更新后必须清零所有滤波状态。

---

## 5. 缓存结构

新增：

```text
src/g_frame_buffer.v
src/g_capture_ctrl.v
```

滤波后的 5MSPS 数据分为两路：

```text
5MSPS滤波结果
   ├─ 时域缓存
   └─ 每2点取1点 → 2.5MSPS FFT缓存
```

建议：

```text
时域缓存：2048或4096点
FFT缓存：8192点
FFT采样率：2.5MSPS
```

频率分辨率：

\[
\frac{2.5\text{MHz}}{8192}\approx305\text{Hz}
\]

能够满足 500Hz 频率分辨率要求。

---

## 6. 顶层集成

在 `top.v` 中新增独立 G 题链：

```text
AD9238输出
   ↓
g_cic_decimator
   ↓
g_iir_filter
   ↓
g_capture_ctrl
   ↓
g_frame_buffer
```

不要破坏现有：

```text
DDS_9767
replay_biquad
legacy_iir_filter
ram_ctrl
```

增加新的输出选择或采集模式，例如：

```text
MODE_NORMAL
MODE_LEGACY_IIR
MODE_G_ANALYZER
```

G 题链主要用于缓存数据，不要求接管 DAC 输出。

---

## 7. SPI协议扩展

在现有 32 位 SPI 协议上增加新的命令区，避免占用旧 IIR 的 `0x51～0x58`。

建议命令：

```text
G_START_CAPTURE
G_STOP_CAPTURE
G_READ_STATUS
G_READ_WAVE_DATA
G_READ_FFT_DATA
G_SET_FILTER_ENABLE
G_CLEAR_FILTER_STATE
G_READ_OVERFLOW_STATUS
```

状态寄存器至少包括：

```text
capture_done
wave_buffer_ready
fft_buffer_ready
filter_overflow
sample_overrun
current_mode
```

需要支持连续突发读取，不能让 STM32 每读取一个样本都重新发送完整命令。

---

## 8. 开发步骤

### 阶段一：静态分析

先不要写代码，检查并给出：

1. EP4CE6 可用乘法器和 RAM 资源；
2. 当前工程资源占用；
3. 新增 3 个乘法器、CIC 和 8192 点 RAM 后是否可放置；
4. 50MHz 时序是否可满足；
5. 当前 RAM 是否足以保存 8192 个 16 位数据。

### 阶段二：只实现CIC

完成：

```text
50MSPS → CIC → 5MSPS
```

Testbench 输入：

- 100kHz；
- 500kHz；
- 1MHz；
- 2.5MHz；
- 5MHz 正弦。

验证输出频率和幅值。

### 阶段三：实现单个SOS引擎

只加入一个二阶节，验证：

- 两时钟完成一个 SOS；
- 连续 5MSPS 输入不丢点；
- 输出与 Python/MATLAB 浮点结果一致；
- 饱和和舍入正确。

### 阶段四：扩展到5个SOS

完成系数寄存器、状态寄存器和二阶节循环。

验证一个采样点严格在 10 个时钟以内完成。

### 阶段五：加入缓存和SPI读取

实现 8192 点 FFT 帧缓存，并让 STM32 能够完整读取一帧。

---

## 9. 仿真验收条件

至少测试以下输入：

```text
100kHz，250mVpp
500kHz，250mVpp
1MHz，200mVpp
500kHz + 1MHz叠加
基波 + 二次谐波
基波 + 三次谐波
```

检查：

```text
500kHz通带幅度误差
1MHz干扰衰减
内部状态是否溢出
输出是否削顶
是否丢采样点
滤波器启动瞬态
更换系数后的状态清零
```

初步数字指标：

```text
500kHz增益误差：≤0.2dB
1MHz衰减：≥40dB
sample_overrun：始终为0
filter_overflow：始终为0
RTL与浮点参考误差：控制在数个LSB内
```

后续再根据模拟前端和整机 ±5mV 测量误差调整指标。

---

## 10. 请Codex先回答的问题

在开始写代码前，请先给出：

1. 这个“CIC 降到 5MSPS＋两拍 SOS 引擎”的架构是否可行；
2. 3 个乘法器能否在两拍内完成一个 DF2T 二阶节；
3. 现有 EP4CE6 资源是否足够；
4. CIC 选择 3 阶还是 4 阶更合适；
5. 8192 点缓存应复用现有 RAM 还是新建双口 RAM；
6. 需要修改哪些具体文件；
7. 推荐的模块接口和状态机；
8. 预估 Quartus 时序风险；
9. 是否需要把 32 位系数和样本位宽适当缩减；
10. 请先提供模块框图和修改清单，不要直接重写整个工程。

---

## 11. 核心原则

```text
旧IIR兼容链保留，不继续扩展
新建G题独立数据链
模拟滤波负责主要抗混叠保障
CIC完成50MSPS到5MSPS降采样
5MSPS域使用多级SOS精确抑制1MHz
滤波后数据进入RAM，不以DAC回放为核心
先仿真和资源评估，再开始全面编码
```
