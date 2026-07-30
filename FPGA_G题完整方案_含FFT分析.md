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
5. 当前 RAM 是否足以保存 4096 个 16 位 FFT 数据及独立时域帧。

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

实现 4096 点 FFT 帧缓存，并让 STM32 能够完整读取一帧。

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

---

## 12. STM32 FFT分析方案

### 12.1 可复用代码来源

参考仓库：

```text
Dearmoments/stm32-impedance-meter
```

其中包含两套可参考的 FFT 代码：

1. `06A_STM32/06A/Hardware/fft.c`
   - 使用 CMSIS-DSP `arm_cfft_f32()`；
   - FFT 长度为 2048；
   - 使用 `arm_cmplx_mag_f32()`计算幅值；
   - 主要针对已知频率、固定频点的阻抗测量。

2. `06A_STM32/06A/Core/Src/main.c`
   - 包含 Hann 窗；
   - 使用 1024 点 CFFT；
   - 搜索最大谱峰；
   - 使用 `atan2()`读取峰值频点相位；
   - 用于双通道幅值、相位与阻抗分析。

这些代码证明 STM32F407 + CMSIS-DSP 的 FFT 基础链已经可用，以下部分可以复用：

```text
CMSIS-DSP工程配置
arm_math.h / arm_const_structs.h
ADC码值转float流程
Hann窗生成思路
arm_cfft_f32或arm_rfft_fast_f32调用方式
复数幅值计算
最大谱峰搜索
atan2相位提取方法
```

### 12.2 不能直接照搬的部分

阻抗仪工程的 FFT 代码不能原样作为 G 题分析模块，原因包括：

```text
原代码主要使用1024或2048点FFT
部分代码固定读取第512号频点
主要面向已知1kHz/10kHz测试信号
没有通用的频率坐标换算接口
没有完整的基波与谐波识别
没有按Hann窗相干增益修正幅值
没有针对10kHz～500kHz任意组合信号的峰值判决
```

`main.c` 中的“峰值移到中心、卷积Hann窗、再折叠”的处理是阻抗相位测量的特定实现，不建议直接移植到 G 题。

G 题应重新整理为独立、标准化的频谱分析模块。

---

### 12.3 FFT长度与采样率

当前阻抗仪仓库中的 CMSIS-DSP 版本，其快速实数 FFT 支持的最大长度为 4096 点，因此推荐：

```text
FFT点数：4096
FFT采样率：2.048MSPS
频率分辨率：500Hz
最高可分析频率：1.024MHz
```

计算：

\[
\Delta f=\frac{F_s}{N}
=\frac{2.048\text{MHz}}{4096}
=500\text{Hz}
\]

这样既覆盖 500kHz 有用信号，又满足题目 500Hz 频率分辨率要求。

不建议继续采用之前的“2.5MSPS＋8192点CMSIS FFT”表述，因为当前 CMSIS-DSP 的 Cortex-M 实数 FFT 接口并不直接支持 8192 点。

---

### 12.4 推荐软件模块

在 STM32 工程中新建：

```text
App/Inc/SpectrumAnalyzer.h
App/Src/SpectrumAnalyzer.c
App/Inc/SignalMetrics.h
App/Src/SignalMetrics.c
App/Inc/SignalCalibration.h
App/Src/SignalCalibration.c
```

职责划分：

#### `SpectrumAnalyzer`

```text
初始化4096点RFFT
去直流
加Hann窗
执行RFFT
生成单边幅度谱
搜索主要谱峰
进行频率插值
识别基波、二次谐波和三次谐波
```

#### `SignalMetrics`

```text
计算峰峰值
计算直流分量
计算真有效值
计算周期与基频的辅助估计
```

#### `SignalCalibration`

```text
ADC零点修正
ADC电压比例修正
模拟前端各增益档校准
滤波链频率响应补偿
```

---

### 12.5 标准FFT处理流程

```text
FPGA提供4096个均匀采样点
        ↓
ADC码值转换为电压
        ↓
计算并减去平均值
        ↓
乘Hann窗
        ↓
4096点arm_rfft_fast_f32
        ↓
计算0～2048号频点单边幅度
        ↓
窗函数相干增益补偿
        ↓
限定10kHz～500kHz范围搜索谱峰
        ↓
峰值插值
        ↓
推断基波和1～2个谐波
        ↓
输出各分量频率与幅值
```

---

### 12.6 去直流

先计算平均值：

\[
\bar{x}=\frac{1}{N}\sum_{n=0}^{N-1}x[n]
\]

然后：

\[
x_0[n]=x[n]-\bar{x}
\]

真有效值计算应使用校准后的时域数据，不能使用“峰值除以 \(\sqrt2\)”替代。

---

### 12.7 Hann窗与幅值修正

建议使用周期型 Hann 窗：

\[
w[n]=0.5-0.5\cos\left(\frac{2\pi n}{N}\right)
\]

窗函数相干增益：

\[
CG=\frac{1}{N}\sum_{n=0}^{N-1}w[n]
\]

Hann 窗的 `CG` 约为 0.5，但代码中应根据实际生成的窗口求和，不要直接写死。

对于非直流、非奈奎斯特频点，单边峰值幅度初步计算：

\[
A_{\text{peak}}[k]
=
\frac{2|X[k]|}{N\cdot CG}
\]

相应正弦分量的峰峰值：

\[
U_{pp,k}=2A_{\text{peak}}[k]
\]

---

### 12.8 频率坐标

第 \(k\) 个频点对应：

\[
f_k=\frac{kF_s}{N}
\]

本方案中：

\[
f_k=500k\text{ Hz}
\]

例如：

```text
20号频点  = 10kHz
1000号频点 = 500kHz
```

搜索范围可限定为：

```text
k = 20～1000
```

---

### 12.9 谱峰插值

为了减小信号不落在整数频点造成的误差，可对峰值左右相邻三个频点做抛物线插值：

\[
\delta=
\frac{1}{2}
\frac{M[k-1]-M[k+1]}
{M[k-1]-2M[k]+M[k+1]}
\]

频率估计：

\[
f=(k+\delta)\frac{F_s}{N}
\]

需要防止分母接近零，并限制：

\[
-0.5\leq\delta\leq0.5
\]

---

### 12.10 基波与谐波识别

题目信号由基波和 1～2 个谐波组成，因此不能直接认为“最大峰就是基波”。

推荐逻辑：

1. 找出 10kHz～500kHz 范围内幅值最大的若干局部峰；
2. 将每个峰依次作为基波候选；
3. 检查是否存在接近 \(2f_0\)、\(3f_0\) 的峰；
4. 允许频率误差落在若干个 FFT 频点范围内；
5. 选择能解释最多主要谱峰的最低频率作为基波；
6. 无法形成谐波关系时，将最低可信主要峰作为基波候选并标记低置信度。

伪代码：

```c
for each candidate f0:
    score = peak_near(f0)
          + peak_near(2 * f0)
          + peak_near(3 * f0);

choose the lowest candidate with the highest valid score;
```

---

### 12.11 幅值精修

FFT主要负责确定频率。为了更稳定地达到毫伏级幅值误差，建议在得到各分量频率后，再对原始时域数据进行正交投影：

\[
C=\sum x[n]\cos(2\pi fn/F_s)
\]

\[
S=\sum x[n]\sin(2\pi fn/F_s)
\]

\[
A_{\text{peak}}=\frac{2}{N}\sqrt{C^2+S^2}
\]

对于多分量信号，可依次估计并扣除已经识别的分量，或者建立小规模最小二乘模型同时拟合基波、二次谐波和三次谐波。

推荐原则：

```text
FFT：定位频率和判断谱线结构
正交投影/最小二乘：精修各分量幅值
时域直接计算：峰峰值和真有效值
```

---

### 12.12 建议接口

```c
#define SPECTRUM_FFT_SIZE       4096U
#define SPECTRUM_SAMPLE_RATE_HZ 2048000.0f
#define SPECTRUM_BIN_HZ         500.0f
#define SPECTRUM_MAX_COMPONENTS 3U

typedef struct
{
    float frequency_hz;
    float amplitude_peak_v;
    float amplitude_pp_v;
    float amplitude_rms_v;
    float confidence;
} SpectrumComponent;

typedef struct
{
    float fundamental_hz;
    SpectrumComponent components[SPECTRUM_MAX_COMPONENTS];
    uint8_t component_count;
    uint8_t valid;
} SpectrumResult;
```

建议主接口：

```c
bool SpectrumAnalyzer_Init(void);

bool SpectrumAnalyzer_Process(
    const int16_t *samples,
    float adc_lsb_volts,
    SpectrumResult *result
);
```

---

### 12.13 Codex需要重点检查的问题

请 Codex 在实现前先确认：

1. 当前 STM32 工程使用的 CMSIS-DSP 具体版本及 4096 点 RFFT 编译配置；
2. 4096 点输入、输出、窗口和幅值数组的 SRAM 占用；
3. 是否采用 `arm_rfft_fast_f32` 替代当前复数 `arm_cfft_f32`；
4. RFFT 打包输出的实部、虚部索引解释；
5. Hann 窗相干增益补偿是否正确；
6. 峰值插值和谐波匹配的边界条件；
7. FFT结果单位如何从 ADC 码值换算为 mV；
8. 模拟滤波器频率响应如何进入幅值校准表；
9. 4096 点处理耗时能否稳定小于题目要求的 2 秒；
10. FPGA 如何产生严格均匀的 2.048MSPS FFT 数据流。

---

## 13. 更新后的整机数据链

```text
模拟抗混叠低通
        ↓
AD9238高速采样
        ↓
FPGA数字滤波与降采样
        ├─ 较高采样率时域帧
        │      ↓
        │  Upp、真有效值、波形显示
        │
        └─ 2.048MSPS均匀FFT帧，4096点
               ↓
          STM32 CMSIS-DSP RFFT
               ↓
          谱峰检测、基波/谐波识别
               ↓
          正交投影精修幅值
               ↓
          频谱和参数显示
```

