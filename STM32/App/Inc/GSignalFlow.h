#ifndef __G_SIGNAL_FLOW_H
#define __G_SIGNAL_FLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "GMeasurement.h"

#include <stdint.h>

/* 初始化FPGA接口和串口屏；上电后等待用户按下开始测量。 */
void GSignalFlow_Init(void);

/*
 * 裸机主循环任务：
 * 1. 读取 1.25 MSPS 分析帧并完成 FFT、频率和参数计算。
 * 2. 将频率、Vpp、Urms及H1~H3结果发送到淘晶驰屏。
 * 3. 将10kHz~500kHz FFT频谱按低频到高频发送到s1。
 * 4. 读取5 MSPS波形帧并向s0发送时域波形。
 */
void GSignalFlow_Process(void);

/* 选择时域波形显示1个或3个完整周期。 */
uint8_t GSignalFlow_SetWaveformCycles(uint8_t cycles);

/* 读取最近一次单次测量结果。 */
const GMeasurementResult *GSignalFlow_GetLatestMeasurement(void);
const GMeasurementWaveform *GSignalFlow_GetLatestWaveform(void);

#ifdef __cplusplus
}
#endif

#endif /* __G_SIGNAL_FLOW_H */
