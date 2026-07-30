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
 * 当前暂不采集或显示时域波形和频谱图。
 */
void GSignalFlow_Process(void);

/* 读取最近一次单次测量结果。 */
const GMeasurementResult *GSignalFlow_GetLatestMeasurement(void);

#ifdef __cplusplus
}
#endif

#endif /* __G_SIGNAL_FLOW_H */
