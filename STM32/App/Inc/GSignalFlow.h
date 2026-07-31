#ifndef __G_SIGNAL_FLOW_H
#define __G_SIGNAL_FLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "GMeasurement.h"

#include <stdint.h>

/* 初始化FPGA接口和串口屏；上电后分别等待时域或频域测量按键。 */
void GSignalFlow_Init(void);

/*
 * 裸机主循环任务：
 * 1. b0读取分析帧和5 MSPS波形帧，只更新t0/t1/t2/t6/s0。
 * 2. b2读取分析帧，更新t3/t4/t5/s1并通过t6显示测量状态。
 * 3. b1只使用最近一次b0缓存的波形帧切换s0的1/3周期。
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
