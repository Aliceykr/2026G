#ifndef __G_SIGNAL_FLOW_H
#define __G_SIGNAL_FLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "GMeasurement.h"

#include <stdint.h>

/* 初始化FPGA接口和串口屏，并自动启动USART1连续测量数据输出。 */
void GSignalFlow_Init(void);

/*
 * 裸机主循环任务：
 * 1. b0读取分析帧和5 MSPS波形帧，只更新t0/t1/t2/t6/s0。
 * 2. b2读取分析帧，更新t3/t4/t5/s1并通过t6显示测量状态。
 * 3. b1只使用最近一次b0缓存的波形帧切换s0的1/3周期。
 * 4. 0.5mV幅值量化默认且始终开启。b3“功能切换”在2958bf3随机数版
 *    与当前优化版之间切换；两种模式分别使用自己的频谱门限、校准表、
 *    谐波补偿、量化后处理和连续采集间隔。
 * 5. 上电后USART1自动连续校准采集；b0/b2测量完成后自动恢复连续采集。
 *    发送'S'在当前帧结束后停止，发送'C'可重新启动。连续模式只输出
 *    诊断数据，不更新屏幕控件。
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
