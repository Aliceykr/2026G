#ifndef __G_MEASUREMENT_CALIBRATION_H
#define __G_MEASUREMENT_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "GMeasurement.h"

/*
 * 返回当前整机的幅频标定表。
 *
 * 当前表来自2026-07-29 High-Z信号源、50/100/200/250 mVpp整机实测。
 * 更改模拟前端、增益、终端方式或ADC/FPGA缩放后必须重新标定。
 */
const GMeasurementCalibration *GMeasurementCalibration_Get(void);

#ifdef __cplusplus
}
#endif

#endif /* __G_MEASUREMENT_CALIBRATION_H */
