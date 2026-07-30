#include "GMeasurementCalibration.h"

/*
 * 2026-07-29整机实测标定：
 * - 信号源负载设置为High-Z；
 * - 每个频点使用50/100/200/250 mVpp单频正弦；
 * - 每个系数由四档平均h1码值等权、强制过原点最小二乘拟合得到。
 *
 * 更换模拟前端、增益、终端方式或ADC/FPGA缩放后必须重新标定。
 */
// static const GMeasurementCalibrationPoint s_CalibrationPoints[] =
// {
//     {  10000.0f, 0.035275259f },
//     {  50000.0f, 0.035068414f },
//     { 100000.0f, 0.034883149f },
//     { 200000.0f, 0.035057586f },
//     { 300000.0f, 0.035663438f },
//     { 400000.0f, 0.035773331f },
//     { 500000.0f, 0.036831843f }
// };

static const GMeasurementCalibrationPoint s_CalibrationPoints[] =
{
    {   10000.0f, 0.031822806f },  // 10 kHz (KEEP)
    {   50000.0f, 0.031795286f },  // 50 kHz (KEEP)
    {  100000.0f, 0.031694095f },  // 100 kHz (KEEP)
    {  200000.0f, 0.031783351f },  // 200 kHz (NEW)
    {  250000.0f, 0.032039673f },  // 250 kHz (LERP)
    {  300000.0f, 0.032295994f },  // 300 kHz (NEW)
    {  350000.0f, 0.032350333f },  // 350 kHz (LERP)
    {  400000.0f, 0.032404671f },  // 400 kHz (NEW)
    {  450000.0f, 0.032853284f },  // 450 kHz (LERP)
    {  500000.0f, 0.033301896f },  // 500 kHz (NEW)
};

static const GMeasurementCalibration s_Calibration =
{
    s_CalibrationPoints,
    (uint8_t)(sizeof(s_CalibrationPoints) /
              sizeof(s_CalibrationPoints[0]))
};

const GMeasurementCalibration *GMeasurementCalibration_Get(void)
{
    return &s_Calibration;
}
