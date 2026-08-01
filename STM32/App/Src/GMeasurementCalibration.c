#include "GMeasurementCalibration.h"

#include <stddef.h>

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

/*
 * RIGOL DG4162、50 ohm、全部相位0度实机采集：
 * - 10/30/50/100/105/150/200/300/400/450/495/500 kHz实测节点；
 * - 每点10/20/30/40/50/100/150/200/250 mVpp九档；
 * - 每档15帧，使用原始amplitude_codes中位数；
 * - 二维表直接输出最终分量峰值mV，不再额外应用0.5缩放。
 */
#include "../../CalibrationData/dg4162_zero_phase_amplitude.inc"

/*
 * RIGOL DG1022Z、50 ohm接法的新相位响应候选表。
 * 由84条H2~H8相对相位约束求解，去除了任意线性时延。
 * 平滑权重0.001时约束残差RMS约0.097度、最大约0.271度。
 * 不同初相和H2~H8组合实测中，相位残差导致的Vpp误差最大
 * 约0.040 mV；1.0~1.2 MHz、200 mVpp干扰下有效分量峰值变化
 * 不超0.0046 mV，当前表可用于幅值重建。
 */
#include "../../CalibrationData/rigol_phase_w0_001.inc"

/*
 * DG4162谐波模式、f0=50/100 kHz、相位全0度实测。
 * H2~H8使用60 mVpp并带一个伴随谐波得到的倍率均值；
 * 仅用于多分量测量中的Hn，H1和单正弦不应用。
 */
static const GMeasurementHarmonicScalePoint s_HarmonicScalePoints[] =
{
    { 100000.0f, 0.999655600f },
    { 150000.0f, 1.016332300f },
    { 200000.0f, 1.014666900f },
    { 250000.0f, 1.013057600f },
    { 300000.0f, 1.015022700f },
    { 350000.0f, 1.019516600f },
    { 400000.0f, 1.016432200f }
};

static const GMeasurementCalibration s_Calibration =
{
    s_CalibrationPoints,
    (uint8_t)(sizeof(s_CalibrationPoints) /
              sizeof(s_CalibrationPoints[0])),
    s_PhaseCalibrationPoints,
    (uint8_t)(sizeof(s_PhaseCalibrationPoints) /
              sizeof(s_PhaseCalibrationPoints[0])),
    s_AmplitudeCalibrationRows,
    (uint8_t)(sizeof(s_AmplitudeCalibrationRows) /
              sizeof(s_AmplitudeCalibrationRows[0])),
    s_HarmonicScalePoints,
    (uint8_t)(sizeof(s_HarmonicScalePoints) /
              sizeof(s_HarmonicScalePoints[0]))
};

const GMeasurementCalibration *GMeasurementCalibration_Get(void)
{
    return &s_Calibration;
}
