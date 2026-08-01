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
 * RIGOL DG1022Z、50 ohm负载接法实机采集：
 * - 10~500 kHz以10 kHz为基础网格，并在105 kHz、495 kHz
 *   两个插值薄弱区增加实测节点；
 * - 每点10/20/30/40/50/100/150/200/250 mVpp九档；
 * - 每档25帧，使用原始amplitude_codes中位数；
 * - 二维表直接输出最终分量峰值mV，不再额外应用0.5缩放。
 * - 2026-08-01复测100/150/300/500 kHz、50~250 mVpp后，
 *   使用新采集的原始码值替换四个锚点，并按频率线性插值
 *   更新100~500 kHz中间行的50~250 mVpp五档节点。
 */
#include "../../CalibrationData/rigol_amplitude_full_9level.inc"

/*
 * RIGOL DG1022Z、50 ohm接法的新相位响应候选表。
 * 由84条H2~H8相对相位约束求解，去除了任意线性时延。
 * 平滑权重0.001时约束残差RMS约0.097度、最大约0.271度。
 * 不同初相和H2~H8组合实测中，相位残差导致的Vpp误差最大
 * 约0.040 mV；1.0~1.2 MHz、200 mVpp干扰下有效分量峰值变化
 * 不超0.0046 mV，当前表可用于幅值重建。
 */
#include "../../CalibrationData/rigol_phase_w0_001.inc"

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
              sizeof(s_AmplitudeCalibrationRows[0]))
};

const GMeasurementCalibration *GMeasurementCalibration_Get(void)
{
    return &s_Calibration;
}
