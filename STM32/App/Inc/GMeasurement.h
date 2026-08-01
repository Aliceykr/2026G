#ifndef __G_MEASUREMENT_H
#define __G_MEASUREMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "SpectrumAnalyzer.h"

#include <stdint.h>

#define G_MEASUREMENT_WAVEFORM_SAMPLE_RATE_HZ  5000000.0f
#define G_MEASUREMENT_WAVEFORM_POINTS          256U
#define G_MEASUREMENT_MAX_AMPLITUDE_LEVELS      9U

/*
 * 50 ohm信号源幅值换算。
 * 设为0可恢复原High-Z换算路径；当前启用后，最多三个有效谐波分量
 * 先统一除以2，再用于幅值显示、Vrms和Vpp重建。
 */
#ifndef G_MEASUREMENT_ENABLE_50_OHM_SCALE
#define G_MEASUREMENT_ENABLE_50_OHM_SCALE       1U
#endif

#define G_MEASUREMENT_50_OHM_AMPLITUDE_SCALE    0.5f

typedef struct
{
    float frequency_hz;
    float mv_per_code;
} GMeasurementCalibrationPoint;

/*
 * 测量链的通用相位响应误差点。
 *
 * phase_error_rad 不是单次采集的绝对相位，而是已经去除任意线性时延后、
 * 随绝对频率变化的连续展开相位误差 P(f)。标定工具必须先对相位解包，
 * 频点之间再由固件线性插值。
 */
typedef struct
{
    float frequency_hz;
    float phase_error_rad;
} GMeasurementPhaseCalibrationPoint;

typedef struct
{
    float amplitude_codes;
    float peak_mv;
} GMeasurementAmplitudeCalibrationLevel;

/*
 * 同一绝对频率下的码值到最终峰值毫伏分段曲线。
 * peak_mv 已经是最终50 ohm显示口径，使用本曲线时不再额外乘0.5。
 */
typedef struct
{
    float frequency_hz;
    uint8_t level_count;
    GMeasurementAmplitudeCalibrationLevel
        levels[G_MEASUREMENT_MAX_AMPLITUDE_LEVELS];
} GMeasurementAmplitudeCalibrationRow;

/*
 * 多谐波联合拟合相对单正弦标定的额外幅值倍率。
 * measured_scale = 多谐波测得峰值 / 信号源显示峰值。
 */
typedef struct
{
    float frequency_hz;
    float measured_scale;
} GMeasurementHarmonicScalePoint;

typedef struct
{
    const GMeasurementCalibrationPoint *points;
    uint8_t point_count;
    const GMeasurementPhaseCalibrationPoint *phase_points;
    uint8_t phase_point_count;
    const GMeasurementAmplitudeCalibrationRow *amplitude_rows;
    uint8_t amplitude_row_count;
    const GMeasurementHarmonicScalePoint *harmonic_scale_points;
    uint8_t harmonic_scale_point_count;
} GMeasurementCalibration;

typedef struct
{
    uint8_t harmonic;
    float frequency_hz;
    float amplitude_mv;
    float phase_rad;
} GMeasurementComponent;

typedef struct
{
    uint8_t valid;
    uint8_t component_count;
    uint8_t spur_valid;
    float fundamental_hz;
    float upp_mv;
    float urms_mv;
    float spur_frequency_hz;
    float spur_amplitude_mv;
    GMeasurementComponent components[SPECTRUM_MAX_COMPONENTS];
} GMeasurementResult;

typedef struct
{
    uint8_t valid;
    uint8_t cycles;
    uint16_t point_count;
    float start_sample;
    float span_samples;
    int16_t minimum_code;
    int16_t maximum_code;
    int16_t points[G_MEASUREMENT_WAVEFORM_POINTS];
} GMeasurementWaveform;

/*
 * 把频谱分析得到的 ADC/FPGA 码值换算为输入端毫伏值。
 *
 * calibration 必须按 frequency_hz 严格递增排列。频点之间线性插值，
 * 频点范围以外使用最近端点。没有有效标定表时返回 0，不输出伪造毫伏值。
 */
uint8_t GMeasurement_Convert(const SpectrumResult *spectrum,
                             const GMeasurementCalibration *calibration,
                             GMeasurementResult *measurement);

/*
 * 内部复制已换算的各分量峰值，并按
 * round(amplitude_mv * 2) / 2量化为0.5mV后重新计算Vpp和Vrms。
 * 原生分量幅值保持不变，供串口屏与遥测继续显示原始小数值。
 */
void GMeasurement_QuantizeHalfMv(GMeasurementResult *measurement);

/*
 * 从 5 MSPS 波形帧中按基波相位对齐，生成固定 256 点的 1 周期或 3 周期
 * 屏幕无关波形数据。cycles 只接受 1 或 3。
 */
uint8_t GMeasurement_BuildWaveform(const int16_t *samples,
                                   uint16_t sample_count,
                                   float sample_rate_hz,
                                   float fundamental_hz,
                                   uint8_t cycles,
                                   GMeasurementWaveform *waveform);

#ifdef __cplusplus
}
#endif

#endif /* __G_MEASUREMENT_H */
