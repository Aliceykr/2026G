#ifndef __G_MEASUREMENT_H
#define __G_MEASUREMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "SpectrumAnalyzer.h"

#include <stdint.h>

#define G_MEASUREMENT_WAVEFORM_SAMPLE_RATE_HZ  5000000.0f
#define G_MEASUREMENT_WAVEFORM_POINTS          256U

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

typedef struct
{
    const GMeasurementCalibrationPoint *points;
    uint8_t point_count;
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
