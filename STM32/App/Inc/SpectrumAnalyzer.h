#ifndef __SPECTRUM_ANALYZER_H
#define __SPECTRUM_ANALYZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SPECTRUM_FRAME_LENGTH       4096U
#define SPECTRUM_SAMPLE_RATE_HZ     1250000.0f
#define SPECTRUM_MAX_COMPONENTS     3U
#define SPECTRUM_MIN_VALID_AMPLITUDE_CODES 16.0f

/*
 * 0：当前量产路径，保持已验证的约0.5 mV Vpp精度。
 * 1：启用保留的0.2 mV实验路径（250 kHz混叠干扰联合拟合与拟合频率细化）。
 */
#ifndef SPECTRUM_ENABLE_02MV_EXPERIMENT
#define SPECTRUM_ENABLE_02MV_EXPERIMENT 1U
#endif

#define SPECTRUM_STATUS_ERROR       0U
#define SPECTRUM_STATUS_VALID       1U
#define SPECTRUM_STATUS_NO_SIGNAL   2U

typedef struct
{
    uint8_t harmonic;
    float frequency_hz;
    float amplitude_codes;
    float phase_rad;
} SpectrumComponent;

typedef struct
{
    uint8_t valid;
    uint8_t status;
    uint8_t component_count;
    uint8_t spur_valid;
    float fundamental_hz;
    float rms_codes;
    float vpp_codes;
    float spur_frequency_hz;
    float spur_amplitude_codes;
    SpectrumComponent components[SPECTRUM_MAX_COMPONENTS];
} SpectrumResult;

/*
 * 对固定 1.25 MSPS、4096 点有符号采样帧执行频谱分析。
 * 当前结果使用 ADC 原始码单位；绝对电压和通道复数频响标定后续接入。
 * 返回0时可检查result->status区分无有效信号和内部分析错误。
 */
uint8_t SpectrumAnalyzer_Run(const int16_t *samples, SpectrumResult *result);

/*
 * 将最近一次FFT的10~500kHz频谱按最大值池化为指定点数，并以60dB动态范围
 * 映射到0~254，供淘晶驰波形控件显示。成功返回1。
 */
uint8_t SpectrumAnalyzer_BuildDisplay(uint8_t *points, uint16_t point_count);

#ifdef __cplusplus
}
#endif

#endif /* __SPECTRUM_ANALYZER_H */
