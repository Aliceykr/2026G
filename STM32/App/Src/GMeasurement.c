#include "GMeasurement.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define G_MEASUREMENT_TWO_PI              6.28318530717958647692f
#define G_MEASUREMENT_RECONSTRUCT_POINTS  2048U

static uint8_t GMeasurement_ValidateCalibration(
    const GMeasurementCalibration *calibration);
static float GMeasurement_GetMvPerCode(
    const GMeasurementCalibration *calibration,
    float frequency_hz);
static float GMeasurement_CubicInterpolate(const int16_t *samples,
                                           float position);
static int16_t GMeasurement_RoundToInt16(float value);

uint8_t GMeasurement_Convert(const SpectrumResult *spectrum,
                             const GMeasurementCalibration *calibration,
                             GMeasurementResult *measurement)
{
    float sum_square = 0.0f;
    float minimum = 0.0f;
    float maximum = 0.0f;
    uint16_t point;
    uint8_t component;

    if ((spectrum == NULL) ||
        (measurement == NULL) ||
        (spectrum->valid == 0U) ||
        (spectrum->component_count == 0U) ||
        (spectrum->component_count > SPECTRUM_MAX_COMPONENTS) ||
        (GMeasurement_ValidateCalibration(calibration) == 0U))
    {
        if (measurement != NULL)
        {
            memset(measurement, 0, sizeof(*measurement));
        }
        return 0U;
    }

    memset(measurement, 0, sizeof(*measurement));
    measurement->fundamental_hz = spectrum->fundamental_hz;
    measurement->component_count = spectrum->component_count;

    for (component = 0U;
         component < spectrum->component_count;
         component++)
    {
        const SpectrumComponent *input = &spectrum->components[component];
        GMeasurementComponent *output = &measurement->components[component];
        float mv_per_code =
            GMeasurement_GetMvPerCode(calibration, input->frequency_hz);

        output->harmonic = input->harmonic;
        output->frequency_hz = input->frequency_hz;
#if G_MEASUREMENT_ENABLE_50_OHM_SCALE
        output->amplitude_mv =
            input->amplitude_codes *
            mv_per_code *
            G_MEASUREMENT_50_OHM_AMPLITUDE_SCALE;
#else
        /* 保留原High-Z幅值换算路径。 */
        output->amplitude_mv = input->amplitude_codes * mv_per_code;
#endif
        output->phase_rad = input->phase_rad;
        sum_square += output->amplitude_mv * output->amplitude_mv;
    }

    measurement->urms_mv = sqrtf(0.5f * sum_square);

    for (point = 0U; point < G_MEASUREMENT_RECONSTRUCT_POINTS; point++)
    {
        float base_phase =
            G_MEASUREMENT_TWO_PI *
            (float)point /
            (float)G_MEASUREMENT_RECONSTRUCT_POINTS;
        float value = 0.0f;

        for (component = 0U;
             component < measurement->component_count;
             component++)
        {
            const GMeasurementComponent *item =
                &measurement->components[component];

            value += item->amplitude_mv *
                     cosf((float)item->harmonic * base_phase +
                          item->phase_rad);
        }

        if (point == 0U)
        {
            minimum = value;
            maximum = value;
        }
        else
        {
            if (value < minimum)
            {
                minimum = value;
            }
            if (value > maximum)
            {
                maximum = value;
            }
        }
    }

    measurement->upp_mv = maximum - minimum;

    if (spectrum->spur_valid != 0U)
    {
        measurement->spur_valid = 1U;
        measurement->spur_frequency_hz = spectrum->spur_frequency_hz;
        measurement->spur_amplitude_mv =
            spectrum->spur_amplitude_codes *
            GMeasurement_GetMvPerCode(calibration,
                                      spectrum->spur_frequency_hz);
    }

    measurement->valid = 1U;
    return 1U;
}

uint8_t GMeasurement_BuildWaveform(const int16_t *samples,
                                   uint16_t sample_count,
                                   float sample_rate_hz,
                                   float fundamental_hz,
                                   uint8_t cycles,
                                   GMeasurementWaveform *waveform)
{
    float mean = 0.0f;
    float cosine_sum = 0.0f;
    float sine_sum = 0.0f;
    float omega;
    float cosine_state = 1.0f;
    float sine_state = 0.0f;
    float cosine_step;
    float sine_step;
    float phase;
    float period_samples;
    float start_sample;
    float span_samples;
    uint16_t index;

    if ((samples == NULL) ||
        (waveform == NULL) ||
        (sample_count < 4U) ||
        (sample_rate_hz <= 0.0f) ||
        (fundamental_hz <= 0.0f) ||
        (fundamental_hz >= 0.5f * sample_rate_hz) ||
        ((cycles != 1U) && (cycles != 3U)))
    {
        if (waveform != NULL)
        {
            memset(waveform, 0, sizeof(*waveform));
        }
        return 0U;
    }

    memset(waveform, 0, sizeof(*waveform));

    period_samples = sample_rate_hz / fundamental_hz;
    span_samples = period_samples * (float)cycles;
    if (span_samples > (float)(sample_count - 2U))
    {
        return 0U;
    }

    for (index = 0U; index < sample_count; index++)
    {
        mean += (float)samples[index];
    }
    mean /= (float)sample_count;

    omega = G_MEASUREMENT_TWO_PI * fundamental_hz / sample_rate_hz;
    cosine_step = cosf(omega);
    sine_step = sinf(omega);

    for (index = 0U; index < sample_count; index++)
    {
        float centered = (float)samples[index] - mean;
        float next_cosine;
        float next_sine;

        cosine_sum += centered * cosine_state;
        sine_sum += centered * sine_state;

        next_cosine =
            cosine_state * cosine_step - sine_state * sine_step;
        next_sine =
            sine_state * cosine_step + cosine_state * sine_step;
        cosine_state = next_cosine;
        sine_state = next_sine;

        if ((index & 0x00FFU) == 0x00FFU)
        {
            float norm = sqrtf(cosine_state * cosine_state +
                               sine_state * sine_state);
            if (norm > 0.0f)
            {
                cosine_state /= norm;
                sine_state /= norm;
            }
        }
    }

    if ((fabsf(cosine_sum) + fabsf(sine_sum)) < 1.0e-6f)
    {
        return 0U;
    }

    /*
     * x[n] ~= A*cos(omega*n + phase)。
     * 对 cos/sin 的投影分别约为 A*cos(phase) 与 -A*sin(phase)。
     */
    phase = atan2f(-sine_sum, cosine_sum);
    start_sample = -phase / omega;
    while (start_sample < 0.0f)
    {
        start_sample += period_samples;
    }
    while (start_sample >= period_samples)
    {
        start_sample -= period_samples;
    }

    /*
     * 后续采用四点三次插值，需要每个目标位置左右各保留采样点。
     * 在保持基波相位不变的前提下，把显示窗口移动到帧中部，避免总是
     * 使用采集帧开头，同时确保p0..p3均不会越界。
     */
    {
        float latest_start = (float)(sample_count - 3U) - span_samples;
        float target_start;

        while (start_sample < 1.0f)
        {
            start_sample += period_samples;
        }
        if (start_sample > latest_start)
        {
            return 0U;
        }

        target_start = 0.5f * (1.0f + latest_start);
        if (target_start > start_sample)
        {
            float shifts = floorf((target_start - start_sample) /
                                  period_samples);
            start_sample += shifts * period_samples;

            if ((start_sample + period_samples <= latest_start) &&
                (fabsf((start_sample + period_samples) - target_start) <
                 fabsf(start_sample - target_start)))
            {
                start_sample += period_samples;
            }
        }
    }

    if ((start_sample < 1.0f) ||
        (start_sample + span_samples > (float)(sample_count - 3U)))
    {
        return 0U;
    }

    waveform->minimum_code = 32767;
    waveform->maximum_code = -32768;

    for (index = 0U; index < G_MEASUREMENT_WAVEFORM_POINTS; index++)
    {
        float position =
            start_sample +
            span_samples *
            (float)index /
            (float)(G_MEASUREMENT_WAVEFORM_POINTS - 1U);
        float value = GMeasurement_CubicInterpolate(samples, position) -
                      mean;
        int16_t rounded = GMeasurement_RoundToInt16(value);

        waveform->points[index] = rounded;
        if (rounded < waveform->minimum_code)
        {
            waveform->minimum_code = rounded;
        }
        if (rounded > waveform->maximum_code)
        {
            waveform->maximum_code = rounded;
        }
    }

    waveform->cycles = cycles;
    waveform->point_count = G_MEASUREMENT_WAVEFORM_POINTS;
    waveform->start_sample = start_sample;
    waveform->span_samples = span_samples;
    waveform->valid = 1U;
    return 1U;
}

static uint8_t GMeasurement_ValidateCalibration(
    const GMeasurementCalibration *calibration)
{
    uint8_t index;

    if ((calibration == NULL) ||
        (calibration->points == NULL) ||
        (calibration->point_count == 0U))
    {
        return 0U;
    }

    for (index = 0U; index < calibration->point_count; index++)
    {
        const GMeasurementCalibrationPoint *point =
            &calibration->points[index];

        if ((point->frequency_hz <= 0.0f) ||
            (point->mv_per_code <= 0.0f))
        {
            return 0U;
        }

        if ((index > 0U) &&
            (point->frequency_hz <=
             calibration->points[index - 1U].frequency_hz))
        {
            return 0U;
        }
    }

    return 1U;
}

static float GMeasurement_GetMvPerCode(
    const GMeasurementCalibration *calibration,
    float frequency_hz)
{
    uint8_t index;

    if ((calibration->point_count == 1U) ||
        (frequency_hz <= calibration->points[0].frequency_hz))
    {
        return calibration->points[0].mv_per_code;
    }

    for (index = 1U; index < calibration->point_count; index++)
    {
        const GMeasurementCalibrationPoint *left =
            &calibration->points[index - 1U];
        const GMeasurementCalibrationPoint *right =
            &calibration->points[index];

        if (frequency_hz <= right->frequency_hz)
        {
            float ratio =
                (frequency_hz - left->frequency_hz) /
                (right->frequency_hz - left->frequency_hz);

            return left->mv_per_code +
                   ratio * (right->mv_per_code - left->mv_per_code);
        }
    }

    return calibration->points[calibration->point_count - 1U].mv_per_code;
}

/*
 * Catmull-Rom四点三次插值。
 * 与原来的两点线性插值相比，高频正弦在每周期只有10~20个真实采样点时
 * 仍能保持连续斜率和圆滑峰谷；不改变原始采样点，也不引入额外滤波。
 */
static float GMeasurement_CubicInterpolate(const int16_t *samples,
                                           float position)
{
    uint16_t left = (uint16_t)position;
    float fraction = position - (float)left;
    float p0 = (float)samples[left - 1U];
    float p1 = (float)samples[left];
    float p2 = (float)samples[left + 1U];
    float p3 = (float)samples[left + 2U];
    float fraction2 = fraction * fraction;
    float fraction3 = fraction2 * fraction;

    return 0.5f *
           ((2.0f * p1) +
            (-p0 + p2) * fraction +
            (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * fraction2 +
            (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * fraction3);
}

static int16_t GMeasurement_RoundToInt16(float value)
{
    if (value >= 32767.0f)
    {
        return 32767;
    }
    if (value <= -32768.0f)
    {
        return (int16_t)-32768;
    }

    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}
