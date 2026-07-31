#include "GMeasurement.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define G_MEASUREMENT_TWO_PI              6.28318530717958647692f
#define G_MEASUREMENT_RECONSTRUCT_POINTS  2048U
#define G_MEASUREMENT_EXTREMUM_ITERATIONS  18U
#define G_MEASUREMENT_GOLDEN_RATIO         0.61803398874989484820f

static uint8_t GMeasurement_ValidateCalibration(
    const GMeasurementCalibration *calibration);
static float GMeasurement_GetMvPerCode(
    const GMeasurementCalibration *calibration,
    float frequency_hz);
static float GMeasurement_GetAmplitudeMv(
    const GMeasurementCalibration *calibration,
    float frequency_hz,
    float amplitude_codes);
static float GMeasurement_ConvertAmplitudeRow(
    const GMeasurementAmplitudeCalibrationRow *row,
    float amplitude_codes);
static float GMeasurement_GetPhaseErrorRad(
    const GMeasurementCalibration *calibration,
    float frequency_hz);
static float GMeasurement_EvaluateReconstruction(
    const GMeasurementResult *measurement,
    float base_phase);
static float GMeasurement_RefineExtremum(
    const GMeasurementResult *measurement,
    float center_phase,
    float half_width,
    uint8_t find_maximum);
static void GMeasurement_UpdateUpp(GMeasurementResult *measurement);
static float GMeasurement_WrapRadians(float phase_rad);
static float GMeasurement_CubicInterpolate(const int16_t *samples,
                                           float position);
static int16_t GMeasurement_RoundToInt16(float value);

uint8_t GMeasurement_Convert(const SpectrumResult *spectrum,
                             const GMeasurementCalibration *calibration,
                             GMeasurementResult *measurement)
{
    float sum_square = 0.0f;
    float fundamental_phase_error_rad;
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
    fundamental_phase_error_rad =
        GMeasurement_GetPhaseErrorRad(calibration,
                                      spectrum->fundamental_hz);

    for (component = 0U;
         component < spectrum->component_count;
         component++)
    {
        const SpectrumComponent *input = &spectrum->components[component];
        GMeasurementComponent *output = &measurement->components[component];

        output->harmonic = input->harmonic;
        output->frequency_hz = input->frequency_hz;
        output->amplitude_mv =
            GMeasurement_GetAmplitudeMv(calibration,
                                        input->frequency_hz,
                                        input->amplitude_codes);
        /*
         * 单次采集带有任意起始时刻，对第n次谐波表现为n倍公共相位。
         * Vpp不受该线性相位影响，因此只消除测量链的非线性相位响应：
         *   phase_error(f_n) - n * phase_error(f_0)
         * 未提供相位表时两个查询均返回0，行为与原程序完全一致。
         */
        output->phase_rad = GMeasurement_WrapRadians(
            input->phase_rad -
            (GMeasurement_GetPhaseErrorRad(calibration,
                                           input->frequency_hz) -
             (float)input->harmonic * fundamental_phase_error_rad));
        sum_square += output->amplitude_mv * output->amplitude_mv;
    }

    measurement->urms_mv = sqrtf(0.5f * sum_square);

    GMeasurement_UpdateUpp(measurement);

    if (spectrum->spur_valid != 0U)
    {
        measurement->spur_valid = 1U;
        measurement->spur_frequency_hz = spectrum->spur_frequency_hz;
        measurement->spur_amplitude_mv =
            GMeasurement_GetAmplitudeMv(calibration,
                                        spectrum->spur_frequency_hz,
                                        spectrum->spur_amplitude_codes);
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

    if (calibration->phase_point_count > 0U)
    {
        if (calibration->phase_points == NULL)
        {
            return 0U;
        }

        for (index = 0U;
             index < calibration->phase_point_count;
             index++)
        {
            const GMeasurementPhaseCalibrationPoint *point =
                &calibration->phase_points[index];

            if (point->frequency_hz <= 0.0f)
            {
                return 0U;
            }

            if ((index > 0U) &&
                (point->frequency_hz <=
                 calibration->phase_points[index - 1U].frequency_hz))
            {
                return 0U;
            }
        }
    }

    if (calibration->amplitude_row_count > 0U)
    {
        if (calibration->amplitude_rows == NULL)
        {
            return 0U;
        }

        for (index = 0U;
             index < calibration->amplitude_row_count;
             index++)
        {
            const GMeasurementAmplitudeCalibrationRow *row =
                &calibration->amplitude_rows[index];
            uint8_t level;

            if ((row->frequency_hz <= 0.0f) ||
                (row->level_count < 2U) ||
                (row->level_count > G_MEASUREMENT_MAX_AMPLITUDE_LEVELS))
            {
                return 0U;
            }
            if ((index > 0U) &&
                (row->frequency_hz <=
                 calibration->amplitude_rows[index - 1U].frequency_hz))
            {
                return 0U;
            }

            for (level = 0U; level < row->level_count; level++)
            {
                const GMeasurementAmplitudeCalibrationLevel *item =
                    &row->levels[level];

                if ((item->amplitude_codes <= 0.0f) ||
                    (item->peak_mv <= 0.0f))
                {
                    return 0U;
                }
                if ((level > 0U) &&
                    ((item->amplitude_codes <=
                      row->levels[level - 1U].amplitude_codes) ||
                     (item->peak_mv <= row->levels[level - 1U].peak_mv)))
                {
                    return 0U;
                }
            }
        }
    }

    return 1U;
}

void GMeasurement_QuantizeHalfMv(GMeasurementResult *measurement)
{
    GMeasurementResult quantized;
    uint8_t component;

    if ((measurement == NULL) ||
        (measurement->valid == 0U) ||
        (measurement->component_count == 0U) ||
        (measurement->component_count > SPECTRUM_MAX_COMPONENTS))
    {
        return;
    }

    quantized = *measurement;
    for (component = 0U;
         component < quantized.component_count;
         component++)
    {
        GMeasurementComponent *item =
            &quantized.components[component];

        item->amplitude_mv =
            roundf(item->amplitude_mv * 2.0f) * 0.5f;
    }

    GMeasurement_UpdateUpp(&quantized);
    measurement->upp_mv = quantized.upp_mv;
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

static float GMeasurement_GetAmplitudeMv(
    const GMeasurementCalibration *calibration,
    float frequency_hz,
    float amplitude_codes)
{
    uint8_t index;

    if ((calibration->amplitude_rows != NULL) &&
        (calibration->amplitude_row_count > 0U))
    {
        if ((calibration->amplitude_row_count == 1U) ||
            (frequency_hz <= calibration->amplitude_rows[0].frequency_hz))
        {
            return GMeasurement_ConvertAmplitudeRow(
                &calibration->amplitude_rows[0], amplitude_codes);
        }

        for (index = 1U;
             index < calibration->amplitude_row_count;
             index++)
        {
            const GMeasurementAmplitudeCalibrationRow *left =
                &calibration->amplitude_rows[index - 1U];
            const GMeasurementAmplitudeCalibrationRow *right =
                &calibration->amplitude_rows[index];

            if (frequency_hz <= right->frequency_hz)
            {
                float ratio =
                    (frequency_hz - left->frequency_hz) /
                    (right->frequency_hz - left->frequency_hz);
                float left_mv =
                    GMeasurement_ConvertAmplitudeRow(left, amplitude_codes);
                float right_mv =
                    GMeasurement_ConvertAmplitudeRow(right, amplitude_codes);

                return left_mv + ratio * (right_mv - left_mv);
            }
        }

        return GMeasurement_ConvertAmplitudeRow(
            &calibration->amplitude_rows[
                calibration->amplitude_row_count - 1U],
            amplitude_codes);
    }

#if G_MEASUREMENT_ENABLE_50_OHM_SCALE
    return amplitude_codes *
           GMeasurement_GetMvPerCode(calibration, frequency_hz) *
           G_MEASUREMENT_50_OHM_AMPLITUDE_SCALE;
#else
    return amplitude_codes *
           GMeasurement_GetMvPerCode(calibration, frequency_hz);
#endif
}

static float GMeasurement_ConvertAmplitudeRow(
    const GMeasurementAmplitudeCalibrationRow *row,
    float amplitude_codes)
{
    uint8_t level;

    if (amplitude_codes <= row->levels[0].amplitude_codes)
    {
        return amplitude_codes *
               row->levels[0].peak_mv /
               row->levels[0].amplitude_codes;
    }

    for (level = 1U; level < row->level_count; level++)
    {
        const GMeasurementAmplitudeCalibrationLevel *left =
            &row->levels[level - 1U];
        const GMeasurementAmplitudeCalibrationLevel *right =
            &row->levels[level];

        if (amplitude_codes <= right->amplitude_codes)
        {
            float ratio =
                (amplitude_codes - left->amplitude_codes) /
                (right->amplitude_codes - left->amplitude_codes);

            return left->peak_mv +
                   ratio * (right->peak_mv - left->peak_mv);
        }
    }

    {
        const GMeasurementAmplitudeCalibrationLevel *left =
            &row->levels[row->level_count - 2U];
        const GMeasurementAmplitudeCalibrationLevel *right =
            &row->levels[row->level_count - 1U];
        float ratio =
            (amplitude_codes - left->amplitude_codes) /
            (right->amplitude_codes - left->amplitude_codes);

        return left->peak_mv +
               ratio * (right->peak_mv - left->peak_mv);
    }
}

static float GMeasurement_GetPhaseErrorRad(
    const GMeasurementCalibration *calibration,
    float frequency_hz)
{
    uint8_t index;

    if ((calibration->phase_points == NULL) ||
        (calibration->phase_point_count == 0U))
    {
        return 0.0f;
    }

    if ((calibration->phase_point_count == 1U) ||
        (frequency_hz <= calibration->phase_points[0].frequency_hz))
    {
        return calibration->phase_points[0].phase_error_rad;
    }

    for (index = 1U;
         index < calibration->phase_point_count;
         index++)
    {
        const GMeasurementPhaseCalibrationPoint *left =
            &calibration->phase_points[index - 1U];
        const GMeasurementPhaseCalibrationPoint *right =
            &calibration->phase_points[index];

        if (frequency_hz <= right->frequency_hz)
        {
            float ratio =
                (frequency_hz - left->frequency_hz) /
                (right->frequency_hz - left->frequency_hz);

            return left->phase_error_rad +
                   ratio *
                       (right->phase_error_rad - left->phase_error_rad);
        }
    }

    return calibration->phase_points[
        calibration->phase_point_count - 1U].phase_error_rad;
}

static float GMeasurement_EvaluateReconstruction(
    const GMeasurementResult *measurement,
    float base_phase)
{
    float value = 0.0f;
    uint8_t component;

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

    return value;
}

static void GMeasurement_UpdateUpp(GMeasurementResult *measurement)
{
    float minimum = 0.0f;
    float maximum = 0.0f;
    uint16_t minimum_point = 0U;
    uint16_t maximum_point = 0U;
    uint16_t point;

    for (point = 0U; point < G_MEASUREMENT_RECONSTRUCT_POINTS; point++)
    {
        float base_phase =
            G_MEASUREMENT_TWO_PI *
            (float)point /
            (float)G_MEASUREMENT_RECONSTRUCT_POINTS;
        float value =
            GMeasurement_EvaluateReconstruction(measurement, base_phase);

        if (point == 0U)
        {
            minimum = value;
            maximum = value;
            minimum_point = point;
            maximum_point = point;
        }
        else
        {
            if (value < minimum)
            {
                minimum = value;
                minimum_point = point;
            }
            if (value > maximum)
            {
                maximum = value;
                maximum_point = point;
            }
        }
    }

    /*
     * 2048点只负责可靠地找到全局峰谷所在的小区间；再在相邻两个粗采样
     * 间隔内做黄金分割细化。这里只重建最多三个已识别分量，不改变
     * FPGA滤波器、FFT、频率或相位。
     */
    {
        const float phase_step =
            G_MEASUREMENT_TWO_PI /
            (float)G_MEASUREMENT_RECONSTRUCT_POINTS;

        maximum = GMeasurement_RefineExtremum(
            measurement,
            phase_step * (float)maximum_point,
            phase_step,
            1U);
        minimum = GMeasurement_RefineExtremum(
            measurement,
            phase_step * (float)minimum_point,
            phase_step,
            0U);
    }

    measurement->upp_mv = maximum - minimum;
}

static float GMeasurement_RefineExtremum(
    const GMeasurementResult *measurement,
    float center_phase,
    float half_width,
    uint8_t find_maximum)
{
    float left = center_phase - half_width;
    float right = center_phase + half_width;
    float x1 = right - G_MEASUREMENT_GOLDEN_RATIO * (right - left);
    float x2 = left + G_MEASUREMENT_GOLDEN_RATIO * (right - left);
    float y1 = GMeasurement_EvaluateReconstruction(measurement, x1);
    float y2 = GMeasurement_EvaluateReconstruction(measurement, x2);
    uint8_t iteration;

    for (iteration = 0U;
         iteration < G_MEASUREMENT_EXTREMUM_ITERATIONS;
         iteration++)
    {
        uint8_t keep_right =
            (find_maximum != 0U) ? (uint8_t)(y2 > y1) :
                                   (uint8_t)(y2 < y1);

        if (keep_right != 0U)
        {
            left = x1;
            x1 = x2;
            y1 = y2;
            x2 = left +
                 G_MEASUREMENT_GOLDEN_RATIO * (right - left);
            y2 = GMeasurement_EvaluateReconstruction(measurement, x2);
        }
        else
        {
            right = x2;
            x2 = x1;
            y2 = y1;
            x1 = right -
                 G_MEASUREMENT_GOLDEN_RATIO * (right - left);
            y1 = GMeasurement_EvaluateReconstruction(measurement, x1);
        }
    }

    return (find_maximum != 0U) ?
           ((y1 > y2) ? y1 : y2) :
           ((y1 < y2) ? y1 : y2);
}

static float GMeasurement_WrapRadians(float phase_rad)
{
    while (phase_rad <= -0.5f * G_MEASUREMENT_TWO_PI)
    {
        phase_rad += G_MEASUREMENT_TWO_PI;
    }
    while (phase_rad > 0.5f * G_MEASUREMENT_TWO_PI)
    {
        phase_rad -= G_MEASUREMENT_TWO_PI;
    }

    return phase_rad;
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
