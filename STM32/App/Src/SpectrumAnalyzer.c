#include "SpectrumAnalyzer.h"

#include "arm_const_structs.h"
#include "arm_math.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define SPECTRUM_PI                 3.14159265358979323846f
#define SPECTRUM_TWO_PI             (2.0f * SPECTRUM_PI)
#define SPECTRUM_MIN_FREQUENCY_HZ   10000.0f
#define SPECTRUM_MAX_FREQUENCY_HZ   500000.0f
#define SPECTRUM_PEAK_CAPACITY      12U
#define SPECTRUM_MAX_HARMONIC_ORDER 50U
#define SPECTRUM_MATCH_TOLERANCE_BIN 2.5f
#define SPECTRUM_NOISE_POWER_FACTOR 4.0f
#define SPECTRUM_RECONSTRUCT_POINTS 2048U
#define SPECTRUM_MAX_LS_COLUMNS     7U

typedef struct
{
    uint16_t bin;
    float power;
    float interpolated_bin;
    float frequency_hz;
} SpectrumPeak;

static float s_FftInput[SPECTRUM_FRAME_LENGTH];
static float s_FftOutput[SPECTRUM_FRAME_LENGTH];
static int16_t s_HannWindowQ15[SPECTRUM_FRAME_LENGTH];
static uint8_t s_WindowReady;
static uint8_t s_FftReady;
static uint8_t s_LastSpectrumReady;
static arm_rfft_fast_instance_f32 s_RfftInstance;

static void SpectrumAnalyzer_InitWindow(void);
static float SpectrumAnalyzer_PowerAt(uint16_t bin);
static float SpectrumAnalyzer_InterpolateBin(uint16_t bin);
static void SpectrumAnalyzer_InsertPeak(SpectrumPeak *peaks,
                                        uint8_t *peak_count,
                                        uint16_t bin,
                                        float power);
static int32_t SpectrumAnalyzer_FindNearestPeak(const SpectrumPeak *peaks,
                                                uint8_t peak_count,
                                                float target_hz,
                                                float tolerance_hz,
                                                float threshold);
static uint8_t SpectrumAnalyzer_SelectHarmonics(const SpectrumPeak *peaks,
                                                uint8_t peak_count,
                                                float noise_power,
                                                uint8_t *harmonics,
                                                float *fundamental_hz);
static uint8_t SpectrumAnalyzer_FitComponents(const int16_t *samples,
                                              const uint8_t *harmonics,
                                              uint8_t component_count,
                                              float fundamental_hz,
                                              SpectrumResult *result);
static void SpectrumAnalyzer_FindStrongestUnmatched(
    const SpectrumPeak *peaks,
    uint8_t peak_count,
    float noise_power,
    const uint8_t *harmonics,
    uint8_t component_count,
    float fundamental_hz,
    SpectrumResult *result);
static uint8_t SpectrumAnalyzer_SolveLinear(float matrix[SPECTRUM_MAX_LS_COLUMNS][SPECTRUM_MAX_LS_COLUMNS],
                                            float vector[SPECTRUM_MAX_LS_COLUMNS],
                                            float solution[SPECTRUM_MAX_LS_COLUMNS],
                                            uint8_t size);
static void SpectrumAnalyzer_ComputeMetrics(SpectrumResult *result);

uint8_t SpectrumAnalyzer_Run(const int16_t *samples, SpectrumResult *result)
{
    SpectrumPeak peaks[SPECTRUM_PEAK_CAPACITY];
    uint8_t harmonics[SPECTRUM_MAX_COMPONENTS];
    uint8_t peak_count = 0U;
    uint8_t component_count;
    uint16_t first_bin;
    uint16_t last_bin;
    uint16_t bin;
    float mean = 0.0f;
    float noise_power = 0.0f;
    float fundamental_hz = 0.0f;
    float strongest_amplitude = 0.0f;

    if ((samples == NULL) || (result == NULL))
    {
        return 0U;
    }

    s_LastSpectrumReady = 0U;
    memset(result, 0, sizeof(*result));
    memset(peaks, 0, sizeof(peaks));
    SpectrumAnalyzer_InitWindow();

    if (s_FftReady == 0U)
    {
        if (arm_rfft_fast_init_f32(&s_RfftInstance,
                                   SPECTRUM_FRAME_LENGTH) != ARM_MATH_SUCCESS)
        {
            return 0U;
        }
        s_FftReady = 1U;
    }

    for (bin = 0U; bin < SPECTRUM_FRAME_LENGTH; bin++)
    {
        mean += (float)samples[bin];
    }
    mean /= (float)SPECTRUM_FRAME_LENGTH;

    for (bin = 0U; bin < SPECTRUM_FRAME_LENGTH; bin++)
    {
        float centered = (float)samples[bin] - mean;
        s_FftInput[bin] = centered * ((float)s_HannWindowQ15[bin] / 32768.0f);
    }

    arm_rfft_fast_f32(&s_RfftInstance,
                      s_FftInput,
                      s_FftOutput,
                      0U);
    s_LastSpectrumReady = 1U;

    first_bin = (uint16_t)ceilf(SPECTRUM_MIN_FREQUENCY_HZ *
                                (float)SPECTRUM_FRAME_LENGTH /
                                SPECTRUM_SAMPLE_RATE_HZ);
    last_bin = (uint16_t)floorf(SPECTRUM_MAX_FREQUENCY_HZ *
                                (float)SPECTRUM_FRAME_LENGTH /
                                SPECTRUM_SAMPLE_RATE_HZ);

    for (bin = first_bin; bin <= last_bin; bin++)
    {
        noise_power += SpectrumAnalyzer_PowerAt(bin);
    }
    noise_power /= (float)(last_bin - first_bin + 1U);

    for (bin = first_bin; bin <= last_bin; bin++)
    {
        float left = SpectrumAnalyzer_PowerAt((uint16_t)(bin - 1U));
        float center = SpectrumAnalyzer_PowerAt(bin);
        float right = SpectrumAnalyzer_PowerAt((uint16_t)(bin + 1U));

        if ((center > left) && (center >= right))
        {
            SpectrumAnalyzer_InsertPeak(peaks, &peak_count, bin, center);
        }
    }

    if (peak_count == 0U)
    {
        result->status = SPECTRUM_STATUS_NO_SIGNAL;
        return 0U;
    }

    for (bin = 0U; bin < peak_count; bin++)
    {
        peaks[bin].interpolated_bin =
            SpectrumAnalyzer_InterpolateBin(peaks[bin].bin);
        peaks[bin].frequency_hz =
            peaks[bin].interpolated_bin *
            SPECTRUM_SAMPLE_RATE_HZ /
            (float)SPECTRUM_FRAME_LENGTH;
    }

    component_count = SpectrumAnalyzer_SelectHarmonics(peaks,
                                                       peak_count,
                                                       noise_power,
                                                       harmonics,
                                                       &fundamental_hz);
    if (component_count == 0U)
    {
        result->status = SPECTRUM_STATUS_NO_SIGNAL;
        return 0U;
    }

    fundamental_hz = roundf(fundamental_hz / 500.0f) * 500.0f;
    {
        float tol = SPECTRUM_MATCH_TOLERANCE_BIN *
                    SPECTRUM_SAMPLE_RATE_HZ /
                    (float)SPECTRUM_FRAME_LENGTH;
        float thresh = noise_power * SPECTRUM_NOISE_POWER_FACTOR;
        uint8_t h;
        uint8_t new_count = 0U;

        if (thresh < peaks[0].power * 1.0e-8f)
            thresh = peaks[0].power * 1.0e-8f;

        for (h = 1U;
             h <= SPECTRUM_MAX_HARMONIC_ORDER &&
             new_count < SPECTRUM_MAX_COMPONENTS;
             h++)
        {
            float target = fundamental_hz * (float)h;

            if (target > SPECTRUM_MAX_FREQUENCY_HZ + tol)
                break;

            if (SpectrumAnalyzer_FindNearestPeak(
                    peaks, peak_count, target, tol, thresh) >= 0)
                harmonics[new_count++] = h;
        }

        if (new_count > 0U)
            component_count = new_count;
    }

    result->fundamental_hz = fundamental_hz;
    result->component_count = component_count;

    if (SpectrumAnalyzer_FitComponents(samples,
                                       harmonics,
                                       component_count,
                                       fundamental_hz,
                                       result) == 0U)
    {
        memset(result, 0, sizeof(*result));
        return 0U;
    }

    for (bin = 0U; bin < result->component_count; bin++)
    {
        if (result->components[bin].amplitude_codes > strongest_amplitude)
        {
            strongest_amplitude =
                result->components[bin].amplitude_codes;
        }
    }

    /*
     * 实机空输入底噪拟合幅值约为1 code。低于16 codes时不报告随机基频，
     * 同时为题目最小50 mVpp输入保留充足余量；该门限还需随实机标定复核。
     */
    if (strongest_amplitude < SPECTRUM_MIN_VALID_AMPLITUDE_CODES)
    {
        memset(result, 0, sizeof(*result));
        result->status = SPECTRUM_STATUS_NO_SIGNAL;
        return 0U;
    }

    SpectrumAnalyzer_FindStrongestUnmatched(peaks,
                                            peak_count,
                                            noise_power,
                                            harmonics,
                                            component_count,
                                            fundamental_hz,
                                            result);
    SpectrumAnalyzer_ComputeMetrics(result);
    result->valid = 1U;
    result->status = SPECTRUM_STATUS_VALID;
    return 1U;
}

uint8_t SpectrumAnalyzer_BuildDisplay(uint8_t *points, uint16_t point_count)
{
    uint16_t first_bin;
    uint16_t last_bin;
    uint16_t point;
    uint32_t bin_span;
    float maximum_power = 0.0f;

    if ((points == NULL) || (point_count == 0U) ||
        (s_LastSpectrumReady == 0U))
    {
        return 0U;
    }

    first_bin = (uint16_t)ceilf(SPECTRUM_MIN_FREQUENCY_HZ *
                                (float)SPECTRUM_FRAME_LENGTH /
                                SPECTRUM_SAMPLE_RATE_HZ);
    last_bin = (uint16_t)floorf(SPECTRUM_MAX_FREQUENCY_HZ *
                                (float)SPECTRUM_FRAME_LENGTH /
                                SPECTRUM_SAMPLE_RATE_HZ);
    bin_span = (uint32_t)last_bin - (uint32_t)first_bin + 1UL;

    for (point = 0U; point < point_count; point++)
    {
        uint16_t start_bin = (uint16_t)(
            (uint32_t)first_bin +
            (bin_span * (uint32_t)point) / (uint32_t)point_count);
        uint16_t end_bin = (uint16_t)(
            (uint32_t)first_bin +
            (bin_span * (uint32_t)(point + 1U)) /
                (uint32_t)point_count);
        uint16_t bin;
        float peak_power = 0.0f;

        if (end_bin <= start_bin)
        {
            end_bin = (uint16_t)(start_bin + 1U);
        }
        if (end_bin > (uint16_t)(last_bin + 1U))
        {
            end_bin = (uint16_t)(last_bin + 1U);
        }

        for (bin = start_bin; bin < end_bin; bin++)
        {
            float power = SpectrumAnalyzer_PowerAt(bin);
            if (power > peak_power)
            {
                peak_power = power;
            }
        }

        if (peak_power > maximum_power)
        {
            maximum_power = peak_power;
        }
    }

    if (maximum_power <= 0.0f)
    {
        memset(points, 0, point_count);
        return 1U;
    }

    for (point = 0U; point < point_count; point++)
    {
        uint16_t start_bin = (uint16_t)(
            (uint32_t)first_bin +
            (bin_span * (uint32_t)point) / (uint32_t)point_count);
        uint16_t end_bin = (uint16_t)(
            (uint32_t)first_bin +
            (bin_span * (uint32_t)(point + 1U)) /
                (uint32_t)point_count);
        uint16_t bin;
        float peak_power = 0.0f;
        float ratio;
        float decibels;
        float display_value;

        if (end_bin <= start_bin)
        {
            end_bin = (uint16_t)(start_bin + 1U);
        }
        if (end_bin > (uint16_t)(last_bin + 1U))
        {
            end_bin = (uint16_t)(last_bin + 1U);
        }

        for (bin = start_bin; bin < end_bin; bin++)
        {
            float power = SpectrumAnalyzer_PowerAt(bin);
            if (power > peak_power)
            {
                peak_power = power;
            }
        }

        ratio = peak_power / maximum_power;
        if (ratio <= 0.000001f)
        {
            points[point] = 0U;
            continue;
        }

        decibels = 10.0f * log10f(ratio);
        if (decibels < -60.0f)
        {
            decibels = -60.0f;
        }

        display_value = (decibels + 60.0f) * (254.0f / 60.0f);
        if (display_value < 0.0f)
        {
            display_value = 0.0f;
        }
        if (display_value > 254.0f)
        {
            display_value = 254.0f;
        }
        points[point] = (uint8_t)(display_value + 0.5f);
    }

    return 1U;
}

static void SpectrumAnalyzer_InitWindow(void)
{
    uint16_t index;

    if (s_WindowReady != 0U)
    {
        return;
    }

    for (index = 0U; index < SPECTRUM_FRAME_LENGTH; index++)
    {
        float phase = SPECTRUM_TWO_PI *
                      (float)index /
                      (float)(SPECTRUM_FRAME_LENGTH - 1U);
        float window = 0.5f - 0.5f * cosf(phase);
        s_HannWindowQ15[index] = (int16_t)(window * 32767.0f + 0.5f);
    }

    s_WindowReady = 1U;
}

static float SpectrumAnalyzer_PowerAt(uint16_t bin)
{
    float real_value;
    float imaginary_value;

    if (bin == 0U)
    {
        return s_FftOutput[0] * s_FftOutput[0];
    }

    if (bin == (SPECTRUM_FRAME_LENGTH / 2U))
    {
        return s_FftOutput[1] * s_FftOutput[1];
    }

    real_value = s_FftOutput[2U * bin];
    imaginary_value = s_FftOutput[2U * bin + 1U];
    return real_value * real_value + imaginary_value * imaginary_value;
}

static float SpectrumAnalyzer_InterpolateBin(uint16_t bin)
{
    float left = SpectrumAnalyzer_PowerAt((uint16_t)(bin - 1U));
    float center = SpectrumAnalyzer_PowerAt(bin);
    float right = SpectrumAnalyzer_PowerAt((uint16_t)(bin + 1U));
    float delta = 0.0f;

    if ((left > 0.0f) && (center > 0.0f) && (right > 0.0f))
    {
        float log_left = logf(left);
        float log_center = logf(center);
        float log_right = logf(right);
        float denominator =
            log_left - 2.0f * log_center + log_right;

        if (fabsf(denominator) > 1.0e-20f)
        {
            delta = 0.5f * (log_left - log_right) / denominator;
            if (delta > 0.5f)
                delta = 0.5f;
            else if (delta < -0.5f)
                delta = -0.5f;
        }
    }

    return (float)bin + delta;
}

static void SpectrumAnalyzer_InsertPeak(SpectrumPeak *peaks,
                                        uint8_t *peak_count,
                                        uint16_t bin,
                                        float power)
{
    uint8_t insert_at = 0U;
    uint8_t move_index;

    while ((insert_at < *peak_count) &&
           (peaks[insert_at].power >= power))
    {
        insert_at++;
    }

    if (insert_at >= SPECTRUM_PEAK_CAPACITY)
    {
        return;
    }

    if (*peak_count < SPECTRUM_PEAK_CAPACITY)
    {
        (*peak_count)++;
    }

    move_index = (uint8_t)(*peak_count - 1U);
    while (move_index > insert_at)
    {
        peaks[move_index] = peaks[move_index - 1U];
        move_index--;
    }

    peaks[insert_at].bin = bin;
    peaks[insert_at].power = power;
    peaks[insert_at].interpolated_bin = (float)bin;
    peaks[insert_at].frequency_hz = 0.0f;
}

static int32_t SpectrumAnalyzer_FindNearestPeak(const SpectrumPeak *peaks,
                                                uint8_t peak_count,
                                                float target_hz,
                                                float tolerance_hz,
                                                float threshold)
{
    int32_t best_index = -1;
    float best_error = tolerance_hz;
    uint8_t index;

    for (index = 0U; index < peak_count; index++)
    {
        float error;

        if (peaks[index].power < threshold)
        {
            continue;
        }

        error = fabsf(peaks[index].frequency_hz - target_hz);
        if (error <= best_error)
        {
            best_error = error;
            best_index = (int32_t)index;
        }
    }

    return best_index;
}

static uint8_t SpectrumAnalyzer_SelectHarmonics(const SpectrumPeak *peaks,
                                                uint8_t peak_count,
                                                float noise_power,
                                                uint8_t *harmonics,
                                                float *fundamental_hz)
{
    float bin_width = SPECTRUM_SAMPLE_RATE_HZ /
                      (float)SPECTRUM_FRAME_LENGTH;
    float tolerance_hz = SPECTRUM_MATCH_TOLERANCE_BIN * bin_width;
    float threshold = noise_power * SPECTRUM_NOISE_POWER_FACTOR;
    uint8_t best_candidate = 0U;
    uint8_t best_match_count = 0U;
    float best_score = -1.0f;
    uint8_t candidate;
    uint8_t harmonic;
    uint8_t output_count = 0U;
    float weighted_fundamental = 0.0f;
    float total_weight = 0.0f;

    if (threshold < peaks[0].power * 1.0e-8f)
    {
        threshold = peaks[0].power * 1.0e-8f;
    }

    for (candidate = 0U; candidate < peak_count; candidate++)
    {
        uint8_t match_count = 0U;
        float score = 0.0f;

        if (peaks[candidate].power < threshold)
        {
            continue;
        }

        for (harmonic = 1U;
             harmonic <= SPECTRUM_MAX_HARMONIC_ORDER;
             harmonic++)
        {
            float target = peaks[candidate].frequency_hz * (float)harmonic;
            int32_t match;

            if (target > SPECTRUM_MAX_FREQUENCY_HZ + tolerance_hz)
            {
                break;
            }

            match = SpectrumAnalyzer_FindNearestPeak(peaks,
                                                     peak_count,
                                                     target,
                                                     tolerance_hz,
                                                     threshold);
            if (match >= 0)
            {
                match_count++;
                score += peaks[match].power;
            }
        }

        if ((match_count > best_match_count) ||
            ((match_count == best_match_count) && (score > best_score)))
        {
            best_candidate = candidate;
            best_match_count = match_count;
            best_score = score;
        }
    }

    if (best_match_count == 0U)
    {
        return 0U;
    }

    for (harmonic = 1U;
         (harmonic <= SPECTRUM_MAX_HARMONIC_ORDER) &&
         (output_count < SPECTRUM_MAX_COMPONENTS);
         harmonic++)
    {
        float target = peaks[best_candidate].frequency_hz * (float)harmonic;
        int32_t match;

        if (target > SPECTRUM_MAX_FREQUENCY_HZ + tolerance_hz)
        {
            break;
        }

        match = SpectrumAnalyzer_FindNearestPeak(peaks,
                                                 peak_count,
                                                 target,
                                                 tolerance_hz,
                                                 threshold);
        if (match >= 0)
        {
            float weight = sqrtf(peaks[match].power);

            harmonics[output_count] = harmonic;
            output_count++;
            weighted_fundamental +=
                (peaks[match].frequency_hz / (float)harmonic) * weight;
            total_weight += weight;
        }
    }

    if ((output_count == 0U) || (total_weight <= 0.0f))
    {
        return 0U;
    }

    *fundamental_hz = weighted_fundamental / total_weight;
    return output_count;
}

static uint8_t SpectrumAnalyzer_FitComponents(const int16_t *samples,
                                              const uint8_t *harmonics,
                                              uint8_t component_count,
                                              float fundamental_hz,
                                              SpectrumResult *result)
{
    float normal[SPECTRUM_MAX_LS_COLUMNS][SPECTRUM_MAX_LS_COLUMNS];
    float rhs[SPECTRUM_MAX_LS_COLUMNS];
    float solution[SPECTRUM_MAX_LS_COLUMNS];
    float cos_state[SPECTRUM_MAX_COMPONENTS];
    float sin_state[SPECTRUM_MAX_COMPONENTS];
    float cos_step[SPECTRUM_MAX_COMPONENTS];
    float sin_step[SPECTRUM_MAX_COMPONENTS];
    float basis[SPECTRUM_MAX_LS_COLUMNS];
    uint8_t column_count = (uint8_t)(1U + 2U * component_count);
    uint8_t row;
    uint8_t column;
    uint8_t component;
    uint16_t sample_index;

    memset(normal, 0, sizeof(normal));
    memset(rhs, 0, sizeof(rhs));
    memset(solution, 0, sizeof(solution));

    for (component = 0U; component < component_count; component++)
    {
        float frequency = fundamental_hz * (float)harmonics[component];
        float omega = SPECTRUM_TWO_PI * frequency / SPECTRUM_SAMPLE_RATE_HZ;
        cos_state[component] = 1.0f;
        sin_state[component] = 0.0f;
        cos_step[component] = cosf(omega);
        sin_step[component] = sinf(omega);
    }

    for (sample_index = 0U; sample_index < SPECTRUM_FRAME_LENGTH; sample_index++)
    {
        float sample_value = (float)samples[sample_index];

        basis[0] = 1.0f;
        for (component = 0U; component < component_count; component++)
        {
            float next_cos;
            float next_sin;

            basis[1U + 2U * component] = cos_state[component];
            basis[2U + 2U * component] = sin_state[component];

            next_cos = cos_state[component] * cos_step[component] -
                       sin_state[component] * sin_step[component];
            next_sin = sin_state[component] * cos_step[component] +
                       cos_state[component] * sin_step[component];
            cos_state[component] = next_cos;
            sin_state[component] = next_sin;

            if ((sample_index & 0x00FFU) == 0x00FFU)
            {
                float norm = sqrtf(cos_state[component] * cos_state[component] +
                                   sin_state[component] * sin_state[component]);
                if (norm > 0.0f)
                {
                    cos_state[component] /= norm;
                    sin_state[component] /= norm;
                }
            }
        }

        for (row = 0U; row < column_count; row++)
        {
            rhs[row] += basis[row] * sample_value;
            for (column = 0U; column < column_count; column++)
            {
                normal[row][column] += basis[row] * basis[column];
            }
        }
    }

    if (SpectrumAnalyzer_SolveLinear(normal,
                                     rhs,
                                     solution,
                                     column_count) == 0U)
    {
        return 0U;
    }

    for (component = 0U; component < component_count; component++)
    {
        float cosine_coefficient = solution[1U + 2U * component];
        float sine_coefficient = solution[2U + 2U * component];
        SpectrumComponent *output = &result->components[component];

        output->harmonic = harmonics[component];
        output->frequency_hz = fundamental_hz * (float)harmonics[component];
        output->amplitude_codes = sqrtf(cosine_coefficient * cosine_coefficient +
                                        sine_coefficient * sine_coefficient);
        output->phase_rad = atan2f(-sine_coefficient, cosine_coefficient);
    }

    return 1U;
}

static void SpectrumAnalyzer_FindStrongestUnmatched(
    const SpectrumPeak *peaks,
    uint8_t peak_count,
    float noise_power,
    const uint8_t *harmonics,
    uint8_t component_count,
    float fundamental_hz,
    SpectrumResult *result)
{
    float bin_width = SPECTRUM_SAMPLE_RATE_HZ /
                      (float)SPECTRUM_FRAME_LENGTH;
    float tolerance_hz = SPECTRUM_MATCH_TOLERANCE_BIN * bin_width;
    float threshold = noise_power * SPECTRUM_NOISE_POWER_FACTOR;
    float reference_power = 0.0f;
    float reference_amplitude = 0.0f;
    uint8_t component;
    uint8_t peak_index;

    if (threshold < peaks[0].power * 1.0e-8f)
    {
        threshold = peaks[0].power * 1.0e-8f;
    }

    for (component = 0U; component < component_count; component++)
    {
        float target = fundamental_hz * (float)harmonics[component];
        int32_t match = SpectrumAnalyzer_FindNearestPeak(peaks,
                                                         peak_count,
                                                         target,
                                                         tolerance_hz,
                                                         0.0f);

        if ((match >= 0) && (peaks[match].power > reference_power))
        {
            reference_power = peaks[match].power;
            reference_amplitude =
                result->components[component].amplitude_codes;
        }
    }

    for (peak_index = 0U; peak_index < peak_count; peak_index++)
    {
        uint8_t matched = 0U;

        if (peaks[peak_index].power < threshold)
        {
            break;
        }

        for (component = 0U; component < component_count; component++)
        {
            float target =
                fundamental_hz * (float)harmonics[component];

            if (fabsf(peaks[peak_index].frequency_hz - target) <=
                tolerance_hz)
            {
                matched = 1U;
                break;
            }
        }

        if (matched == 0U)
        {
            result->spur_valid = 1U;
            result->spur_frequency_hz =
                peaks[peak_index].frequency_hz;

            if ((reference_power > 0.0f) &&
                (reference_amplitude > 0.0f))
            {
                result->spur_amplitude_codes =
                    reference_amplitude *
                    sqrtf(peaks[peak_index].power /
                          reference_power);
            }
            return;
        }
    }
}

static uint8_t SpectrumAnalyzer_SolveLinear(float matrix[SPECTRUM_MAX_LS_COLUMNS][SPECTRUM_MAX_LS_COLUMNS],
                                            float vector[SPECTRUM_MAX_LS_COLUMNS],
                                            float solution[SPECTRUM_MAX_LS_COLUMNS],
                                            uint8_t size)
{
    uint8_t pivot;

    for (pivot = 0U; pivot < size; pivot++)
    {
        uint8_t best_row = pivot;
        uint8_t row;
        uint8_t column;
        float best_value = fabsf(matrix[pivot][pivot]);

        for (row = (uint8_t)(pivot + 1U); row < size; row++)
        {
            float candidate = fabsf(matrix[row][pivot]);
            if (candidate > best_value)
            {
                best_value = candidate;
                best_row = row;
            }
        }

        if (best_value < 1.0e-9f)
        {
            return 0U;
        }

        if (best_row != pivot)
        {
            float temporary;

            for (column = pivot; column < size; column++)
            {
                temporary = matrix[pivot][column];
                matrix[pivot][column] = matrix[best_row][column];
                matrix[best_row][column] = temporary;
            }

            temporary = vector[pivot];
            vector[pivot] = vector[best_row];
            vector[best_row] = temporary;
        }

        for (row = (uint8_t)(pivot + 1U); row < size; row++)
        {
            float factor = matrix[row][pivot] / matrix[pivot][pivot];

            matrix[row][pivot] = 0.0f;
            for (column = (uint8_t)(pivot + 1U); column < size; column++)
            {
                matrix[row][column] -= factor * matrix[pivot][column];
            }
            vector[row] -= factor * vector[pivot];
        }
    }

    pivot = size;
    while (pivot > 0U)
    {
        uint8_t row = (uint8_t)(pivot - 1U);
        uint8_t column;
        float value = vector[row];

        for (column = (uint8_t)(row + 1U); column < size; column++)
        {
            value -= matrix[row][column] * solution[column];
        }

        solution[row] = value / matrix[row][row];
        pivot--;
    }

    return 1U;
}

static void SpectrumAnalyzer_ComputeMetrics(SpectrumResult *result)
{
    float sum_square = 0.0f;
    float minimum = 0.0f;
    float maximum = 0.0f;
    uint16_t point;
    uint8_t component;

    for (component = 0U; component < result->component_count; component++)
    {
        float amplitude = result->components[component].amplitude_codes;
        sum_square += amplitude * amplitude;
    }
    result->rms_codes = sqrtf(0.5f * sum_square);

    for (point = 0U; point < SPECTRUM_RECONSTRUCT_POINTS; point++)
    {
        float base_phase = SPECTRUM_TWO_PI *
                           (float)point /
                           (float)SPECTRUM_RECONSTRUCT_POINTS;
        float value = 0.0f;

        for (component = 0U; component < result->component_count; component++)
        {
            const SpectrumComponent *item = &result->components[component];
            value += item->amplitude_codes *
                     cosf((float)item->harmonic * base_phase + item->phase_rad);
        }

        if (point == 0U)
        {
            minimum = value;
            maximum = value;
        }
        else
        {
            if (value < minimum)
                minimum = value;
            if (value > maximum)
                maximum = value;
        }
    }

    result->vpp_codes = maximum - minimum;
}
