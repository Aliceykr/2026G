#include "GMeasurement.h"
#include "GMeasurementCalibration.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_PI 3.14159265358979323846
#define TEST_FRAME_LENGTH 4096U
#define TEST_RECONSTRUCT_POINTS 2048U

static int16_t s_FrameA[TEST_FRAME_LENGTH];
static int16_t s_FrameB[TEST_FRAME_LENGTH];

static int NearlyEqual(double actual, double expected, double tolerance)
{
    return fabs(actual - expected) <= tolerance;
}

static double ReconstructExpectedUpp(const SpectrumResult *spectrum,
                                     double mv_per_code)
{
    double minimum = 0.0;
    double maximum = 0.0;
    unsigned int point;
    uint8_t component;

    for (point = 0U; point < 65536U; point++)
    {
        double phase = 2.0 * TEST_PI * (double)point / 65536.0;
        double value = 0.0;

        for (component = 0U;
             component < spectrum->component_count;
             component++)
        {
            const SpectrumComponent *item =
                &spectrum->components[component];

            value += (double)item->amplitude_codes * mv_per_code *
                     (double)G_MEASUREMENT_50_OHM_AMPLITUDE_SCALE *
                     cos((double)item->harmonic * phase +
                         (double)item->phase_rad);
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

    return maximum - minimum;
}

static double ReconstructComponentUpp(const double *amplitudes,
                                      const unsigned int *harmonics,
                                      const double *phases,
                                      unsigned int component_count)
{
    const unsigned int point_count = 262144U;
    double minimum = 0.0;
    double maximum = 0.0;
    unsigned int point;

    for (point = 0U; point < point_count; point++)
    {
        double theta =
            2.0 * TEST_PI * (double)point / (double)point_count;
        double value = 0.0;
        unsigned int component;

        for (component = 0U;
             component < component_count;
             component++)
        {
            value += amplitudes[component] *
                     cos((double)harmonics[component] * theta +
                         phases[component]);
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

    return maximum - minimum;
}

static int TestMeasurementConversion(void)
{
    static const GMeasurementCalibrationPoint calibration_points[] =
    {
        {  10000.0f, 0.0100f },
        { 500000.0f, 0.0200f }
    };
    static const GMeasurementCalibration calibration =
    {
        calibration_points,
        2U,
        NULL,
        0U,
        NULL,
        0U
    };
    SpectrumResult spectrum;
    GMeasurementResult measurement;
    double expected_rms;
    double expected_upp;

    memset(&spectrum, 0, sizeof(spectrum));
    spectrum.valid = 1U;
    spectrum.fundamental_hz = 100000.0f;
    spectrum.component_count = 3U;
    spectrum.components[0].harmonic = 1U;
    spectrum.components[0].frequency_hz = 100000.0f;
    spectrum.components[0].amplitude_codes = 1000.0f;
    spectrum.components[0].phase_rad = 0.2f;
    spectrum.components[1].harmonic = 3U;
    spectrum.components[1].frequency_hz = 300000.0f;
    spectrum.components[1].amplitude_codes = 400.0f;
    spectrum.components[1].phase_rad = -0.7f;
    spectrum.components[2].harmonic = 4U;
    spectrum.components[2].frequency_hz = 400000.0f;
    spectrum.components[2].amplitude_codes = 200.0f;
    spectrum.components[2].phase_rad = 1.1f;

    if (GMeasurement_Convert(&spectrum,
                             &calibration,
                             &measurement) == 0U)
    {
        printf("FAIL conversion returned invalid\n");
        return 0;
    }

    /*
     * 100 kHz 插值得 0.0118367347 mV/code；
     * 300 kHz 插值得 0.0159183673；400 kHz 插值得 0.0179591837。
     */
    if (!NearlyEqual(measurement.components[0].amplitude_mv,
                     5.91836735,
                     0.002) ||
        !NearlyEqual(measurement.components[1].amplitude_mv,
                     3.18367345,
                     0.002) ||
        !NearlyEqual(measurement.components[2].amplitude_mv,
                     1.79591835,
                     0.002))
    {
        printf("FAIL conversion interpolation %.5f %.5f %.5f\n",
               (double)measurement.components[0].amplitude_mv,
               (double)measurement.components[1].amplitude_mv,
               (double)measurement.components[2].amplitude_mv);
        return 0;
    }

    expected_rms =
        sqrt((5.91836735 * 5.91836735 +
              3.18367345 * 3.18367345 +
              1.79591835 * 1.79591835) / 2.0);
    if (!NearlyEqual(measurement.urms_mv, expected_rms, 0.003))
    {
        printf("FAIL conversion rms actual=%.6f expected=%.6f\n",
               (double)measurement.urms_mv,
               expected_rms);
        return 0;
    }

    /*
     * 构造一个等比例标定结果，只用来独立核对相位重建 Upp。
     */
    {
        static const GMeasurementCalibrationPoint flat_point =
        {
            10000.0f,
            0.01f
        };
        static const GMeasurementCalibration flat_calibration =
        {
            &flat_point,
            1U,
            NULL,
            0U,
            NULL,
            0U
        };

        if (GMeasurement_Convert(&spectrum,
                                 &flat_calibration,
                                 &measurement) == 0U)
        {
            printf("FAIL flat conversion returned invalid\n");
            return 0;
        }
        expected_upp = ReconstructExpectedUpp(&spectrum, 0.01);
        if (!NearlyEqual(measurement.upp_mv, expected_upp, 0.02))
        {
            printf("FAIL conversion upp actual=%.6f expected=%.6f\n",
                   (double)measurement.upp_mv,
                   expected_upp);
            return 0;
        }
    }

    printf("PASS calibrated measurement conversion\n");
    return 1;
}

static int TestMissingCalibration(void)
{
    SpectrumResult spectrum;
    GMeasurementResult measurement;
    GMeasurementCalibration calibration =
        { NULL, 0U, NULL, 0U, NULL, 0U };

    memset(&spectrum, 0, sizeof(spectrum));
    memset(&measurement, 0xA5, sizeof(measurement));
    spectrum.valid = 1U;
    spectrum.component_count = 1U;
    spectrum.components[0].harmonic = 1U;
    spectrum.components[0].frequency_hz = 100000.0f;
    spectrum.components[0].amplitude_codes = 1000.0f;

    if ((GMeasurement_Convert(&spectrum,
                              &calibration,
                              &measurement) != 0U) ||
        (measurement.valid != 0U))
    {
        printf("FAIL missing calibration accepted\n");
        return 0;
    }

    printf("PASS missing calibration rejected\n");
    return 1;
}

static int TestProductionCalibration(void)
{
    static const float expected_frequencies[] =
    {
         10000.0f,
         50000.0f,
        100000.0f,
        200000.0f,
        250000.0f,
        300000.0f,
        350000.0f,
        400000.0f,
        450000.0f,
        500000.0f
    };
    static const float expected_scales[] =
    {
        0.031822806f,
        0.031795286f,
        0.031694095f,
        0.031783351f,
        0.032039673f,
        0.032295994f,
        0.032350333f,
        0.032404671f,
        0.032853284f,
        0.033301896f
    };
    const GMeasurementCalibration *calibration =
        GMeasurementCalibration_Get();
    SpectrumResult spectrum;
    GMeasurementResult measurement;
    uint8_t index;

    if ((calibration == NULL) ||
        (calibration->points == NULL) ||
        (calibration->point_count != 10U) ||
        (calibration->phase_points == NULL) ||
        (calibration->phase_point_count != 50U) ||
        (calibration->amplitude_rows == NULL) ||
        (calibration->amplitude_row_count != 50U))
    {
        printf("FAIL production calibration point count\n");
        return 0;
    }

    for (index = 0U; index < calibration->point_count; index++)
    {
        if (!NearlyEqual(calibration->points[index].frequency_hz,
                         expected_frequencies[index],
                         0.01) ||
            !NearlyEqual(calibration->points[index].mv_per_code,
                         expected_scales[index],
                         1.0e-8))
        {
            printf("FAIL production calibration point %u\n",
                   (unsigned int)index);
            return 0;
        }
    }

    for (index = 0U; index < calibration->amplitude_row_count; index++)
    {
        const GMeasurementAmplitudeCalibrationRow *row =
            &calibration->amplitude_rows[index];
        uint8_t level;

        if (!NearlyEqual(row->frequency_hz,
                         10000.0 * (double)(index + 1U),
                         0.01) ||
            (row->level_count != 5U))
        {
            printf("FAIL production 2D row %u\n", (unsigned int)index);
            return 0;
        }

        for (level = 0U; level < row->level_count; level++)
        {
            if (!NearlyEqual(row->levels[level].peak_mv,
                             25.0 * (double)(level + 1U),
                             0.001))
            {
                printf("FAIL production 2D level %u/%u\n",
                       (unsigned int)index,
                       (unsigned int)level);
                return 0;
            }
        }
    }

    if (!NearlyEqual(calibration->phase_points[0].frequency_hz,
                     10000.0,
                     0.01) ||
        !NearlyEqual(calibration->phase_points[49].frequency_hz,
                     500000.0,
                     0.01))
    {
        printf("FAIL production phase table range\n");
        return 0;
    }

    memset(&spectrum, 0, sizeof(spectrum));
    spectrum.valid = 1U;
    spectrum.fundamental_hz = 100000.0f;
    spectrum.component_count = 1U;
    spectrum.components[0].harmonic = 1U;
    spectrum.components[0].frequency_hz = 100000.0f;
    spectrum.components[0].amplitude_codes =
        calibration->amplitude_rows[9].levels[1].amplitude_codes;

    if ((GMeasurement_Convert(&spectrum,
                              calibration,
                              &measurement) == 0U) ||
        !NearlyEqual(measurement.components[0].amplitude_mv,
                     50.0,
                     0.001) ||
        !NearlyEqual(measurement.upp_mv, 100.0, 0.002))
    {
        printf("FAIL production calibration interpolation\n");
        return 0;
    }

    printf("PASS production 50x5 amplitude and 50-point phase tables\n");
    return 1;
}

static int TestTwoDimensionalAmplitudeCalibration(void)
{
    static const GMeasurementCalibrationPoint fallback_point =
    {
        10000.0f,
        0.01f
    };
    static const GMeasurementAmplitudeCalibrationRow amplitude_rows[] =
    {
        {
            100000.0f,
            5U,
            {
                { 1000.0f,  25.0f },
                { 2000.0f,  50.0f },
                { 3000.0f,  75.0f },
                { 4000.0f, 100.0f },
                { 5000.0f, 125.0f }
            }
        },
        {
            300000.0f,
            5U,
            {
                {  800.0f,  25.0f },
                { 1600.0f,  50.0f },
                { 2400.0f,  75.0f },
                { 3200.0f, 100.0f },
                { 4000.0f, 125.0f }
            }
        }
    };
    static const GMeasurementCalibration calibration =
    {
        &fallback_point,
        1U,
        NULL,
        0U,
        amplitude_rows,
        2U
    };
    SpectrumResult spectrum;
    GMeasurementResult measurement;

    memset(&spectrum, 0, sizeof(spectrum));
    spectrum.valid = 1U;
    spectrum.fundamental_hz = 200000.0f;
    spectrum.component_count = 1U;
    spectrum.components[0].harmonic = 1U;
    spectrum.components[0].frequency_hz = 200000.0f;
    spectrum.components[0].amplitude_codes = 1800.0f;
    spectrum.components[0].phase_rad = 0.37f;

    if ((GMeasurement_Convert(&spectrum,
                              &calibration,
                              &measurement) == 0U) ||
        !NearlyEqual(measurement.components[0].amplitude_mv,
                     50.625,
                     0.001) ||
        !NearlyEqual(measurement.upp_mv, 101.25, 0.002))
    {
        printf("FAIL 2D amplitude interpolation amp=%.6f upp=%.6f\n",
               (double)measurement.components[0].amplitude_mv,
               (double)measurement.upp_mv);
        return 0;
    }

    printf("PASS 2D frequency/amplitude calibration\n");
    return 1;
}

static int TestGeneralPhaseCalibration(void)
{
    static const GMeasurementCalibrationPoint amplitude_point =
    {
        10000.0f,
        0.01f
    };
    static const GMeasurementPhaseCalibrationPoint phase_points[] =
    {
        {  25000.0f,  0.15f },
        { 175000.0f, -0.40f },
        { 325000.0f,  0.70f }
    };
    static const GMeasurementCalibration calibration =
    {
        &amplitude_point,
        1U,
        phase_points,
        3U,
        NULL,
        0U
    };
    static const double amplitudes[] = { 50.0, 25.0, 15.0 };
    static const unsigned int harmonics[] = { 1U, 7U, 13U };
    static const double true_phases[] = { 0.20, -0.80, 1.10 };
    static const double phase_errors[] = { 0.15, -0.40, 0.70 };
    const double common_phase = 0.47;
    SpectrumResult spectrum;
    GMeasurementResult measurement;
    double expected_upp =
        ReconstructComponentUpp(amplitudes,
                                harmonics,
                                true_phases,
                                3U);
    unsigned int component;

    memset(&spectrum, 0, sizeof(spectrum));
    spectrum.valid = 1U;
    spectrum.fundamental_hz = 25000.0f;
    spectrum.component_count = 3U;

    for (component = 0U; component < 3U; component++)
    {
        spectrum.components[component].harmonic =
            (uint8_t)harmonics[component];
        spectrum.components[component].frequency_hz =
            25000.0f * (float)harmonics[component];
        spectrum.components[component].amplitude_codes =
            (float)(amplitudes[component] /
                    (0.01 * G_MEASUREMENT_50_OHM_AMPLITUDE_SCALE));
        spectrum.components[component].phase_rad =
            (float)(true_phases[component] +
                    phase_errors[component] +
                    (double)harmonics[component] * common_phase);
    }

    if ((GMeasurement_Convert(&spectrum,
                              &calibration,
                              &measurement) == 0U) ||
        !NearlyEqual(measurement.upp_mv, expected_upp, 0.01))
    {
        printf("FAIL general phase calibration actual=%.6f expected=%.6f\n",
               (double)measurement.upp_mv,
               expected_upp);
        return 0;
    }

    printf("PASS general H1/H7/H13 phase calibration upperr=%.6fmV\n",
           fabs((double)measurement.upp_mv - expected_upp));
    return 1;
}

static int TestRefinedVppExtrema(void)
{
    static const GMeasurementCalibrationPoint amplitude_point =
    {
        10000.0f,
        0.01f
    };
    static const GMeasurementCalibration calibration =
    {
        &amplitude_point,
        1U,
        NULL,
        0U,
        NULL,
        0U
    };
    static const double amplitudes[] = { 5.0, 100.0 };
    static const unsigned int harmonics[] = { 1U, 49U };
    double phases[2];
    SpectrumResult spectrum;
    GMeasurementResult measurement;
    double expected_upp;
    unsigned int component;

    phases[0] = 0.31;
    phases[1] =
        -49.0 * TEST_PI / (double)TEST_RECONSTRUCT_POINTS;
    expected_upp = ReconstructComponentUpp(amplitudes,
                                           harmonics,
                                           phases,
                                           2U);

    memset(&spectrum, 0, sizeof(spectrum));
    spectrum.valid = 1U;
    spectrum.fundamental_hz = 10000.0f;
    spectrum.component_count = 2U;

    for (component = 0U; component < 2U; component++)
    {
        spectrum.components[component].harmonic =
            (uint8_t)harmonics[component];
        spectrum.components[component].frequency_hz =
            10000.0f * (float)harmonics[component];
        spectrum.components[component].amplitude_codes =
            (float)(amplitudes[component] /
                    (0.01 * G_MEASUREMENT_50_OHM_AMPLITUDE_SCALE));
        spectrum.components[component].phase_rad =
            (float)phases[component];
    }

    if ((GMeasurement_Convert(&spectrum,
                              &calibration,
                              &measurement) == 0U) ||
        !NearlyEqual(measurement.upp_mv, expected_upp, 0.01))
    {
        printf("FAIL refined Vpp actual=%.6f expected=%.6f\n",
               (double)measurement.upp_mv,
               expected_upp);
        return 0;
    }

    printf("PASS refined H49 Vpp upperr=%.6fmV\n",
           fabs((double)measurement.upp_mv - expected_upp));
    return 1;
}

static void GenerateWaveform(int16_t *frame, double phase_offset)
{
    unsigned int index;
    const double f0 = 10500.0;
    const double fs = 5000000.0;

    for (index = 0U; index < TEST_FRAME_LENGTH; index++)
    {
        double theta =
            2.0 * TEST_PI * f0 * (double)index / fs + phase_offset;
        double value =
            700.0 +
            3500.0 * cos(theta) +
            7000.0 * cos(3.0 * theta - 0.7) +
            2400.0 * cos(4.0 * theta + 1.1);

        frame[index] =
            (int16_t)((value >= 0.0) ? (value + 0.5) : (value - 0.5));
    }
}

static int TestWaveformCycles(void)
{
    GMeasurementWaveform one_a;
    GMeasurementWaveform one_b;
    GMeasurementWaveform three;
    double normalized_error = 0.0;
    double range;
    uint16_t index;

    GenerateWaveform(s_FrameA, 0.37);
    GenerateWaveform(s_FrameB, 1.61);

    if ((GMeasurement_BuildWaveform(s_FrameA,
                                    TEST_FRAME_LENGTH,
                                    5000000.0f,
                                    10500.0f,
                                    1U,
                                    &one_a) == 0U) ||
        (GMeasurement_BuildWaveform(s_FrameB,
                                    TEST_FRAME_LENGTH,
                                    5000000.0f,
                                    10500.0f,
                                    1U,
                                    &one_b) == 0U) ||
        (GMeasurement_BuildWaveform(s_FrameA,
                                    TEST_FRAME_LENGTH,
                                    5000000.0f,
                                    10500.0f,
                                    3U,
                                    &three) == 0U))
    {
        printf("FAIL waveform builder returned invalid\n");
        return 0;
    }

    if ((one_a.cycles != 1U) ||
        (three.cycles != 3U) ||
        !NearlyEqual(one_a.span_samples,
                     5000000.0 / 10500.0,
                     0.05) ||
        !NearlyEqual(three.span_samples,
                     3.0 * 5000000.0 / 10500.0,
                     0.15))
    {
        printf("FAIL waveform span one=%.3f three=%.3f\n",
               (double)one_a.span_samples,
               (double)three.span_samples);
        return 0;
    }

    range = (double)one_a.maximum_code - (double)one_a.minimum_code;
    for (index = 0U; index < G_MEASUREMENT_WAVEFORM_POINTS; index++)
    {
        normalized_error +=
            fabs((double)one_a.points[index] -
                 (double)one_b.points[index]) /
            range;
    }
    normalized_error /= (double)G_MEASUREMENT_WAVEFORM_POINTS;

    if (normalized_error > 0.03)
    {
        printf("FAIL waveform phase alignment error=%.5f\n",
               normalized_error);
        return 0;
    }

    printf("PASS 1/3-cycle waveform alignment error=%.5f\n",
           normalized_error);
    return 1;
}

static int TestHighFrequencyWaveformSmoothness(void)
{
    GMeasurementWaveform one_cycle;
    GMeasurementWaveform three_cycles;
    const double frequency_hz = 500000.0;
    const double sample_rate_hz = 5000000.0;
    const double amplitude = 12000.0;
    const double phase_offset = 0.73;
    double maximum_error_one = 0.0;
    double maximum_error_three = 0.0;
    unsigned int index;

    for (index = 0U; index < TEST_FRAME_LENGTH; index++)
    {
        double phase = 2.0 * TEST_PI * frequency_hz * (double)index /
                       sample_rate_hz + phase_offset;
        double value = 500.0 + amplitude * cos(phase);

        s_FrameA[index] =
            (int16_t)((value >= 0.0) ? (value + 0.5) : (value - 0.5));
    }

    if ((GMeasurement_BuildWaveform(s_FrameA,
                                    TEST_FRAME_LENGTH,
                                    (float)sample_rate_hz,
                                    (float)frequency_hz,
                                    1U,
                                    &one_cycle) == 0U) ||
        (GMeasurement_BuildWaveform(s_FrameA,
                                    TEST_FRAME_LENGTH,
                                    (float)sample_rate_hz,
                                    (float)frequency_hz,
                                    3U,
                                    &three_cycles) == 0U))
    {
        printf("FAIL 500kHz waveform builder returned invalid\n");
        return 0;
    }

    for (index = 0U; index < G_MEASUREMENT_WAVEFORM_POINTS; index++)
    {
        double base_phase = 2.0 * TEST_PI * (double)index /
                            (double)(G_MEASUREMENT_WAVEFORM_POINTS - 1U);
        double expected_one = amplitude * cos(base_phase);
        double expected_three = amplitude * cos(3.0 * base_phase);
        double error_one =
            fabs((double)one_cycle.points[index] - expected_one);
        double error_three =
            fabs((double)three_cycles.points[index] - expected_three);

        if (error_one > maximum_error_one)
        {
            maximum_error_one = error_one;
        }
        if (error_three > maximum_error_three)
        {
            maximum_error_three = error_three;
        }
    }

    /* 线性插值在10点/周期时误差约为峰值的5%；三次插值应低于1.5%。 */
    if ((maximum_error_one > amplitude * 0.015) ||
        (maximum_error_three > amplitude * 0.015))
    {
        printf("FAIL 500kHz waveform interpolation one=%.2f three=%.2f codes\n",
               maximum_error_one,
               maximum_error_three);
        return 0;
    }

    printf("PASS 500kHz smooth waveform one=%.2f three=%.2f codes\n",
           maximum_error_one,
           maximum_error_three);
    return 1;
}

static double TestCalibrationScale(double frequency_hz)
{
    const double scale_100k = 0.010000;
    const double scale_500k = 0.010563;

    if (frequency_hz <= 100000.0)
    {
        return scale_100k;
    }
    if (frequency_hz >= 500000.0)
    {
        return scale_500k;
    }

    return scale_100k +
           (frequency_hz - 100000.0) /
           400000.0 *
           (scale_500k - scale_100k);
}

static double ExpectedPhysicalUpp(void)
{
    double minimum = 0.0;
    double maximum = 0.0;
    unsigned int point;

    for (point = 0U; point < 65536U; point++)
    {
        double theta = 2.0 * TEST_PI * (double)point / 65536.0;
        double value =
            80.0 * cos(theta + 0.2) +
            30.0 * cos(5.0 * theta - 0.7);

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

    return maximum - minimum;
}

static int TestEndToEndMvLimits(void)
{
    static const GMeasurementCalibrationPoint calibration_points[] =
    {
        { 100000.0f, 0.010000f },
        { 500000.0f, 0.010563f }
    };
    static const GMeasurementCalibration calibration =
    {
        calibration_points,
        2U,
        NULL,
        0U,
        NULL,
        0U
    };
    SpectrumResult spectrum;
    GMeasurementResult measurement;
    double expected_rms =
        sqrt((40.0 * 40.0 + 15.0 * 15.0) / 2.0);
    double expected_upp =
        ExpectedPhysicalUpp() * G_MEASUREMENT_50_OHM_AMPLITUDE_SCALE;
    unsigned int index;

    for (index = 0U; index < TEST_FRAME_LENGTH; index++)
    {
        double time = (double)index / (double)SPECTRUM_SAMPLE_RATE_HZ;
        double fundamental_codes =
            80.0 / TestCalibrationScale(100000.0);
        double harmonic_codes =
            30.0 / TestCalibrationScale(500000.0);
        double value =
            700.0 +
            fundamental_codes *
                cos(2.0 * TEST_PI * 100000.0 * time + 0.2) +
            harmonic_codes *
                cos(2.0 * TEST_PI * 500000.0 * time - 0.7);

        s_FrameA[index] =
            (int16_t)((value >= 0.0) ? (value + 0.5) : (value - 0.5));
    }

    if ((SpectrumAnalyzer_Run(s_FrameA, &spectrum) == 0U) ||
        (GMeasurement_Convert(&spectrum,
                              &calibration,
                              &measurement) == 0U))
    {
        printf("FAIL end-to-end pipeline returned invalid\n");
        return 0;
    }

    if ((spectrum.component_count != 2U) ||
        (spectrum.components[0].harmonic != 1U) ||
        (spectrum.components[1].harmonic != 5U) ||
        (fabs((double)spectrum.fundamental_hz - 100000.0) > 1000.0) ||
        (fabs((double)measurement.components[0].amplitude_mv - 40.0) >
         5.0) ||
        (fabs((double)measurement.components[1].amplitude_mv - 15.0) >
         5.0) ||
        (fabs((double)measurement.urms_mv - expected_rms) > 5.0) ||
        (fabs((double)measurement.upp_mv - expected_upp) > 5.0))
    {
        printf("FAIL end-to-end f0=%.2f h1=%.3f h5=%.3f rms=%.3f upp=%.3f\n",
               (double)spectrum.fundamental_hz,
               (double)measurement.components[0].amplitude_mv,
               (double)measurement.components[1].amplitude_mv,
               (double)measurement.urms_mv,
               (double)measurement.upp_mv);
        return 0;
    }

    printf("PASS end-to-end absolute limits f0err=%.2fHz "
           "h1err=%.3fmV h5err=%.3fmV rmserr=%.3fmV upperr=%.3fmV\n",
           fabs((double)spectrum.fundamental_hz - 100000.0),
           fabs((double)measurement.components[0].amplitude_mv - 40.0),
           fabs((double)measurement.components[1].amplitude_mv - 15.0),
           fabs((double)measurement.urms_mv - expected_rms),
           fabs((double)measurement.upp_mv - expected_upp));
    return 1;
}

int main(void)
{
    int passed = 1;

    passed &= TestMeasurementConversion();
    passed &= TestMissingCalibration();
    passed &= TestProductionCalibration();
    passed &= TestTwoDimensionalAmplitudeCalibration();
    passed &= TestGeneralPhaseCalibration();
    passed &= TestRefinedVppExtrema();
    passed &= TestWaveformCycles();
    passed &= TestHighFrequencyWaveformSmoothness();
    passed &= TestEndToEndMvLimits();
    return passed ? 0 : 1;
}
