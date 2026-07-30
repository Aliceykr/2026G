#include "SpectrumAnalyzer.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_PI 3.14159265358979323846

typedef struct
{
    const char *name;
    double frequency_hz;
    uint8_t component_count;
    uint8_t harmonics[SPECTRUM_MAX_COMPONENTS];
    double amplitudes[SPECTRUM_MAX_COMPONENTS];
    double phases[SPECTRUM_MAX_COMPONENTS];
    double max_frequency_error_hz;
    double max_amplitude_relative_error;
    double spur_frequency_hz;
    double spur_amplitude;
} SpectrumTestCase;

static int16_t s_Samples[SPECTRUM_FRAME_LENGTH];

static void GenerateSamples(const SpectrumTestCase *test_case)
{
    uint16_t sample_index;

    for (sample_index = 0U;
         sample_index < SPECTRUM_FRAME_LENGTH;
         sample_index++)
    {
        double time =
            (double)sample_index / (double)SPECTRUM_SAMPLE_RATE_HZ;
        double value = 800.0;
        uint8_t component;

        for (component = 0U;
             component < test_case->component_count;
             component++)
        {
            double harmonic =
                (double)test_case->harmonics[component];
            value += test_case->amplitudes[component] *
                     cos(2.0 * TEST_PI *
                         test_case->frequency_hz * harmonic * time +
                         test_case->phases[component]);
        }

        if (test_case->spur_amplitude > 0.0)
        {
            value += test_case->spur_amplitude *
                     cos(2.0 * TEST_PI *
                         test_case->spur_frequency_hz * time + 0.35);
        }

        if (value > 32767.0)
        {
            value = 32767.0;
        }
        else if (value < -32768.0)
        {
            value = -32768.0;
        }

        s_Samples[sample_index] =
            (int16_t)((value >= 0.0) ? (value + 0.5) : (value - 0.5));
    }
}

static int RunCase(const SpectrumTestCase *test_case)
{
    SpectrumResult result;
    uint8_t display_points[256];
    double frequency_error;
    uint8_t component;
    uint16_t display_index;
    uint8_t display_maximum = 0U;
    int passed = 1;

    GenerateSamples(test_case);
    if (SpectrumAnalyzer_Run(s_Samples, &result) == 0U)
    {
        printf("FAIL %-18s analyzer returned invalid\n", test_case->name);
        return 0;
    }

    if (SpectrumAnalyzer_BuildDisplay(display_points,
                                      sizeof(display_points)) == 0U)
    {
        passed = 0;
    }
    else
    {
        for (display_index = 0U;
             display_index < sizeof(display_points);
             display_index++)
        {
            if (display_points[display_index] > display_maximum)
            {
                display_maximum = display_points[display_index];
            }
        }
        if (display_maximum != 254U)
        {
            passed = 0;
        }
    }

    frequency_error =
        fabs((double)result.fundamental_hz - test_case->frequency_hz);
    if (frequency_error > test_case->max_frequency_error_hz)
    {
        passed = 0;
    }

    if (result.component_count != test_case->component_count)
    {
        passed = 0;
    }

    if (test_case->spur_amplitude > 0.0)
    {
        double spur_frequency_error =
            fabs((double)result.spur_frequency_hz -
                 test_case->spur_frequency_hz);
        double spur_amplitude_error =
            fabs((double)result.spur_amplitude_codes -
                 test_case->spur_amplitude) /
            test_case->spur_amplitude;

        if ((result.spur_valid == 0U) ||
            (spur_frequency_error >
             test_case->max_frequency_error_hz) ||
            (spur_amplitude_error > 0.20))
        {
            passed = 0;
        }
    }

    for (component = 0U;
         (component < result.component_count) &&
         (component < test_case->component_count);
         component++)
    {
        double expected = test_case->amplitudes[component];
        double relative_error =
            fabs((double)result.components[component].amplitude_codes -
                 expected) / expected;

        if ((result.components[component].harmonic !=
             test_case->harmonics[component]) ||
            (relative_error > test_case->max_amplitude_relative_error))
        {
            passed = 0;
        }
    }

    printf("%s %-18s f0=%9.2fHz err=%7.2fHz n=%u rms=%8.2f vpp=%8.2f",
           passed ? "PASS" : "FAIL",
           test_case->name,
           (double)result.fundamental_hz,
           frequency_error,
           (unsigned int)result.component_count,
           (double)result.rms_codes,
           (double)result.vpp_codes);

    for (component = 0U; component < result.component_count; component++)
    {
        printf(" h%u=%.1f",
               (unsigned int)result.components[component].harmonic,
               (double)result.components[component].amplitude_codes);
    }
    if (result.spur_valid != 0U)
    {
        printf(" spur=%.1fHz:%.1f",
               (double)result.spur_frequency_hz,
               (double)result.spur_amplitude_codes);
    }
    printf("\n");

    return passed;
}

static int TestNoSignal(void)
{
    SpectrumResult result;
    uint16_t index;

    memset(s_Samples, 0, sizeof(s_Samples));
    if ((SpectrumAnalyzer_Run(s_Samples, &result) != 0U) ||
        (result.valid != 0U) ||
        (result.status != SPECTRUM_STATUS_NO_SIGNAL))
    {
        printf("FAIL no-signal zero frame accepted status=%u\n",
               (unsigned int)result.status);
        return 0;
    }

    for (index = 0U; index < SPECTRUM_FRAME_LENGTH; index++)
    {
        s_Samples[index] = (int16_t)((int32_t)(index % 5U) - 2);
    }

    if ((SpectrumAnalyzer_Run(s_Samples, &result) != 0U) ||
        (result.valid != 0U) ||
        (result.status != SPECTRUM_STATUS_NO_SIGNAL))
    {
        printf("FAIL no-signal bounded noise accepted status=%u\n",
               (unsigned int)result.status);
        return 0;
    }

    printf("PASS no-signal zero/bounded-noise rejected threshold=%.0f\n",
           (double)SPECTRUM_MIN_VALID_AMPLITUDE_CODES);
    return 1;
}

int main(void)
{
    static const SpectrumTestCase cases[] =
    {
        {
            "lower-bound",
            10000.0,
            3U,
            {1U, 2U, 3U},
            {9000.0, 2400.0, 900.0},
            {0.2, -0.7, 1.1},
            180.0,
            0.06,
            0.0,
            0.0
        },
        {
            "non-bin-103333",
            103333.3,
            3U,
            {1U, 2U, 3U},
            {10000.0, 2500.0, 1200.0},
            {0.5, -1.0, 0.8},
            180.0,
            0.06,
            0.0,
            0.0
        },
        {
            "two-tone-250k",
            250000.0,
            2U,
            {1U, 2U, 0U},
            {9000.0, 1800.0, 0.0},
            {-0.3, 0.9, 0.0},
            180.0,
            0.06,
            0.0,
            0.0
        },
        {
            "upper-bound",
            500000.0,
            1U,
            {1U, 0U, 0U},
            {8000.0, 0.0, 0.0},
            {0.4, 0.0, 0.0},
            180.0,
            0.06,
            0.0,
            0.0
        },
        {
            "official-h1-h3-h4",
            10500.0,
            3U,
            {1U, 3U, 4U},
            {3500.0, 7000.0, 2400.0},
            {0.1, -0.8, 1.2},
            180.0,
            0.06,
            0.0,
            0.0
        },
        {
            "unmatched-spur-250k",
            100000.0,
            1U,
            {1U, 0U, 0U},
            {1000.0, 0.0, 0.0},
            {0.2, 0.0, 0.0},
            180.0,
            0.06,
            250000.0,
            300.0
        }
    };
    size_t index;
    int passed = 1;

    passed &= TestNoSignal();

    for (index = 0U; index < sizeof(cases) / sizeof(cases[0]); index++)
    {
        if (!RunCase(&cases[index]))
        {
            passed = 0;
        }
    }

    return passed ? 0 : 1;
}
