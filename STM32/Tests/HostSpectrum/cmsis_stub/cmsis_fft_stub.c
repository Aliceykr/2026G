#include "arm_math.h"

#include <math.h>
#include <stddef.h>

#define HOST_FFT_MAX_LENGTH 4096U
#define HOST_PI             3.14159265358979323846f

static float s_Real[HOST_FFT_MAX_LENGTH];
static float s_Imaginary[HOST_FFT_MAX_LENGTH];

arm_status arm_rfft_fast_init_f32(arm_rfft_fast_instance_f32 *instance,
                                  uint16_t fft_length)
{
    if ((instance == NULL) || (fft_length != HOST_FFT_MAX_LENGTH))
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }

    instance->fftLenRFFT = fft_length;
    return ARM_MATH_SUCCESS;
}

void arm_rfft_fast_f32(const arm_rfft_fast_instance_f32 *instance,
                       float32_t *input,
                       float32_t *output,
                       uint8_t inverse)
{
    uint16_t fft_length;
    uint16_t index;
    uint16_t reverse = 0U;
    uint16_t span;

    if ((instance == NULL) ||
        (input == NULL) ||
        (output == NULL) ||
        (inverse != 0U))
    {
        return;
    }

    fft_length = instance->fftLenRFFT;
    if (fft_length != HOST_FFT_MAX_LENGTH)
    {
        return;
    }

    for (index = 0U; index < fft_length; index++)
    {
        s_Real[index] = input[index];
        s_Imaginary[index] = 0.0f;
    }

    for (index = 1U; index < fft_length; index++)
    {
        uint16_t bit = (uint16_t)(fft_length >> 1U);

        while ((reverse & bit) != 0U)
        {
            reverse ^= bit;
            bit >>= 1U;
        }
        reverse ^= bit;

        if (index < reverse)
        {
            float temporary = s_Real[index];
            s_Real[index] = s_Real[reverse];
            s_Real[reverse] = temporary;
        }
    }

    for (span = 2U; span <= fft_length; span <<= 1U)
    {
        uint16_t half_span = (uint16_t)(span >> 1U);
        float angle = -2.0f * HOST_PI / (float)span;
        float step_real = cosf(angle);
        float step_imaginary = sinf(angle);
        uint16_t base;

        for (base = 0U; base < fft_length; base = (uint16_t)(base + span))
        {
            float twiddle_real = 1.0f;
            float twiddle_imaginary = 0.0f;
            uint16_t offset;

            for (offset = 0U; offset < half_span; offset++)
            {
                uint16_t even = (uint16_t)(base + offset);
                uint16_t odd = (uint16_t)(even + half_span);
                float odd_real =
                    twiddle_real * s_Real[odd] -
                    twiddle_imaginary * s_Imaginary[odd];
                float odd_imaginary =
                    twiddle_real * s_Imaginary[odd] +
                    twiddle_imaginary * s_Real[odd];
                float next_twiddle_real;

                s_Real[odd] = s_Real[even] - odd_real;
                s_Imaginary[odd] = s_Imaginary[even] - odd_imaginary;
                s_Real[even] += odd_real;
                s_Imaginary[even] += odd_imaginary;

                next_twiddle_real =
                    twiddle_real * step_real -
                    twiddle_imaginary * step_imaginary;
                twiddle_imaginary =
                    twiddle_imaginary * step_real +
                    twiddle_real * step_imaginary;
                twiddle_real = next_twiddle_real;
            }
        }
    }

    output[0] = s_Real[0];
    output[1] = s_Real[fft_length / 2U];
    for (index = 1U; index < (fft_length / 2U); index++)
    {
        output[2U * index] = s_Real[index];
        output[2U * index + 1U] = s_Imaginary[index];
    }
}
