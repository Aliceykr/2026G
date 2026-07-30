#ifndef HOST_CMSIS_ARM_MATH_H
#define HOST_CMSIS_ARM_MATH_H

#include <stdint.h>

typedef float float32_t;

typedef enum
{
    ARM_MATH_SUCCESS = 0,
    ARM_MATH_ARGUMENT_ERROR = -1
} arm_status;

typedef struct
{
    uint16_t fftLenRFFT;
} arm_rfft_fast_instance_f32;

arm_status arm_rfft_fast_init_f32(arm_rfft_fast_instance_f32 *instance,
                                  uint16_t fft_length);
void arm_rfft_fast_f32(const arm_rfft_fast_instance_f32 *instance,
                       float32_t *input,
                       float32_t *output,
                       uint8_t inverse);

#endif
