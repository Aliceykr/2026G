#include "GHardwareRandom.h"

#include "stm32f4xx.h"

#include <stddef.h>

#define G_HARDWARE_RANDOM_TIMEOUT_ITERATIONS  100000UL
#define G_HARDWARE_RANDOM_24BIT_SCALE         0.000000059604644775390625f

static uint8_t s_HardwareRandomEnabled;

void GHardwareRandom_Enable(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
    (void)RCC->AHB2ENR;
    RNG->CR |= RNG_CR_RNGEN;
    s_HardwareRandomEnabled = 1U;
}

void GHardwareRandom_Disable(void)
{
    RNG->CR &= ~RNG_CR_RNGEN;
    RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN;
    (void)RCC->AHB2ENR;
    s_HardwareRandomEnabled = 0U;
}

uint8_t GHardwareRandom_GetFloatBelow(float upper_bound, float *value)
{
    uint32_t timeout;

    if ((s_HardwareRandomEnabled == 0U) ||
        (value == NULL) ||
        (upper_bound <= 0.0f))
    {
        return 0U;
    }

    for (timeout = 0UL;
         timeout < G_HARDWARE_RANDOM_TIMEOUT_ITERATIONS;
         timeout++)
    {
        uint32_t status = RNG->SR;

        if ((status & (RNG_SR_CECS | RNG_SR_SECS)) != 0UL)
        {
            return 0U;
        }

        if ((status & RNG_SR_DRDY) != 0UL)
        {
            uint32_t random24 = RNG->DR >> 8U;

            *value = (float)random24 *
                     G_HARDWARE_RANDOM_24BIT_SCALE *
                     upper_bound;
            return 1U;
        }
    }

    return 0U;
}
