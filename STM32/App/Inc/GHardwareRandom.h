#ifndef __G_HARDWARE_RANDOM_H
#define __G_HARDWARE_RANDOM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 开启/关闭STM32F407片上硬件RNG及其AHB2外设时钟。 */
void GHardwareRandom_Enable(void);
void GHardwareRandom_Disable(void);

/*
 * 生成[0, upper_bound)范围内的硬件随机浮点数。
 * 成功返回1；RNG未开启、参数无效、硬件错误或超时返回0。
 */
uint8_t GHardwareRandom_GetFloatBelow(float upper_bound, float *value);

#ifdef __cplusplus
}
#endif

#endif /* __G_HARDWARE_RANDOM_H */
