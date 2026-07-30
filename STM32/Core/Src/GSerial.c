#include "GSerial.h"

#include "usart.h"

#include <stddef.h>

#define G_SERIAL_RX_QUEUE_SIZE 8U

static uint8_t s_RxByte;
static volatile uint32_t s_RxByteCount;
static volatile uint32_t s_RxErrorCount;
static volatile uint8_t s_RxQueue[G_SERIAL_RX_QUEUE_SIZE];
static volatile uint8_t s_RxWriteIndex;
static volatile uint8_t s_RxReadIndex;

static void GSerial_StartReceive(void)
{
    if (HAL_UART_Receive_IT(&huart1, &s_RxByte, 1U) != HAL_OK)
    {
        s_RxErrorCount++;
    }
}

void GSerial_Init(void)
{
    s_RxByteCount = 0UL;
    s_RxErrorCount = 0UL;
    s_RxWriteIndex = 0U;
    s_RxReadIndex = 0U;
    GSerial_StartReceive();
}

uint8_t GSerial_Transmit(const uint8_t *data,
                         uint16_t length,
                         uint32_t timeout_ms)
{
    if ((data == NULL) || (length == 0U))
    {
        return 0U;
    }

    return (HAL_UART_Transmit(&huart1,
                              (uint8_t *)data,
                              length,
                              timeout_ms) == HAL_OK) ? 1U : 0U;
}

uint8_t GSerial_ReadByte(uint8_t *byte)
{
    uint32_t primask;

    if (byte == NULL)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_RxReadIndex == s_RxWriteIndex)
    {
        __set_PRIMASK(primask);
        return 0U;
    }

    *byte = s_RxQueue[s_RxReadIndex];
    s_RxReadIndex =
        (uint8_t)((s_RxReadIndex + 1U) % G_SERIAL_RX_QUEUE_SIZE);
    __set_PRIMASK(primask);
    return 1U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t next_write;

    if (huart != &huart1)
    {
        return;
    }

    s_RxByteCount++;
    next_write =
        (uint8_t)((s_RxWriteIndex + 1U) % G_SERIAL_RX_QUEUE_SIZE);

    if (next_write == s_RxReadIndex)
    {
        s_RxErrorCount++;
    }
    else
    {
        s_RxQueue[s_RxWriteIndex] = s_RxByte;
        s_RxWriteIndex = next_write;
    }

    GSerial_StartReceive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart1)
    {
        return;
    }

    s_RxErrorCount++;
    GSerial_StartReceive();
}
