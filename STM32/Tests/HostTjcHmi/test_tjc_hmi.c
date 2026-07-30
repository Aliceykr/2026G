#include "tjc_usart_hmi.h"

#include <stdio.h>
static uint32_t s_TestTick;

uint32_t HAL_GetTick(void)
{
    return s_TestTick;
}

void HAL_Delay(uint32_t delay_ms)
{
    s_TestTick += delay_ms;
}

static void LoadInput(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    for (index = 0U; index < length; index++)
    {
        write1ByteToRingBuffer(data[index]);
    }
}

static int ExpectEvent(const uint8_t *data,
                       uint16_t length,
                       TjcHmiEvent expected_event,
                       uint8_t expected_cycles)
{
    TjcHmiEvent event = TJC_HMI_EVENT_NONE;
    uint8_t cycles = 0U;

    LoadInput(data, length);
    if (TjcHmi_ReadEvent(&event, &cycles) == 0U)
    {
        return 0;
    }

    if (event != expected_event)
    {
        return 0;
    }

    if (event == TJC_HMI_EVENT_SET_CYCLES)
    {
        return cycles == expected_cycles;
    }

    return 1;
}

int main(void)
{
    static const uint8_t button_one[] =
        {0x55U, 0x01U, 0x00U, 0x00U, 0xFFU, 0xFFU, 0xFFU};
    static const uint8_t button_three[] =
        {0x55U, 0x02U, 0x00U, 0x00U, 0xFFU, 0xFFU, 0xFFU};
    static const uint8_t noisy_button_three[] =
        {0x00U, 0xAAU, 0x55U, 0x02U, 0x00U, 0x00U, 0xFFU, 0xFFU, 0xFFU};
    static const uint8_t false_header_then_button[] =
        {0x55U, 0xAAU, 0x00U, 0x55U, 0x01U, 0x00U, 0x00U,
         0xFFU, 0xFFU, 0xFFU};
    static const uint8_t ascii_one[] = {'1'};
    static const uint8_t ascii_three[] = {'3'};

    TjcHmi_Init();
    s_TestTick = 0U;

    if (!ExpectEvent(button_one,
                     sizeof(button_one),
                     TJC_HMI_EVENT_START_MEASUREMENT,
                     0U) ||
        !ExpectEvent(button_three,
                     sizeof(button_three),
                     TJC_HMI_EVENT_TOGGLE_CYCLES,
                     0U) ||
        !ExpectEvent(noisy_button_three,
                     sizeof(noisy_button_three),
                     TJC_HMI_EVENT_TOGGLE_CYCLES,
                     0U) ||
        !ExpectEvent(false_header_then_button,
                     sizeof(false_header_then_button),
                     TJC_HMI_EVENT_START_MEASUREMENT,
                     0U) ||
        !ExpectEvent(ascii_one,
                     sizeof(ascii_one),
                     TJC_HMI_EVENT_SET_CYCLES,
                     1U) ||
        !ExpectEvent(ascii_three,
                     sizeof(ascii_three),
                     TJC_HMI_EVENT_SET_CYCLES,
                     3U))
    {
        puts("FAIL TJC HMI button parser");
        return 1;
    }

    puts("PASS TJC HMI: first b0 frame, header resync, b1 and ASCII compatible");
    return 0;
}
