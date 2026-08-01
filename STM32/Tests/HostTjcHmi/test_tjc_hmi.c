#include "tjc_usart_hmi.h"

#include <stdio.h>
#include <string.h>
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
    uint16_t component_width = 0U;
    TjcHmiEvent deferred_event = TJC_HMI_EVENT_NONE;
    uint8_t deferred_cycles = 0U;
    static const uint8_t button_one[] =
        {0x55U, 0x01U, 0x00U, 0x00U, 0xFFU, 0xFFU, 0xFFU};
    static const uint8_t button_three[] =
        {0x55U, 0x02U, 0x00U, 0x00U, 0xFFU, 0xFFU, 0xFFU};
    static const uint8_t button_frequency[] =
        {0x55U, 0x03U, 0x00U, 0x00U, 0xFFU, 0xFFU, 0xFFU};
    static const uint8_t button_range[] =
        {0x55U, 0x04U, 0x00U, 0x00U, 0xFFU, 0xFFU, 0xFFU};
    static const uint8_t noisy_button_three[] =
        {0x00U, 0xAAU, 0x55U, 0x02U, 0x00U, 0x00U, 0xFFU, 0xFFU, 0xFFU};
    static const uint8_t false_header_then_button[] =
        {0x55U, 0xAAU, 0x00U, 0x55U, 0x01U, 0x00U, 0x00U,
         0xFFU, 0xFFU, 0xFFU};
    static const uint8_t ascii_one[] = {'1'};
    static const uint8_t ascii_three[] = {'3'};
    static const uint8_t width_512_response[] =
        {0x71U, 0x00U, 0x02U, 0x00U, 0x00U, 0xFFU, 0xFFU, 0xFFU};
    static const uint8_t button_during_get[] =
        {0x55U, 0x02U, 0x00U, 0x00U, 0xFFU, 0xFFU, 0xFFU,
         0x71U, 0x00U, 0x02U, 0x00U, 0x00U, 0xFFU, 0xFFU, 0xFFU};
    static const uint8_t addt_with_button[] =
        {0x55U, 0x03U, 0x00U, 0x00U, 0xFFU, 0xFFU, 0xFFU,
         0xFEU, 0xFFU, 0xFFU, 0xFFU,
         0xFDU, 0xFFU, 0xFFU, 0xFFU};
    static const uint8_t wave_data[] = {20U, 40U, 60U, 40U};

    TjcHmi_Init();
    s_TestTick = 0U;

    TjcHmi_SetQuantizationEnabled(1U);
    TjcHmi_SetComputeBusy(1U);
    if (strstr(str1, "\xB4\xF3" "1 ") == NULL)
    {
        puts("FAIL TJC HMI large quantization-on status");
        return 1;
    }
    TjcHmi_SetQuantizationEnabled(0U);
    if (strstr(str1, "\xB4\xF3" "0 ") == NULL)
    {
        puts("FAIL TJC HMI large quantization-off status");
        return 1;
    }
    TjcHmi_SetAlgorithmOptimized(0U);
    if (strstr(str1, "\xD0\xA1" "0 ") == NULL)
    {
        puts("FAIL TJC HMI small quantization-off status");
        return 1;
    }

    if (!ExpectEvent(button_one,
                     sizeof(button_one),
                     TJC_HMI_EVENT_START_MEASUREMENT,
                     0U) ||
        !ExpectEvent(button_three,
                     sizeof(button_three),
                     TJC_HMI_EVENT_TOGGLE_CYCLES,
                     0U) ||
        !ExpectEvent(button_frequency,
                     sizeof(button_frequency),
                     TJC_HMI_EVENT_FREQUENCY_MEASUREMENT,
                     0U) ||
        !ExpectEvent(button_range,
                     sizeof(button_range),
                     TJC_HMI_EVENT_TOGGLE_ALGORITHM,
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

    LoadInput(width_512_response, sizeof(width_512_response));
    if ((TjcHmi_GetComponentWidth("s0", &component_width) == 0U) ||
        (component_width != 512U))
    {
        puts("FAIL TJC HMI component width response");
        return 1;
    }

    LoadInput(button_during_get, sizeof(button_during_get));
    if ((TjcHmi_GetComponentWidth("s1", &component_width) == 0U) ||
        (component_width != 512U) ||
        (TjcHmi_ReadEvent(&deferred_event, &deferred_cycles) == 0U) ||
        (deferred_event != TJC_HMI_EVENT_TOGGLE_CYCLES))
    {
        puts("FAIL TJC HMI deferred button during get");
        return 1;
    }

    LoadInput(addt_with_button, sizeof(addt_with_button));
    if ((tjc_send_wave("s0.id", 0, wave_data, sizeof(wave_data)) == 0U) ||
        (TjcHmi_ReadEvent(&deferred_event, &deferred_cycles) == 0U) ||
        (deferred_event != TJC_HMI_EVENT_FREQUENCY_MEASUREMENT))
    {
        puts("FAIL TJC HMI addt handshake");
        return 1;
    }

    puts("PASS TJC HMI: buttons, deferred queries and addt handshake");
    return 0;
}
