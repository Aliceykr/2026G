#include "tjc_usart_hmi.h"

#include <stdio.h>
#include <string.h>

/**
 * @file tjc_usart_hmi.c
 * @brief 淘晶驰串口屏的独立UART驱动、命令发送和按键协议解析模块。
 *
 * 模块数据流：
 *
 *   淘晶驰屏TX -> PA1/UART4_RX -> UART4_IRQHandler
 *              -> 500字节环形缓冲区 -> TjcHmi_ReadEvent
 *              -> GSignalFlow业务事件
 *
 *   GSignalFlow/显示函数 -> 淘晶驰ASCII指令或addt原始数据
 *                       -> PA0/UART4_TX -> 淘晶驰屏RX
 *
 * 设计边界：
 * 1. UART4句柄、GPIO、外设时钟、NVIC和中断入口均封装在本文件。
 * 2. 不依赖CubeMX生成UART4代码，也不占用工程原有USART1调试串口。
 * 3. 中断只接收单字节并写入环形缓冲区，不在中断内解析协议或发送数据。
 * 4. 协议解析和所有阻塞发送均在裸机主循环上下文中执行。
 * 5. 移植到其他串口时，只需修改下面“专用串口硬件配置”宏。
 */

/*
 * 淘晶驰专用串口硬件配置全部集中在本文件，便于整体移植。
 * STM32F407ZG默认连接：PA0 -> 屏幕RX，PA1 <- 屏幕TX，公共GND。
 * 注意TX/RX必须交叉连接，屏幕与MCU必须共地并使用3.3V TTL电平。
 */
/* UART外设、IRQ入口和GPIO复用选择。 */
#define TJC_UART_INSTANCE               UART4
#define TJC_UART_IRQn                   UART4_IRQn
#define TJC_UART_IRQHandler             UART4_IRQHandler
#define TJC_UART_GPIO_PORT              GPIOA
#define TJC_UART_TX_PIN                 GPIO_PIN_0
#define TJC_UART_RX_PIN                 GPIO_PIN_1
#define TJC_UART_GPIO_AF                GPIO_AF8_UART4

/* 屏幕工程当前运行参数：921600波特率、8数据位、无校验、1停止位。 */
#define TJC_UART_BAUD_RATE              921600UL

/* 接收中断优先级与USART1保持同一等级，不调用RTOS接口。 */
#define TJC_UART_IRQ_PREEMPT_PRIORITY   5U
#define TJC_UART_IRQ_SUB_PRIORITY       0U

/* 外设时钟宏也放在本文件，移植时无需修改stm32f4xx_hal_msp.c。 */
#define TJC_UART_ENABLE_CLOCK()         __HAL_RCC_UART4_CLK_ENABLE()
#define TJC_UART_ENABLE_GPIO_CLOCK()    __HAL_RCC_GPIOA_CLK_ENABLE()

/*
 * 921600波特率发送256字节理论需要约2.8ms；保留较宽超时以容纳屏幕忙时。
 * 按钮文字延迟发送并周期重发，避免MCU启动快于7寸串口屏而丢失初始化命令。
 */
#define TJC_UART_TIMEOUT_MS             1000U
#define TJC_START_LABEL_DELAY_MS        2000U
#define TJC_START_LABEL_RETRY_MS        2000U
#define TJC_COMMAND_GAP_MS              2U
#define TJC_NUMERIC_RESPONSE_TIMEOUT_MS 50U
#define TJC_ADDT_RESPONSE_TIMEOUT_MS    100U

/*
 * 串口屏工程使用GB2312编码，不能直接依赖C源文件的UTF-8中文编码。
 * CA B1 D3 F2 B2 E2 C1 BF = “时域测量”
 * C7 D0 BB BB D6 DC C6 DA = “切换周期”
 */
#define TJC_TIME_LABEL_GB2312    "\xCA\xB1\xD3\xF2\xB2\xE2\xC1\xBF"
#define TJC_TOGGLE_LABEL_GB2312  "\xC7\xD0\xBB\xBB\xD6\xDC\xC6\xDA"
#define TJC_FREQUENCY_LABEL_GB2312 "\xC6\xB5\xD3\xF2\xB2\xE2\xC1\xBF"
#define TJC_COMPUTING_TEXT       "BUSY"
#define TJC_READY_TEXT           "READY"

/** UART4接收环形缓冲区；ISR写tail，主循环读head。 */
typedef struct
{
    volatile uint16_t head;   /**< 主循环下一次读取的位置。 */
    volatile uint16_t tail;   /**< 中断下一次写入的位置。 */
    volatile uint16_t length; /**< 当前已缓存但尚未处理的字节数。 */
    volatile uint8_t data[RINGBUFFER_LEN]; /**< 实际字节存储区。 */
} TjcRingBuffer;

static UART_HandleTypeDef s_TjcUart; /**< 模块私有UART4 HAL句柄。 */
static TjcRingBuffer s_RingBuffer;   /**< UART4中断接收缓冲区。 */
static uint8_t s_RxFrame[TJC_RX_FRAME_LENGTH]; /**< 7字节按键帧缓存。 */
static uint8_t s_RxFrameLength;      /**< 当前已收集的帧字节数。 */
static uint32_t s_NextStartLabelTick; /**< 下一次刷新静态文字的时刻。 */
static volatile uint32_t s_RxErrorCount; /**< ORE/NE/FE/PE错误计数。 */
static TjcHmiEvent s_PendingEvent; /**< get查询期间收到的按键事件。 */
static uint8_t s_PendingCycles;    /**< 延迟事件携带的周期参数。 */

/* 通用淘晶驰命令格式化缓冲区。所有调用都位于主循环，故不需要加锁。 */
char str1[TJC_TEXT_BUFFER_LENGTH];

static uint8_t TjcHmi_ParseEventByte(uint8_t byte,
                                    TjcHmiEvent *event,
                                    uint8_t *cycles);

/**
 * @brief 使用模块私有UART4阻塞发送一段数据。
 * @param data 待发送数据首地址。
 * @param length 数据长度，单位字节。
 * @return 1表示发送完成，0表示参数错误或HAL发送失败/超时。
 *
 * 波形的addt数据也是通过本函数发送。超时保留为1000ms，以兼容屏幕
 * 忙碌或后续临时降速调试；TJC_HMI_HOST_TEST模式下不访问真实硬件。
 */
static uint8_t TjcHmi_Send(const uint8_t *data, uint16_t length)
{
#ifdef TJC_HMI_HOST_TEST
    (void)data;
    (void)length;
    return 1U;
#else
    if ((data == NULL) || (length == 0U))
    {
        return 0U;
    }

    return (HAL_UART_Transmit(&s_TjcUart,
                              (uint8_t *)data,
                              length,
                              TJC_UART_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
#endif
}

/**
 * @brief 初始化淘晶驰专用UART4及其GPIO和接收中断。
 *
 * 初始化顺序：GPIOA/UART4时钟 -> PA0/PA1 AF8 -> UART参数 -> NVIC
 * -> RXNE中断。GPIO和时钟在HAL_UART_Init前显式完成，因此不要求工程的
 * HAL_UART_MspInit()了解UART4。
 */
static void TjcHmi_UartInit(void)
{
#ifndef TJC_HMI_HOST_TEST
    GPIO_InitTypeDef gpio = {0};

    TJC_UART_ENABLE_GPIO_CLOCK();
    TJC_UART_ENABLE_CLOCK();

    /* TX和RX均配置为AF8；RX使用上拉，避免屏幕未连接时输入悬空。 */
    gpio.Pin = TJC_UART_TX_PIN | TJC_UART_RX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = TJC_UART_GPIO_AF;
    HAL_GPIO_Init(TJC_UART_GPIO_PORT, &gpio);

    memset(&s_TjcUart, 0, sizeof(s_TjcUart));
    s_TjcUart.Instance = TJC_UART_INSTANCE;
    s_TjcUart.Init.BaudRate = TJC_UART_BAUD_RATE;
    s_TjcUart.Init.WordLength = UART_WORDLENGTH_8B;
    s_TjcUart.Init.StopBits = UART_STOPBITS_1;
    s_TjcUart.Init.Parity = UART_PARITY_NONE;
    s_TjcUart.Init.Mode = UART_MODE_TX_RX;
    s_TjcUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_TjcUart.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&s_TjcUart) != HAL_OK)
    {
        /* 初始化失败时保持模块静默；后续发送会由HAL返回失败。 */
        return;
    }

    /* 只启用接收非空中断，发送继续采用主循环中的阻塞HAL接口。 */
    HAL_NVIC_SetPriority(TJC_UART_IRQn,
                         TJC_UART_IRQ_PREEMPT_PRIORITY,
                         TJC_UART_IRQ_SUB_PRIORITY);
    HAL_NVIC_EnableIRQ(TJC_UART_IRQn);
    __HAL_UART_ENABLE_IT(&s_TjcUart, UART_IT_RXNE);
#endif
}

/**
 * @brief 从中断环形缓冲区弹出一个字节。
 * @param byte 用于保存读出字节的地址。
 * @return 缓冲区有数据返回1，空缓冲区或空指针返回0。
 *
 * head和length同时被UART4中断访问，因此生产固件中短暂关闭全局中断，
 * 保证一次弹出操作的原子性。函数会保存并恢复原PRIMASK，不会错误打开
 * 调用前已经关闭的中断。
 */
static uint8_t TjcHmi_ReadByte(uint8_t *byte)
{
#ifndef TJC_HMI_HOST_TEST
    uint32_t primask;
#endif

    if (byte == NULL)
    {
        return 0U;
    }

#ifndef TJC_HMI_HOST_TEST
    primask = __get_PRIMASK();
    __disable_irq();
#endif
    if (s_RingBuffer.length == 0U)
    {
#ifndef TJC_HMI_HOST_TEST
        __set_PRIMASK(primask);
#endif
        return 0U;
    }

    *byte = s_RingBuffer.data[s_RingBuffer.head];
    s_RingBuffer.head =
        (uint16_t)((s_RingBuffer.head + 1U) % RINGBUFFER_LEN);
    s_RingBuffer.length--;
#ifndef TJC_HMI_HOST_TEST
    __set_PRIMASK(primask);
#endif
    return 1U;
}

/* 等待淘晶驰透明传输应答：code FF FF FF，同时保留期间收到的按键事件。 */
static uint8_t TjcHmi_WaitTransparentResponse(uint8_t code)
{
    uint8_t matched = 0U;
    uint32_t deadline = HAL_GetTick() + TJC_ADDT_RESPONSE_TIMEOUT_MS;

    while ((int32_t)(HAL_GetTick() - deadline) < 0)
    {
        uint8_t byte;

        while (TjcHmi_ReadByte(&byte) != 0U)
        {
            if (matched == 0U)
            {
                if (byte == code)
                {
                    matched = 1U;
                }
                else
                {
                    TjcHmiEvent event = TJC_HMI_EVENT_NONE;
                    uint8_t cycles = 0U;

                    if ((TjcHmi_ParseEventByte(byte,
                                               &event,
                                               &cycles) != 0U) &&
                        (s_PendingEvent == TJC_HMI_EVENT_NONE))
                    {
                        s_PendingEvent = event;
                        s_PendingCycles = cycles;
                    }
                }
                continue;
            }

            if (byte == 0xFFU)
            {
                matched++;
                if (matched == 4U)
                {
                    return 1U;
                }
                continue;
            }

            matched = (byte == code) ? 1U : 0U;
        }

        HAL_Delay(1U);
    }

    return 0U;
}

/** @brief 发送每条淘晶驰ASCII指令必须携带的FF FF FF结束符。 */
static void TjcHmi_SendTerminator(void)
{
    static const uint8_t terminator[3] = {0xFFU, 0xFFU, 0xFFU};

    (void)TjcHmi_Send(terminator, sizeof(terminator));
}

/**
 * @brief 周期刷新按钮文字。
 *
 * 7寸屏上电通常慢于STM32，所以不能只在MCU初始化瞬间发送一次。该函数
 * 首次延迟2秒，之后每2秒重发b0/b1/b2文字；即使早期命令丢失，屏幕
 * 就绪后也会恢复。
 */
static void TjcHmi_UpdateButtonText(void)
{
    uint32_t now;

    now = HAL_GetTick();
    if ((int32_t)(now - s_NextStartLabelTick) < 0)
    {
        return;
    }

    tjc_send_txt("page0.b0", "txt", TJC_TIME_LABEL_GB2312);
    tjc_send_txt("page0.b1", "txt", TJC_TOGGLE_LABEL_GB2312);
    tjc_send_txt("page0.b2", "txt", TJC_FREQUENCY_LABEL_GB2312);
    s_NextStartLabelTick = now + TJC_START_LABEL_RETRY_MS;
}

/**
 * @brief 校验并解释已经收满的7字节淘晶驰按键帧。
 * @param event 返回业务事件类型。
 * @return 帧格式和命令号均有效返回1，否则返回0。
 *
 * 页面弹起事件协议：
 *   55 01 00 00 FF FF FF -> b0，时域测量
 *   55 02 00 00 FF FF FF -> b1，切换1/3周期
 *   55 03 00 00 FF FF FF -> b2，频域测量
 */
static uint8_t TjcHmi_DecodeFrame(TjcHmiEvent *event)
{
    if ((s_RxFrame[0] != 0x55U) ||
        (s_RxFrame[2] != 0x00U) ||
        (s_RxFrame[3] != 0x00U) ||
        (s_RxFrame[4] != 0xFFU) ||
        (s_RxFrame[5] != 0xFFU) ||
        (s_RxFrame[6] != 0xFFU))
    {
        return 0U;
    }

    if (s_RxFrame[1] == TJC_BUTTON_START)
    {
        *event = TJC_HMI_EVENT_START_MEASUREMENT;
        return 1U;
    }

    if (s_RxFrame[1] == TJC_BUTTON_TOGGLE_CYCLES)
    {
        *event = TJC_HMI_EVENT_TOGGLE_CYCLES;
        return 1U;
    }

    if (s_RxFrame[1] == TJC_BUTTON_FREQUENCY)
    {
        *event = TJC_HMI_EVENT_FREQUENCY_MEASUREMENT;
        return 1U;
    }

    return 0U;
}

/**
 * @brief 向按键帧解析器投递单个字节。
 * @return 刚好解析出一条业务事件时返回1。
 */
static uint8_t TjcHmi_ParseEventByte(uint8_t byte,
                                    TjcHmiEvent *event,
                                    uint8_t *cycles)
{
    if ((event == NULL) || (cycles == NULL))
    {
        return 0U;
    }

    if ((s_RxFrameLength == 0U) &&
        ((byte == (uint8_t)'1') || (byte == (uint8_t)'3')))
    {
        *cycles = (uint8_t)(byte - (uint8_t)'0');
        *event = TJC_HMI_EVENT_SET_CYCLES;
        return 1U;
    }

    if (s_RxFrameLength == 0U)
    {
        if (byte == 0x55U)
        {
            s_RxFrame[0] = byte;
            s_RxFrameLength = 1U;
        }
        return 0U;
    }

    /* 有效按键帧不含0x55，中途看到帧头时立即重同步。 */
    if (byte == 0x55U)
    {
        s_RxFrame[0] = byte;
        s_RxFrameLength = 1U;
        return 0U;
    }

    s_RxFrame[s_RxFrameLength] = byte;
    s_RxFrameLength++;
    if (s_RxFrameLength < TJC_RX_FRAME_LENGTH)
    {
        return 0U;
    }

    s_RxFrameLength = 0U;
    return TjcHmi_DecodeFrame(event);
}

/**
 * @brief 初始化整个淘晶驰模块。
 *
 * 调用位置为MX_GPIO_Init/MX_USART1_UART_Init之后。函数先清空解析状态和
 * 环形缓冲区，再独立初始化UART4；无需在main.c中增加MX_UART4_Init。
 */
void TjcHmi_Init(void)
{
    uint32_t now = HAL_GetTick();

    memset(s_RxFrame, 0, sizeof(s_RxFrame));
    s_RxFrameLength = 0U;
    s_NextStartLabelTick = now + TJC_START_LABEL_DELAY_MS;
    s_RxErrorCount = 0UL;
    s_PendingEvent = TJC_HMI_EVENT_NONE;
    s_PendingCycles = 0U;
    initRingBuffer();
    TjcHmi_UartInit();
    /* 从旧固件热更新时，先解除屏幕可能保留的暂停刷新状态。 */
    tjc_send_string("ref_star");
}

/**
 * @brief 在主循环中解析一条屏幕事件。
 * @param event 输出时域测量、频域测量、切换周期或直接设置周期事件。
 * @param cycles 仅TJC_HMI_EVENT_SET_CYCLES时输出1或3。
 * @return 解析到一条完整事件返回1，当前无完整事件返回0。
 *
 * 解析器会丢弃帧头0x55之前的噪声；收到完整非法帧后重新等待帧头。
 * ASCII字符'1'/'3'只在不处于二进制帧中时识别，避免误判帧内数据。
 */
uint8_t TjcHmi_ReadEvent(TjcHmiEvent *event, uint8_t *cycles)
{
    uint8_t byte;

    if ((event == NULL) || (cycles == NULL))
    {
        return 0U;
    }

    *event = TJC_HMI_EVENT_NONE;

    if (s_PendingEvent != TJC_HMI_EVENT_NONE)
    {
        *event = s_PendingEvent;
        *cycles = s_PendingCycles;
        s_PendingEvent = TJC_HMI_EVENT_NONE;
        s_PendingCycles = 0U;
        return 1U;
    }

    while (TjcHmi_ReadByte(&byte) != 0U)
    {
        if (TjcHmi_ParseEventByte(byte, event, cycles) != 0U)
        {
            /* 收到屏幕按键说明页面已经启动，下一轮立即重发静态文字。 */
            s_NextStartLabelTick = HAL_GetTick();
            return 1U;
        }

    }

    /* 仅在没有按键事件时处理按钮文字后台刷新。 */
    TjcHmi_UpdateButtonText();

    return 0U;
}

/** @brief 兼容原淘晶驰库：发送单个字符，不自动添加结束符。 */
void uart_send_char(char ch)
{
    uint8_t byte = (uint8_t)ch;

    (void)TjcHmi_Send(&byte, 1U);
}

/** @brief 兼容原淘晶驰库：发送C字符串，不自动添加结束符。 */
void uart_send_string(const char *str)
{
    if (str == NULL)
    {
        return;
    }

    (void)TjcHmi_Send((const uint8_t *)str, (uint16_t)strlen(str));
}

/**
 * @brief 发送一条完整淘晶驰ASCII指令。
 * @param str 不包含结束符的指令字符串。
 *
 * 本函数自动追加FF FF FF；调用者不能再手工追加结束符。
 */
void tjc_send_string(const char *str)
{
    uart_send_string(str);
    TjcHmi_SendTerminator();
    /* 921600下限制命令突发速度，给屏幕解释器留出处理时间。 */
    HAL_Delay(TJC_COMMAND_GAP_MS);
}

void TjcHmi_SetComputeBusy(uint8_t busy)
{
    TjcHmi_SetStatusText(
        (busy != 0U) ? TJC_COMPUTING_TEXT : TJC_READY_TEXT);
}

void TjcHmi_SetStatusText(const char *text)
{
    if (text != NULL)
    {
        tjc_send_txt("t6", "txt", text);
    }
}

static uint8_t TjcHmi_GetNumericValue(const char *expression,
                                     uint32_t *value)
{
    char command[32];
    uint8_t payload[4] = {0U, 0U, 0U, 0U};
    uint8_t payload_index = 0U;
    uint8_t terminator_count = 0U;
    uint8_t receiving = 0U;
    uint32_t deadline;
    int written;

    if ((expression == NULL) || (value == NULL))
    {
        return 0U;
    }

    written = snprintf(command, sizeof(command), "get %s", expression);
    if ((written <= 0) || ((size_t)written >= sizeof(command)))
    {
        return 0U;
    }

    tjc_send_string(command);
    deadline = HAL_GetTick() + TJC_NUMERIC_RESPONSE_TIMEOUT_MS;

    while ((int32_t)(HAL_GetTick() - deadline) < 0)
    {
        uint8_t byte;

        while (TjcHmi_ReadByte(&byte) != 0U)
        {
            if (receiving == 0U)
            {
                if (byte == 0x71U)
                {
                    receiving = 1U;
                    payload_index = 0U;
                    terminator_count = 0U;
                }
                else
                {
                    TjcHmiEvent event = TJC_HMI_EVENT_NONE;
                    uint8_t cycles = 0U;

                    if ((TjcHmi_ParseEventByte(byte,
                                               &event,
                                               &cycles) != 0U) &&
                        (s_PendingEvent == TJC_HMI_EVENT_NONE))
                    {
                        s_PendingEvent = event;
                        s_PendingCycles = cycles;
                    }
                }
                continue;
            }

            if (payload_index < 4U)
            {
                payload[payload_index++] = byte;
                continue;
            }

            if (byte == 0xFFU)
            {
                terminator_count++;
                if (terminator_count == 3U)
                {
                    uint32_t response_value =
                        (uint32_t)payload[0] |
                        ((uint32_t)payload[1] << 8U) |
                        ((uint32_t)payload[2] << 16U) |
                        ((uint32_t)payload[3] << 24U);

                    *value = response_value;
                    return 1U;
                }
                continue;
            }

            receiving = (byte == 0x71U) ? 1U : 0U;
            payload_index = 0U;
            terminator_count = 0U;
        }

        HAL_Delay(1U);
    }

    return 0U;
}

uint8_t TjcHmi_GetComponentWidth(const char *name, uint16_t *width)
{
    char expression[24];
    uint32_t value;
    int written;

    if ((name == NULL) || (width == NULL))
    {
        return 0U;
    }

    written = snprintf(expression, sizeof(expression), "%s.w", name);
    if ((written <= 0) || ((size_t)written >= sizeof(expression)) ||
        (TjcHmi_GetNumericValue(expression, &value) == 0U) ||
        (value < 16UL) || (value > 1024UL))
    {
        return 0U;
    }

    *width = (uint16_t)value;
    return 1U;
}

/**
 * @brief 设置控件的字符串属性。
 * @param objname 控件名，可包含页面前缀，如"page0.b0"。
 * @param attribute 字符串属性名，通常为"txt"。
 * @param txt 新文本内容；中文必须使用屏幕字库对应的GB2312字节。
 *
 * 生成示例：page0.b0.txt="时域测量" + FF FF FF。
 */
void tjc_send_txt(const char *objname,
                  const char *attribute,
                  const char *txt)
{
    int written;

    if ((objname == NULL) || (attribute == NULL) || (txt == NULL))
    {
        return;
    }

    written = snprintf(str1,
                       sizeof(str1),
                       "%s.%s=\"%s\"",
                       objname,
                       attribute,
                       txt);
    if ((written > 0) && ((size_t)written < sizeof(str1)))
    {
        tjc_send_string(str1);
    }
}

/**
 * @brief 设置控件的整数属性。
 * @param objname 控件名称。
 * @param attribute 数值属性名，例如"val"。
 * @param val 要写入的有符号整数。
 *
 * 生成示例：n0.val=123 + FF FF FF。
 */
void tjc_send_val(const char *objname, const char *attribute, int val)
{
    int written;

    if ((objname == NULL) || (attribute == NULL))
    {
        return;
    }

    written = snprintf(str1, sizeof(str1), "%s.%s=%d", objname, attribute, val);
    if ((written > 0) && ((size_t)written < sizeof(str1)))
    {
        tjc_send_string(str1);
    }
}

/**
 * @brief 发送指定长度的字符串并自动追加FF FF FF。
 * @note 适用于数据未以'\0'结尾但仍属于ASCII指令的场景。
 */
void tjc_send_nstring(const char *str, uint16_t str_length)
{
    if ((str == NULL) || (str_length == 0U))
    {
        return;
    }

    (void)TjcHmi_Send((const uint8_t *)str, str_length);
    TjcHmi_SendTerminator();
}

/**
 * @brief 清空接收环形缓冲区。
 * @note 正常仅在UART4中断启用前由TjcHmi_Init调用。
 */
void initRingBuffer(void)
{
    memset(&s_RingBuffer, 0, sizeof(s_RingBuffer));
}

/**
 * @brief 从UART4中断向环形缓冲区写入一个字节。
 * @param data UART4_DR读出的低8位数据。
 *
 * 缓冲区满时直接丢弃新字节，避免覆盖尚未解析的数据。函数不阻塞、
 * 不格式化字符串，满足中断服务函数必须短小的要求。
 */
void write1ByteToRingBuffer(uint8_t data)
{
    if (s_RingBuffer.length >= RINGBUFFER_LEN)
    {
        return;
    }

    s_RingBuffer.data[s_RingBuffer.tail] = data;
    s_RingBuffer.tail = (uint16_t)((s_RingBuffer.tail + 1U) % RINGBUFFER_LEN);
    s_RingBuffer.length++;
}

/**
 * @brief 从环形缓冲区头部删除指定数量的字节。
 * @param size 需要删除的字节数。
 * @note 这是原淘晶驰下载库的兼容API，当前协议解析器使用TjcHmi_ReadByte。
 */
void deleteRingBuffer(uint16_t size)
{
    if (size >= s_RingBuffer.length)
    {
        initRingBuffer();
        return;
    }

    s_RingBuffer.head = (uint16_t)((s_RingBuffer.head + size) % RINGBUFFER_LEN);
    s_RingBuffer.length = (uint16_t)(s_RingBuffer.length - size);
}

/**
 * @brief 查看环形缓冲区中指定相对位置的字节，但不删除数据。
 * @param position 相对于head的偏移，0表示队首字节。
 * @return 对应字节；越界时返回0。
 */
uint8_t read1ByteFromRingBuffer(uint16_t position)
{
    uint16_t real_position;

    if (position >= s_RingBuffer.length)
    {
        return 0U;
    }

    real_position = (uint16_t)((s_RingBuffer.head + position) % RINGBUFFER_LEN);
    return s_RingBuffer.data[real_position];
}

/** @return 当前环形缓冲区内尚未处理的字节数。 */
uint16_t getRingBufferLength(void)
{
    return s_RingBuffer.length;
}

/**
 * @brief 判断环形缓冲区是否仍有剩余空间。
 * @return 未满返回1，已经占满返回0。
 * @note 保留原库函数名称；名称中的Overflow与返回值语义相反，使用时注意。
 */
uint8_t isRingBufferOverflow(void)
{
    return (s_RingBuffer.length < RINGBUFFER_LEN) ? 1U : 0U;
}

/**
 * @brief UART4全局中断服务函数，由启动文件的UART4向量直接调用。
 *
 * 处理内容：
 * 1. 读取SR后只读取一次DR，同时完成RXNE及错误标志的硬件清除流程。
 * 2. RXNE有效时把接收字节压入环形缓冲区。
 * 3. ORE/NE/FE/PE有效时累计错误次数，便于调试观察。
 *
 * 本中断定义在本文件中，因此stm32f4xx_it.c无需声明huart4或调用
 * HAL_UART_IRQHandler，也不会与USART1的HAL回调发生冲突。
 */
void TJC_UART_IRQHandler(void)
{
#ifndef TJC_HMI_HOST_TEST
    uint32_t status = TJC_UART_INSTANCE->SR;

    if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE |
                   USART_SR_FE | USART_SR_PE)) != 0UL)
    {
        uint8_t byte = (uint8_t)(TJC_UART_INSTANCE->DR & 0xFFUL);

        if ((status & USART_SR_RXNE) != 0UL)
        {
            write1ByteToRingBuffer(byte);
        }
        if ((status & (USART_SR_ORE | USART_SR_NE |
                       USART_SR_FE | USART_SR_PE)) != 0UL)
        {
            s_RxErrorCount++;
        }
    }
#endif
}

/**
 * @brief 兼容页面：向page0.tN写入“整数.两位小数 名称”文本。
 * @param uart_handle 旧接口遗留参数，当前忽略，始终使用模块私有UART4。
 * @param counter 文本控件编号N。
 * @param num1 小数点前数字。
 * @param num2 小数点后数字。
 * @param name 文本后缀或单位。
 */
void printf_txt(UART_HandleTypeDef *uart_handle,
                int counter,
                int num1,
                int num2,
                const char *name)
{
    char object_name[16];
    char text[64];
    int object_written;
    int text_written;

    (void)uart_handle;
    if (name == NULL)
    {
        return;
    }

    object_written = snprintf(object_name,
                              sizeof(object_name),
                              "page0.t%d",
                              counter);
    text_written = snprintf(text,
                            sizeof(text),
                            "%d.%02d %s",
                            num1,
                            num2,
                            name);
    if ((object_written > 0) &&
        ((size_t)object_written < sizeof(object_name)) &&
        (text_written > 0) &&
        ((size_t)text_written < sizeof(text)))
    {
        tjc_send_txt(object_name, "txt", text);
    }
}

/** @brief 兼容旧接口：清除s0波形控件的指定通道。 */
void clean_wave(int ch)
{
    tjc_clear_wave("s0.id", ch);
}

/**
 * @brief 清除任意淘晶驰波形控件的指定通道。
 * @param name 波形控件ID表达式，例如"s0.id"或"s1.id"。
 * @param ch 波形通道号，当前页面使用0。
 *
 * 生成指令：cle s0.id,0 + FF FF FF。
 */
void tjc_clear_wave(const char *name, int ch)
{
    int written;

    if (name == NULL)
    {
        return;
    }

    written = snprintf(str1, sizeof(str1), "cle %s,%d", name, ch);

    if ((written > 0) && ((size_t)written < sizeof(str1)))
    {
        tjc_send_string(str1);
    }
}

/**
 * @brief 使用add指令向波形控件追加单个数据点。
 * @param uart_handle 旧接口遗留参数，当前忽略。
 * @param name 波形控件ID表达式。
 * @param ch 通道号。
 * @param data 纵坐标，淘晶驰波形控件通常接受0~255。
 * @note 大量数据应优先使用tjc_send_wave的addt批量方式。
 */
void printf_wave(UART_HandleTypeDef *uart_handle,
                 const char *name,
                 int ch,
                 int data)
{
    int written;

    (void)uart_handle;
    if (name == NULL)
    {
        return;
    }

    written = snprintf(str1, sizeof(str1), "add %s,%d,%d", name, ch, data);
    if ((written > 0) && ((size_t)written < sizeof(str1)))
    {
        tjc_send_string(str1);
    }
}

/**
 * @brief 使用淘晶驰addt协议批量发送波形或频谱数据。
 * @param name 波形控件ID表达式，例如"s0.id"。
 * @param ch 波形通道号。
 * @param data 已缩放为0~254的原始纵坐标数组。
 * @param count 数据点数量，当前按波形控件实际宽度发送。
 *
 * 发送时序：
 *   addt s0.id,0,256 + FF FF FF
 *   等待屏幕返回FE FF FF FF并进入透明数据接收状态
 *   连续发送256个原始字节
 *
 * 注意data阶段不是ASCII文本，不得使用strlen，也不能逐点添加引号或逗号。
 * addt按count自动结束透明传输，原始数据后不能再追加FF FF FF，否则这些
 * 字节会被屏幕当作下一条空指令。发送前必须收到FE就绪应答，发送后必须
 * 收到FD完成应答；等待期间收到的b0/b1/b2事件会暂存并交给业务层。
 * 921600波特率下256点约需2.8ms，底层仍保留1000ms容错超时。
 * @return FE/FD握手和原始数据发送全部成功返回1，否则返回0。
 */
uint8_t tjc_send_wave(const char *name,
                      int ch,
                      const uint8_t *data,
                      uint16_t count)
{
    int written;

    if ((name == NULL) || (data == NULL) || (count == 0U))
    {
        return 0U;
    }

    written = snprintf(str1,
                       sizeof(str1),
                       "addt %s,%d,%u",
                       name,
                       ch,
                       (unsigned int)count);
    if ((written <= 0) || ((size_t)written >= sizeof(str1)))
    {
        return 0U;
    }

    tjc_send_string(str1);
    if (TjcHmi_WaitTransparentResponse(0xFEU) == 0U)
    {
        return 0U;
    }
    if (TjcHmi_Send(data, count) == 0U)
    {
        return 0U;
    }
    if (TjcHmi_WaitTransparentResponse(0xFDU) == 0U)
    {
        return 0U;
    }

    HAL_Delay(TJC_COMMAND_GAP_MS);
    return 1U;

}
