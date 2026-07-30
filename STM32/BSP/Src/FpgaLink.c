#include "FpgaLink.h"
#include "main.h"
#include "spi.h"

/*
 * FpgaLink.c
 *
 * 模块功能：
 * 1. 封装 STM32F407 与 FPGA 的 SPI1 通信。
 * 2. 将上层传入的 32 bit 命令拆成两个 16 bit 半字发送。
 * 3. 统一维护 FPGA_CS 的片选时序，避免上层业务直接操作片选引脚。
 *
 * 当前硬件连接：
 * - PA4：FPGA_CS，软件片选，低电平选中 FPGA。
 * - PA5：SPI1_SCK。
 * - PA6：SPI1_MISO。
 * - PA7：SPI1_MOSI。
 *
 * 当前实现边界：
 * - 本模块只负责最小通信链路和 ID 读取。
 * - 不负责串口报告或显示业务。
 * - 不负责滤波器类型、频率档位、IIR 系数等业务含义。
 */

/*
 * 函数功能：选中 FPGA。
 * 作用说明：把 FPGA_CS 拉低，通知 FPGA 当前 SPI 事务开始。
 */
static void FpgaLink_Select(void);

/*
 * 函数功能：取消选中 FPGA。
 * 作用说明：把 FPGA_CS 拉高，通知 FPGA 当前 SPI 事务结束或总线空闲。
 */
static void FpgaLink_Deselect(void);

/* 微秒级软件片选时序和SPI异常恢复。 */
static void FpgaLink_DelayUs(uint32_t delay_us);
static void FpgaLink_RecoverBus(void);
static uint8_t FpgaLink_ReadCaptureFrameOnce(int16_t *samples,
                                            uint16_t sample_count);

#define FPGA_RESPONSE_ANALYSIS_STARTED  0x414E4C59UL
#define FPGA_RESPONSE_WAVEFORM_STARTED  0x57415645UL

static uint8_t s_CycleCounterReady;

/*
 * 函数功能：初始化 FPGA 通信模块。
 * 作用说明：
 * 1. 独立初始化 PA4 软件片选，使 G 最小闭环无需启动整套板级 GPIO。
 * 2. 配置输出模式前先把输出锁存器置高，避免产生误选中的低脉冲。
 */
void FpgaLink_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    __HAL_RCC_GPIOA_CLK_ENABLE();
    FpgaLink_Deselect();

    GPIO_InitStruct.Pin = FPGA_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(FPGA_CS_GPIO_Port, &GPIO_InitStruct);

    /* 使用Cortex-M4周期计数器产生不依赖SysTick的微秒级片选间隔。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0UL;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_CycleCounterReady =
        ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0UL) ? 1U : 0U;
    FpgaLink_DelayUs(FPGA_LINK_CS_HIGH_US);
}

/*
 * 函数功能：执行一次 32 bit SPI 收发事务。
 *
 * 参数说明：
 * - tx_word：要发送给 FPGA 的 32 bit 数据，高 16 bit 先发送，低 16 bit 后发送。
 * - rx_word：用于保存 FPGA 返回的 32 bit 数据；如果传入 NULL，则丢弃返回数据。
 * - timeout_ms：HAL 阻塞式传输超时时间，单位 ms。
 *
 * 返回值：
 * - 1：HAL_SPI_TransmitReceive() 返回 HAL_OK，事务成功。
 * - 0：HAL_SPI_TransmitReceive() 返回失败，事务失败。
 *
 * 时序保证：
 * 1. 传输前调用 FpgaLink_Select() 拉低 CS。
 * 2. 无论传输成功还是失败，都会调用 FpgaLink_Deselect() 拉高 CS。
 * 3. 因此上层不需要也不应该直接操作 FPGA_CS。
 */
uint8_t FpgaLink_Transfer32(uint32_t tx_word, uint32_t *rx_word, uint32_t timeout_ms)
{
    /* SPI1 当前配置为 16 bit 数据宽度，因此 32 bit 数据要拆成两个半字。 */
    uint16_t tx_buf[2];

    /* rx_buf 保存 FPGA 返回的两个 16 bit 半字，默认清零便于失败路径处理。 */
    uint16_t rx_buf[2] = { 0U, 0U };

    /* HAL SPI 传输返回状态，用于判断本次事务是否成功。 */
    HAL_StatusTypeDef status;

    /* 高 16 bit 先发，保持与文档中 32 bit 命令字 MSB first 的语义一致。 */
    tx_buf[0] = (uint16_t)((tx_word >> 16U) & 0xFFFFU);

    /* 低 16 bit 后发。 */
    tx_buf[1] = (uint16_t)(tx_word & 0xFFFFU);

    /*
     * FPGA在50MHz域同步CS。每帧前明确保持CS高电平，再提供CS建立时间，
     * 防止相邻HAL事务间的高脉冲过窄而被FPGA同步器漏采。
     */
    FpgaLink_Deselect();
    FpgaLink_DelayUs(FPGA_LINK_CS_HIGH_US);
    FpgaLink_Select();
    FpgaLink_DelayUs(FPGA_LINK_CS_SETUP_US);

    /*
     * HAL_SPI_TransmitReceive 的 Size 参数在 16 bit 模式下表示半字数量。
     * 这里 Size=2，表示连续传输两个 16 bit 半字，即完整 32 bit。
     */
    status = HAL_SPI_TransmitReceive(&hspi1,
                                      (uint8_t *)tx_buf,
                                      (uint8_t *)rx_buf,
                                      2U,
                                      timeout_ms);

    /* 最后一位SCLK后保持CS低电平，使FPGA有时间提交rx_valid。 */
    FpgaLink_DelayUs(FPGA_LINK_CS_HOLD_US);
    FpgaLink_Deselect();

    if (status != HAL_OK)
    {
        FpgaLink_RecoverBus();
        /* 若调用者关心返回值，失败时显式写 0，避免上层读到旧数据。 */
        if (rx_word != NULL)
        {
            *rx_word = 0UL;
        }
        return 0U;
    }

    /* 调用者传入 rx_word 时，把两个 16 bit 返回半字重新组合成 32 bit。 */
    if (rx_word != NULL)
    {
        *rx_word = (((uint32_t)rx_buf[0]) << 16U) | ((uint32_t)rx_buf[1]);
    }

    return 1U;
}

/*
 * 函数功能：读取 FPGA ID 并判断 FPGA 是否在线。
 *
 * 参数说明：
 * - id：用于保存第二帧读回的 32 bit ID；允许传入 NULL。
 *
 * 返回值：
 * - 1：两帧 SPI 事务均成功，且第二帧读回 FPGA_EXPECTED_ID。
 * - 0：任一 SPI 事务失败，或读回 ID 与 FPGA_EXPECTED_ID 不一致。
 *
 * 协议说明：
 * 1. 第一帧发送 FPGA_CMD_READ_ID，请求 FPGA 准备 ID 返回值。
 * 2. 第二帧发送 FPGA_CMD_NOP，同时读取 FPGA 返回的 ID。
 * 3. 不能只根据 HAL SPI 成功判断 FPGA 在线，必须校验实际 ID。
 */
uint8_t Fpga_ReadId(uint32_t *id)
{
    /* 第二帧读回的 32 bit 数据，后续用于和 FPGA_EXPECTED_ID 比较。 */
    uint32_t rx_word = 0UL;

    /* 先把输出 ID 清零，保证失败时上层不会误用旧 ID。 */
    if (id != NULL)
    {
        *id = 0UL;
    }

    {
        uint8_t attempt;

        for (attempt = 0U; attempt < FPGA_LINK_RETRY_COUNT; attempt++)
        {
            if ((FpgaLink_Transfer32(FPGA_CMD_READ_ID,
                                     NULL,
                                     FPGA_LINK_DEFAULT_TIMEOUT_MS) != 0U) &&
                (FpgaLink_Transfer32(FPGA_CMD_NOP,
                                     &rx_word,
                                     FPGA_LINK_DEFAULT_TIMEOUT_MS) != 0U) &&
                (rx_word == FPGA_EXPECTED_ID))
            {
                if (id != NULL)
                {
                    *id = rx_word;
                }
                return 1U;
            }
            FpgaLink_DelayUs(10U);
        }
    }

    if (id != NULL)
    {
        *id = rx_word;
    }
    return 0U;
}

uint8_t Fpga_StartCapture(uint8_t waveform_mode)
{
    uint32_t command = (waveform_mode != 0U)
                     ? FPGA_CMD_START_WAVEFORM
                     : FPGA_CMD_START_ANALYSIS;
    uint32_t expected_response = (waveform_mode != 0U)
                               ? FPGA_RESPONSE_WAVEFORM_STARTED
                               : FPGA_RESPONSE_ANALYSIS_STARTED;
    uint32_t rx_word = 0UL;
    uint8_t attempt;

    for (attempt = 0U; attempt < FPGA_LINK_RETRY_COUNT; attempt++)
    {
        if ((FpgaLink_Transfer32(command,
                                 NULL,
                                 FPGA_LINK_DEFAULT_TIMEOUT_MS) != 0U) &&
            (FpgaLink_Transfer32(FPGA_CMD_NOP,
                                 &rx_word,
                                 FPGA_LINK_DEFAULT_TIMEOUT_MS) != 0U) &&
            (rx_word == expected_response))
        {
            return 1U;
        }
        FpgaLink_DelayUs(10U);
    }

    return 0U;
}

uint8_t Fpga_ReadCaptureStatus(uint32_t *status)
{
    uint32_t rx_word = 0UL;
    uint8_t attempt;

    if (status == NULL)
    {
        return 0U;
    }

    *status = 0UL;

    for (attempt = 0U; attempt < FPGA_LINK_RETRY_COUNT; attempt++)
    {
        if ((FpgaLink_Transfer32(FPGA_CMD_READ_STATUS,
                                 NULL,
                                 FPGA_LINK_DEFAULT_TIMEOUT_MS) != 0U) &&
            (FpgaLink_Transfer32(FPGA_CMD_NOP,
                                 &rx_word,
                                 FPGA_LINK_DEFAULT_TIMEOUT_MS) != 0U) &&
            ((uint16_t)(rx_word >> 16U) == FPGA_CAPTURE_FRAME_LENGTH) &&
            ((rx_word & 0x0000FFF8UL) == 0UL))
        {
            *status = rx_word;
            return 1U;
        }
        FpgaLink_DelayUs(10U);
    }

    return 0U;
}

uint8_t Fpga_ReadCaptureFrame(int16_t *samples, uint16_t sample_count)
{
    uint8_t attempt;

    if ((samples == NULL) ||
        (sample_count == 0U) ||
        (sample_count > FPGA_CAPTURE_FRAME_LENGTH))
    {
        return 0U;
    }

    for (attempt = 0U; attempt < FPGA_LINK_RETRY_COUNT; attempt++)
    {
        if (FpgaLink_ReadCaptureFrameOnce(samples, sample_count) != 0U)
        {
            return 1U;
        }
        FpgaLink_RecoverBus();
        FpgaLink_DelayUs(10U);
    }

    return 0U;
}

static uint8_t FpgaLink_ReadCaptureFrameOnce(int16_t *samples,
                                            uint16_t sample_count)
{
    uint32_t rx_word = 0UL;
    uint16_t index;

    /*
     * RAM 读取响应采用一帧流水：
     * 第一次只提交地址 0；之后每次提交下一个地址，同时接收上一个样本。
     */
    if (FpgaLink_Transfer32(FPGA_CMD_READ_SAMPLE_BASE,
                            NULL,
                            FPGA_LINK_DEFAULT_TIMEOUT_MS) == 0U)
    {
        return 0U;
    }

    for (index = 0U; index + 1U < sample_count; index++)
    {
        uint32_t next_command = FPGA_CMD_READ_SAMPLE_BASE |
                                ((uint32_t)(index + 1U) & 0x0FFFUL);

        if (FpgaLink_Transfer32(next_command,
                                &rx_word,
                                FPGA_LINK_DEFAULT_TIMEOUT_MS) == 0U)
        {
            return 0U;
        }

        if ((uint16_t)(rx_word >> 16U) != FPGA_SAMPLE_RESPONSE_TAG)
        {
            return 0U;
        }

        samples[index] = (int16_t)(rx_word & 0xFFFFUL);
    }

    if (FpgaLink_Transfer32(FPGA_CMD_NOP,
                            &rx_word,
                            FPGA_LINK_DEFAULT_TIMEOUT_MS) == 0U)
    {
        return 0U;
    }

    if ((uint16_t)(rx_word >> 16U) != FPGA_SAMPLE_RESPONSE_TAG)
    {
        return 0U;
    }

    samples[sample_count - 1U] = (int16_t)(rx_word & 0xFFFFUL);
    return 1U;
}

static void FpgaLink_DelayUs(uint32_t delay_us)
{
    if ((delay_us == 0UL) || (s_CycleCounterReady == 0U))
    {
        volatile uint32_t fallback = delay_us * 32UL;

        while (fallback-- != 0UL)
        {
            __NOP();
        }
        return;
    }

    {
        uint32_t cycles_per_us = SystemCoreClock / 1000000UL;
        uint32_t start = DWT->CYCCNT;
        uint32_t required = cycles_per_us * delay_us;

        while ((uint32_t)(DWT->CYCCNT - start) < required)
        {
            __NOP();
        }
    }
}

static void FpgaLink_RecoverBus(void)
{
    FpgaLink_Deselect();
    FpgaLink_DelayUs(10U);

    (void)HAL_SPI_Abort(&hspi1);
    __HAL_SPI_DISABLE(&hspi1);
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
    __HAL_SPI_ENABLE(&hspi1);

    FpgaLink_DelayUs(FPGA_LINK_CS_HIGH_US);
}

/*
 * 函数功能：拉低 FPGA_CS。
 * 作用说明：低电平片选有效，表示 FPGA 被当前 SPI 事务选中。
 */
static void FpgaLink_Select(void)
{
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_RESET);
}

/*
 * 函数功能：拉高 FPGA_CS。
 * 作用说明：高电平片选无效，表示 SPI 总线处于空闲状态或本次事务结束。
 */
static void FpgaLink_Deselect(void)
{
    HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_SET);
}
