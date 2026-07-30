#ifndef __FPGA_LINK_H
#define __FPGA_LINK_H

/*
 * FpgaLink.h
 *
 * 模块功能：
 * 1. 对外提供 STM32 与 FPGA 之间的最小通信接口。
 * 2. 当前阶段固定使用 SPI1 外设和 PA4 软件片选。
 * 3. 上层业务只关心 32 bit 命令字，不直接处理 16 bit 半字拆分和 CS 时序。
 *
 * 分层说明：
 * - Core 层负责 CubeMX 生成的 SPI1/GPIO 初始化。
 * - FpgaLink 层负责 FPGA_CS 拉低/拉高和 32 bit 事务封装。
 * - GSignalFlow 层负责决定何时读取 FPGA ID、启动采集和读取样本。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* HAL 基础头文件：提供 uint8_t、GPIO、SPI 等 HAL 相关类型声明。 */
#include "stm32f4xx_hal.h"

/* 标准整数类型头文件：提供 uint32_t 等固定宽度整数类型。 */
#include <stdint.h>

/* 读取 FPGA ID 的命令字。当前协议规定先发送该命令，再发送 NOP 读取返回值。 */
#define FPGA_CMD_READ_ID             0x20000000UL

/* 空操作命令字。用于第二帧读取 FPGA 对上一帧命令的返回数据。 */
#define FPGA_CMD_NOP                 0x00000000UL

/* 当前文档约定的 FPGA 期望 ID。只有读回该值时，业务层才认为 FPGA 在线。 */
#define FPGA_EXPECTED_ID             0x000004D2UL

/* 2026 G 题最小闭环命令。响应均在下一帧 SPI 事务读回。 */
#define FPGA_CMD_READ_STATUS         0x21000000UL
#define FPGA_CMD_READ_SAMPLE_BASE    0x22000000UL
#define FPGA_CMD_START_ANALYSIS      0x30000000UL
#define FPGA_CMD_START_WAVEFORM      0x31000000UL

/* 每个样本响应高 16 bit 的同步标记，低 16 bit 为有符号采样值。 */
#define FPGA_SAMPLE_RESPONSE_TAG     0x5341U

/* G 题最小闭环固定帧长。 */
#define FPGA_CAPTURE_FRAME_LENGTH    4096U

/* 状态寄存器低位定义。 */
#define FPGA_CAPTURE_STATUS_DONE     (1UL << 0U)
#define FPGA_CAPTURE_STATUS_BUSY     (1UL << 1U)
#define FPGA_CAPTURE_STATUS_WAVEFORM (1UL << 2U)

/* SPI 阻塞式传输默认超时时间，单位 ms。 */
#define FPGA_LINK_DEFAULT_TIMEOUT_MS 10UL

/* SPI链路容错：短事务重试及软件片选的明确时序裕量。 */
#define FPGA_LINK_RETRY_COUNT         3U
#define FPGA_LINK_CS_HIGH_US          2U
#define FPGA_LINK_CS_SETUP_US         1U
#define FPGA_LINK_CS_HOLD_US          1U

/*
 * 函数功能：初始化 FPGA 通信模块。
 * 作用说明：当前只做防御性处理，把 FPGA_CS 拉高，确保总线处于空闲状态。
 */
void    FpgaLink_Init(void);

/*
 * 函数功能：执行一次 32 bit FPGA SPI 事务。
 * 参数说明：
 * - tx_word：要发送给 FPGA 的 32 bit 命令或数据。
 * - rx_word：接收 32 bit 返回值的指针；允许为 NULL，表示不关心返回值。
 * - timeout_ms：HAL_SPI_TransmitReceive 的超时时间，单位 ms。
 * 返回值：
 * - 1：HAL SPI 事务成功。
 * - 0：HAL SPI 事务失败，失败时仍保证 FPGA_CS 恢复高电平。
 */
uint8_t FpgaLink_Transfer32(uint32_t tx_word, uint32_t *rx_word, uint32_t timeout_ms);

/*
 * 函数功能：读取并校验 FPGA ID。
 * 参数说明：
 * - id：保存读回 ID 的指针；允许为 NULL。
 * 返回值：
 * - 1：两帧传输成功，且读回值等于 FPGA_EXPECTED_ID。
 * - 0：传输失败或读回 ID 不匹配。
 */
uint8_t Fpga_ReadId(uint32_t *id);

/*
 * 启动一帧采集。
 * waveform_mode=0：1.25 MSPS 分析帧；waveform_mode!=0：5 MSPS 波形帧。
 */
uint8_t Fpga_StartCapture(uint8_t waveform_mode);

/* 读取采集状态；返回的低位使用 FPGA_CAPTURE_STATUS_* 定义。 */
uint8_t Fpga_ReadCaptureStatus(uint32_t *status);

/*
 * 读取已完成的有符号 16 bit 帧。
 * sample_count 当前必须为 1..FPGA_CAPTURE_FRAME_LENGTH。
 */
uint8_t Fpga_ReadCaptureFrame(int16_t *samples, uint16_t sample_count);

#ifdef __cplusplus
}
#endif

#endif /* __FPGA_LINK_H */
