#ifndef __G_SERIAL_H
#define __G_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 初始化G题串口适配层并启动单字节中断接收。
 * 调用前必须先执行MX_USART1_UART_Init()。
 */
void GSerial_Init(void);

/* 阻塞发送ASCII协议数据；成功返回1，失败返回0。 */
uint8_t GSerial_Transmit(const uint8_t *data,
                         uint16_t length,
                         uint32_t timeout_ms);

/* 从中断接收队列取出一个字节；有数据返回1，否则返回0。 */
uint8_t GSerial_ReadByte(uint8_t *byte);

#ifdef __cplusplus
}
#endif

#endif /* __G_SERIAL_H */
