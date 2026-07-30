#ifndef __TJC_USART_HMI_H
#define __TJC_USART_HMI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include <stdint.h>

/*
 * 淘晶驰b0/b1弹起事件固定发送7字节二进制帧：
 * 55 <命令号> 00 00 FF FF FF。
 */
#define TJC_RX_FRAME_LENGTH      7U
#define TJC_BUTTON_START          0x01U
#define TJC_BUTTON_TOGGLE_CYCLES  0x02U

/* UART4中断接收队列和ASCII指令格式化缓冲区大小。 */
#define RINGBUFFER_LEN           500U
#define TJC_TEXT_BUFFER_LENGTH   96U

/** 屏幕输入经过协议解析后交给GSignalFlow的业务事件。 */
typedef enum
{
    TJC_HMI_EVENT_NONE = 0,          /**< 当前没有完整事件。 */
    TJC_HMI_EVENT_START_MEASUREMENT, /**< b0：开始一次测量并保持结果。 */
    TJC_HMI_EVENT_TOGGLE_CYCLES,     /**< b1：在1/3周期之间切换。 */
    TJC_HMI_EVENT_SET_CYCLES         /**< ASCII 1/3：直接设置周期。 */
} TjcHmiEvent;

/* 公共格式化缓冲区，为兼容原淘晶驰例程保留。 */
extern char str1[TJC_TEXT_BUFFER_LENGTH];

/*
 * 初始化淘晶驰专用串口和按键帧解析状态。
 * 初始化后会延时设置b0为“开始测量”、b1为“切换周期”。
 */
void TjcHmi_Init(void);

/*
 * UART4中断入口由本模块直接提供，不需要加入stm32f4xx_it.c，也不要在
 * 其他文件重复定义同名函数。
 */
void UART4_IRQHandler(void);

/*
 * 从本模块UART4环形接收队列解析一次屏幕事件。
 * 支持：
 *   55 01 00 00 FF FF FF -> 开始测量
 *   55 02 00 00 FF FF FF -> 1/3周期切换
 * 同时保留ASCII字符'1'和'3'直接选择周期的无屏调试兼容。
 * 有事件返回1，无事件返回0。
 */
uint8_t TjcHmi_ReadEvent(TjcHmiEvent *event, uint8_t *cycles);

/* g9状态显示：busy=1显示BUSY，busy=0显示READY。 */
void TjcHmi_SetComputeBusy(uint8_t busy);
void TjcHmi_SetStatusText(const char *text);

/* 读取控件宽度属性，例如name="s0"；成功返回1。 */
uint8_t TjcHmi_GetComponentWidth(const char *name, uint16_t *width);

/* 在10~500kHz频谱峰下方绘制“Hn 频率”标签。 */
uint8_t TjcHmi_DrawSpectrumPeakLabel(uint32_t frequency_hundredths_hz,
                                    uint8_t harmonic,
                                    uint8_t row);

/*
 * 淘晶驰ASCII指令发送辅助函数：tjc_send_string/txt/val/nstring会自动
 * 追加FF FF FF；uart_send_char/string不会自动追加。
 */
void uart_send_char(char ch);
void uart_send_string(const char *str);
void tjc_send_string(const char *str);
void tjc_send_txt(const char *objname, const char *attribute, const char *txt);
void tjc_send_val(const char *objname, const char *attribute, int val);
void tjc_send_nstring(const char *str, uint16_t str_length);

/* 原下载库的环形缓冲区API，保留以兼容已有上层代码和主机测试。 */
void initRingBuffer(void);
void write1ByteToRingBuffer(uint8_t data);
void deleteRingBuffer(uint16_t size);
uint16_t getRingBufferLength(void);
uint8_t read1ByteFromRingBuffer(uint16_t position);
uint8_t isRingBufferOverflow(void);

#define usize       getRingBufferLength()
#define code_c()    initRingBuffer()
#define udelete(x)  deleteRingBuffer(x)
#define u(x)        read1ByteFromRingBuffer(x)

/*
 * 波形/文本兼容接口。tjc_send_wave使用addt透明批量传输，适合本工程的
 * 256点时域波形和256点频谱；printf_wave每次只发送一个点。
 */
void printf_txt(UART_HandleTypeDef *uart_handle,
                int counter,
                int num1,
                int num2,
                const char *name);
void printf_wave(UART_HandleTypeDef *uart_handle,
                 const char *name,
                 int ch,
                 int data);
void clean_wave(int ch);
void tjc_clear_wave(const char *name, int ch);
void tjc_send_wave(const char *name,
                   int ch,
                   const uint8_t *data,
                   uint16_t count);

#ifdef __cplusplus
}
#endif

#endif /* __TJC_USART_HMI_H */
