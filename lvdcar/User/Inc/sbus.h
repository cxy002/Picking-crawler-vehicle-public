#ifndef __SBUS_H__
#define __SBUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define SBUS_RS485_ANALOG_CHANNELS  16U  /* 模拟通道 CH1~CH16 */
#define SBUS_RS485_FRAME_LEN        38U  /* 55 AA + 16路int16 + 数字通道 + CRC16 + 00 */
#define SBUS_RS485_DMA_RX_BUF_LEN   128U /* 串口1 DMA事件接收缓存长度 */
#define SBUS_RS485_FIFO_BUF_LEN     256U /* 软件FIFO长度 */

typedef struct
{
  int16_t analog[SBUS_RS485_ANALOG_CHANNELS]; /* CH1~CH16，高字节在前int16_t */
  uint8_t ch17;                               /* 数字通道 bit0 */
  uint8_t ch18;                               /* 数字通道 bit1 */
  uint8_t link_lost;                          /* 数字通道 bit2：1表示链路中断 */
  uint8_t error;                              /* 数字通道 bit3：1表示通讯错误 */
  uint8_t digital_raw;                        /* 数字通道低4位原始值 */
  uint32_t frame_count;                       /* 已解析的有效帧数量 */
  uint32_t crc_error_count;                   /* CRC16校验失败计数 */
  uint32_t frame_error_count;                 /* 帧尾错误计数 */
  uint32_t fifo_overflow_count;               /* 软件FIFO溢出计数 */
  uint32_t rx_event_count;                    /* 串口接收事件计数 */
  uint32_t dma_restart_fail_count;            /* DMA重启失败计数 */
  uint32_t uart_error_count;                  /* 串口错误总计数 */
  uint32_t uart_ore_count;                    /* 串口溢出错误计数 */
  uint32_t uart_fe_count;                     /* 串口帧错误计数 */
  uint32_t uart_ne_count;                     /* 串口噪声错误计数 */
  uint32_t uart_pe_count;                     /* 串口校验错误计数 */
  uint32_t last_frame_tick;                   /* 最近一次有效帧的HAL时间戳 */
} SBUS_RS485_Data_t;

extern SBUS_RS485_Data_t sbus_data;
extern uint8_t sbus_data_ready;     // 数据就绪标志



HAL_StatusTypeDef SBUS_RS485_Init(void);
void SBUS_RS485_Update(void);
const SBUS_RS485_Data_t *SBUS_RS485_GetData(void);
uint8_t SBUS_RS485_IsOnline(uint32_t timeout_ms);

void Sbus_Init(void);
void Sbus_Process_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __SBUS_H__ */
