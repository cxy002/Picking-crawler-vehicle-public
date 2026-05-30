#include "sbus.h"
#include "usart.h"
#include <string.h>
#include "MoveControll.h"


#define SBUS_RS485_HEAD0       0x55U
#define SBUS_RS485_HEAD1       0xAAU
#define SBUS_RS485_TAIL        0x00U
#define SBUS_RS485_CRC_OFFSET  35U /* CRC校验覆盖第0~34字节 */

/* DMA事件缓存：HAL_UARTEx_RxEventCallback()收到一段数据后推入软件FIFO。 */
static uint8_t sbus_rx_buf[SBUS_RS485_DMA_RX_BUF_LEN];

/* 软件FIFO：中断里写入，主循环里的Sbus_Process_Task()读取并解析完整帧。 */
static uint8_t sbus_fifo[SBUS_RS485_FIFO_BUF_LEN];
static volatile uint16_t sbus_head;
static volatile uint16_t sbus_tail;

static uint8_t sbus_frame_buf[SBUS_RS485_FRAME_LEN];
static uint8_t sbus_frame_pos;
SBUS_RS485_Data_t sbus_data;
uint8_t sbus_data_ready = 0;     // 数据就绪标志


/**
  * @brief  计算软件FIFO的下一个位置。
  * @param  pos 当前FIFO下标。
  * @retval 回绕后的下一个FIFO下标。
  */
static uint16_t SBUS_RS485_NextFifoPos(uint16_t pos)
{
  pos++;
  if (pos >= SBUS_RS485_FIFO_BUF_LEN)
  {
    pos = 0U;
  }
  return pos;
}

/**
  * @brief  将DMA收到的一段数据写入软件FIFO。
  * @param  data 数据首地址。
  * @param  len  数据长度。
  * @retval 无。
  */
static void SBUS_RS485_Push(const uint8_t *data, uint16_t len)
{
  uint16_t i;
  uint16_t next;

  for (i = 0U; i < len; i++)
  {
    next = SBUS_RS485_NextFifoPos(sbus_head);
    if (next == sbus_tail)
    {
      /* FIFO满时丢掉最旧的1字节，保留最新数据，方便尽快重新同步帧头。 */
      sbus_tail = SBUS_RS485_NextFifoPos(sbus_tail);
      sbus_data.fifo_overflow_count++;
    }

    sbus_fifo[sbus_head] = data[i];
    sbus_head = next;
  }
}

/**
  * @brief  从软件FIFO读取1个字节。
  * @param  byte 输出字节指针。
  * @retval 1表示读取成功，0表示FIFO为空。
  */
static uint8_t SBUS_RS485_Pop(uint8_t *byte)
{
  uint8_t ret = 0U;

  __disable_irq();
  if (sbus_tail != sbus_head)
  {
    *byte = sbus_fifo[sbus_tail];
    sbus_tail = SBUS_RS485_NextFifoPos(sbus_tail);
    ret = 1U;
  }
  __enable_irq();

  return ret;
}

/**
  * @brief  计算MODBUS CRC16。
  * @param  data 待校验数据首地址。
  * @param  len  待校验数据长度。
  * @retval CRC16结果，协议帧中低字节在前。
  */
static uint16_t SBUS_RS485_Crc16Modbus(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t bit;

  for (i = 0U; i < len; i++)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return crc;
}

/**
  * @brief  读取高字节在前的int16_t数据。
  * @param  data 两字节数据首地址，data[0]为高字节。
  * @retval 转换后的int16_t数值。
  */
static int16_t SBUS_RS485_ReadInt16BE(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
}

/**
  * @brief  校验并解析一帧完整的SBUS-RS485数据。
  * @param  frame 38字节完整帧首地址。
  * @retval 无。
  */
static void SBUS_RS485_ParseFrame(const uint8_t *frame)
{
  uint16_t crc_calc;
  uint16_t crc_recv;
  uint8_t i;
  uint8_t digital;

  if (frame[SBUS_RS485_FRAME_LEN - 1U] != SBUS_RS485_TAIL)
  {
    sbus_data.frame_error_count++;
    return;
  }

  crc_calc = SBUS_RS485_Crc16Modbus(frame, SBUS_RS485_CRC_OFFSET);
  crc_recv = (uint16_t)frame[SBUS_RS485_CRC_OFFSET] |
             ((uint16_t)frame[SBUS_RS485_CRC_OFFSET + 1U] << 8U);
  if (crc_calc != crc_recv)
  {
    sbus_data.crc_error_count++;
    return;
  }

  /* CH1从第2字节开始，每个模拟通道都是高字节在前，例如0x07FF发送为07 FF。 */
  for (i = 0U; i < SBUS_RS485_ANALOG_CHANNELS; i++)
  {
    sbus_data.analog[i] = SBUS_RS485_ReadInt16BE(&frame[2U + (uint16_t)i * 2U]);
  }

  /* 数字通道低4位：bit0=CH17，bit1=CH18，bit2=Link，bit3=Error。 */
  digital = frame[34U] & 0x0FU;
  sbus_data.digital_raw = digital;
  sbus_data.ch17 = digital & 0x01U;
  sbus_data.ch18 = (digital >> 1U) & 0x01U;
  sbus_data.link_lost = (digital >> 2U) & 0x01U;
  sbus_data.error = (digital >> 3U) & 0x01U;
  sbus_data.frame_count++;
  sbus_data.last_frame_tick = HAL_GetTick();
}

/**
  * @brief  向帧状态机输入1个字节，自动寻找55 AA帧头并凑满38字节。
  * @param  byte 新收到的1个字节。
  * @retval 无。
  */
static void SBUS_RS485_FeedByte(uint8_t byte)
{
  if (sbus_frame_pos == 0U)
  {
    /* 等待帧头第1字节0x55。 */
    if (byte == SBUS_RS485_HEAD0)
    {
      sbus_frame_buf[sbus_frame_pos++] = byte;
    }
    return;
  }

  if (sbus_frame_pos == 1U)
  {
    /* 等待帧头第2字节0xAA；如果再次收到0x55，则保持在等待0xAA状态。 */
    if (byte == SBUS_RS485_HEAD1)
    {
      sbus_frame_buf[sbus_frame_pos++] = byte;
    }
    else if (byte == SBUS_RS485_HEAD0)
    {
      sbus_frame_buf[0] = byte;
      sbus_frame_pos = 1U;
    }
    else
    {
      sbus_frame_pos = 0U;
    }
    return;
  }

  sbus_frame_buf[sbus_frame_pos++] = byte;
  if (sbus_frame_pos >= SBUS_RS485_FRAME_LEN)
  {
    SBUS_RS485_ParseFrame(sbus_frame_buf);
		sbus_data_ready = 1;
    sbus_frame_pos = 0U;
  }
}

/**
  * @brief  清理串口接收状态并重新启动USART3 ReceiveToIdle DMA接收。
  * @param  无。
  * @retval HAL状态。
  */
static HAL_StatusTypeDef SBUS_RS485_RestartRx(void)
{
  HAL_StatusTypeDef status;

  HAL_UART_AbortReceive(&huart3);

  __HAL_UART_CLEAR_IDLEFLAG(&huart3);
  __HAL_UART_CLEAR_OREFLAG(&huart3);
  __HAL_UART_CLEAR_FEFLAG(&huart3);
  __HAL_UART_CLEAR_NEFLAG(&huart3);
  __HAL_UART_CLEAR_PEFLAG(&huart3);
  huart3.ErrorCode = HAL_UART_ERROR_NONE;

  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buf, sizeof(sbus_rx_buf));
  if (huart3.hdmarx != NULL)
  {
    /* 只使用空闲/完成事件，半满事件不解析数据，减少中断次数。 */
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
  }

  if (status != HAL_OK)
  {
    sbus_data.dma_restart_fail_count++;
  }

  return status;
}

/**
  * @brief  初始化SBUS-RS485接收模块。
  * @param  无。
  * @retval HAL状态。
  */
HAL_StatusTypeDef SBUS_RS485_Init(void)
{
  memset(&sbus_data, 0, sizeof(sbus_data));
  memset(sbus_rx_buf, 0, sizeof(sbus_rx_buf));
  memset(sbus_fifo, 0, sizeof(sbus_fifo));
  memset(sbus_frame_buf, 0, sizeof(sbus_frame_buf));

  sbus_head = 0U;
  sbus_tail = 0U;
  sbus_frame_pos = 0U;

  return SBUS_RS485_RestartRx();
}

/**
  * @brief  从软件FIFO取出数据并执行帧解析，应在主循环中周期调用。
  * @param  无。
  * @retval 无。
  */
void SBUS_RS485_Update(void)
{
  uint8_t byte;

  while (SBUS_RS485_Pop(&byte) != 0U)
  {
    SBUS_RS485_FeedByte(byte);
  }
}

/**
  * @brief  SBUS模块初始化外层接口。
  * @param  无。
  * @retval 无。
  */
void Sbus_Init(void)
{
  (void)SBUS_RS485_Init();
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // RS485 接收模式
}

/**
  * @brief  SBUS模块处理任务外层接口，在主循环中调用。
  * @param  无。
  * @retval 无。
  */
void Sbus_Process_Task(void)
{
  SBUS_RS485_Update();
	
	if(sbus_data_ready)
    {
        sbus_data_ready = 0;  // 清除标志
				RCGetValue();
    }
}

/**
  * @brief  获取最近一次解析出的SBUS-RS485数据。
  * @param  无。
  * @retval 数据结构只读指针，外部只能读取不能修改。
  */
const SBUS_RS485_Data_t *SBUS_RS485_GetData(void)
{
  return &sbus_data;
}

/**
  * @brief  判断SBUS-RS485是否在线。
  * @param  timeout_ms 超时时间，单位ms。
  * @retval 1表示在线，0表示超时或尚未收到有效帧。
  */
uint8_t SBUS_RS485_IsOnline(uint32_t timeout_ms)
{
  if (sbus_data.frame_count == 0U)
  {
    return 0U;
  }

  return ((HAL_GetTick() - sbus_data.last_frame_tick) <= timeout_ms) ? 1U : 0U;
}

/**
  * @brief  UART ReceiveToIdle DMA接收事件回调。
  * @param  huart UART句柄。
  * @param  Size  本次DMA收到的字节数，从sbus_rx_buf[0]开始有效。
  * @retval 无。
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART3)
  {
    if (Size > sizeof(sbus_rx_buf))
    {
      Size = sizeof(sbus_rx_buf);
    }

    sbus_data.rx_event_count++;
    SBUS_RS485_Push(sbus_rx_buf, Size);
    (void)SBUS_RS485_RestartRx();
  }
}

/**
  * @brief  UART错误回调，记录错误并重启USART1 DMA接收。
  * @param  huart UART句柄。
  * @retval 无。
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    sbus_data.uart_error_count++;

    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0U)
    {
      sbus_data.uart_ore_count++;
    }
    if ((huart->ErrorCode & HAL_UART_ERROR_FE) != 0U)
    {
      sbus_data.uart_fe_count++;
    }
    if ((huart->ErrorCode & HAL_UART_ERROR_NE) != 0U)
    {
      sbus_data.uart_ne_count++;
    }
    if ((huart->ErrorCode & HAL_UART_ERROR_PE) != 0U)
    {
      sbus_data.uart_pe_count++;
    }

    (void)SBUS_RS485_RestartRx();
  }
}



