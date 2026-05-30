#include "Navcmd.h"
#include "main.h"
#include "string.h"



uint8_t serial2_frame_index = 0U;
uint8_t serial2_frame_buf[SERIAL_PACKET_SIZE] = {0};
uint8_t uart1_rx_byte[1];
float wheel_target_velocity = 0.0f;
float wheel_target_angular_velocity = 0.0f;
volatile uint8_t serial_pose_new = 0U;
volatile SerialPosePacket_t serial_pose_latest = {0};
SerialPosePacket_t NAV_Diff_CMD;
Serial2RxState_t serial2_state = SERIAL2_WAIT_HEADER1;

uint8_t SerialPose_TryGet(SerialPosePacket_t *packet)//主循环调用获取导航信息
{
  if (packet == NULL)
  {
    return 0U;
  }

  uint8_t has_new = 0U;
  __disable_irq();
  if (serial_pose_new != 0U)
  {
    memcpy(packet, (const void *)&serial_pose_latest, sizeof(SerialPosePacket_t));
    serial_pose_new = 0U;
    has_new = 1U;
  }
  __enable_irq();
  return has_new;
}

void serial2_reset_state(void)//重置标志位
{
  serial2_state = SERIAL2_WAIT_HEADER1;
  serial2_frame_index = 0U;
}

uint16_t serial2_crc16(const uint8_t *data, uint16_t length)//crc校验
{
  uint16_t crc = 0xFFFFU;
  if (data == NULL)
  {
    return 0U;
  }

  for (uint16_t i = 0U; i < length; ++i)
  {
    crc ^= (uint16_t)data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (crc >> 1U) ^ 0xA001U;
      }
      else
      {
        crc >>= 1U;
      }
    }
  }
  return crc;
}

void serial2_process_byte(uint8_t byte)//将接收的字节合成一帧之后进行处理
{
  switch (serial2_state)
  {
    case SERIAL2_WAIT_HEADER1:
      if (byte == SERIAL_PACKET_HEADER1)
      {
        serial2_frame_buf[0] = byte;
        serial2_frame_index = 1U;
        serial2_state = SERIAL2_WAIT_HEADER2;
      }
      break;

    case SERIAL2_WAIT_HEADER2:
      if (byte == SERIAL_PACKET_HEADER2)
      {
        serial2_frame_buf[1] = byte;
        serial2_frame_index = 2U;
        serial2_state = SERIAL2_READ_FRAME;
      }
      else
      {
        serial2_reset_state();
        if (byte == SERIAL_PACKET_HEADER1)
        {
          serial2_frame_buf[0] = byte;
          serial2_frame_index = 1U;
          serial2_state = SERIAL2_WAIT_HEADER2;
        }
      }
      break;

    case SERIAL2_READ_FRAME:
      if (serial2_frame_index < SERIAL_PACKET_SIZE)
      {
        serial2_frame_buf[serial2_frame_index++] = byte;
      }

      else
      {
        serial2_handle_frame(serial2_frame_buf);
        serial2_reset_state();
      }
      break;

    default:
      serial2_reset_state();
      break;
  }
}

void serial2_handle_frame(const uint8_t *frame)//校验帧头帧尾crc后处理
{
  if (frame == NULL)
  {
    return;
  }

  if ((frame[0] != SERIAL_PACKET_HEADER1) || (frame[1] != SERIAL_PACKET_HEADER2))
  {
    return;
  }

  if (frame[SERIAL_PACKET_SIZE - 1U] != SERIAL_PACKET_FOOTER)
  {
    return;
  }

  const uint16_t crc_rx = (uint16_t)(frame[SERIAL_PACKET_SIZE - 3U] |
                                     ((uint16_t)frame[SERIAL_PACKET_SIZE - 2U] << 8));
  const uint16_t crc_calc = serial2_crc16(&frame[2], (uint16_t)(SERIAL_PACKET_SIZE - 5U));
  if (crc_calc != crc_rx)
  {
    return;
  }

  SerialPosePacket_t packet;
  memcpy(&packet, frame, sizeof(packet));
  serial2_apply_packet(&packet);
}

void serial2_apply_packet(const SerialPosePacket_t *packet)//将完整的一帧放入全局结构体
{
  if (packet == NULL)
  {
    return;
  }

  serial_pose_latest = *packet;
  serial_pose_new = 1U;
  wheel_target_velocity = packet->linear_x;
  wheel_target_angular_velocity = packet->angular_z;
	
}
