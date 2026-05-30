#ifndef __NAVCMD_H
#define __NAVCMD_H


#include "main.h"


#define SERIAL_PACKET_SIZE 41U
#define SERIAL_PACKET_HEADER1 0x7AU
#define SERIAL_PACKET_HEADER2 0x7BU
#define SERIAL_PACKET_FOOTER 0x7DU


#pragma pack(push, 1)
typedef struct
{
  uint8_t header;
  uint8_t header2;
  float linear_x;
  float linear_y;
  float angular_z;
  double pos_x;
  double pos_y;
  double yaw;
  uint16_t checksum;
  uint8_t footer;
} SerialPosePacket_t;
#pragma pack(pop)


typedef enum
{
  SERIAL2_WAIT_HEADER1 = 0,
  SERIAL2_WAIT_HEADER2,
  SERIAL2_READ_FRAME
} Serial2RxState_t;


extern volatile uint8_t serial_pose_new;
extern uint8_t serial2_frame_index;
extern uint8_t serial2_frame_buf[SERIAL_PACKET_SIZE];
extern volatile SerialPosePacket_t serial_pose_latest;
extern uint8_t uart1_rx_byte[1];
extern float wheel_target_velocity;
extern float wheel_target_angular_velocity;
extern SerialPosePacket_t NAV_Diff_CMD;


extern uint8_t SerialPose_TryGet(SerialPosePacket_t *packet);
void serial2_reset_state(void);
uint16_t serial2_crc16(const uint8_t *data, uint16_t length);
void serial2_handle_frame(const uint8_t *frame);
void serial2_apply_packet(const SerialPosePacket_t *packet);
extern void serial2_process_byte(uint8_t byte);




#endif
