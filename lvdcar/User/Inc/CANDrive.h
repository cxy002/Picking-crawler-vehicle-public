#ifndef __CANDRIVE_H
#define __CANDRIVE_H

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "fdcan.h"

/* ============ 基本配置 ============ */

#define MAX_MOTORS      2
#define ENCODER_RES     10000.0f

/* ============ 双电机 CAN 反馈 ID 配置 ============ */

/* 电机1反馈配置 */
#define MOTOR1_ACTUAL_VELOCITY_ID    0x181
#define MOTOR1_ACTUAL_CURRENT_ID     0x181
#define MOTOR1_ERROR_FEEDBACK_ID     0x182
#define MOTOR1_STATUS_FEEDBACK_ID    0x183

/* 电机2反馈配置 */
#define MOTOR2_ACTUAL_VELOCITY_ID    0x281
#define MOTOR2_ACTUAL_CURRENT_ID     0x281
#define MOTOR2_ERROR_FEEDBACK_ID     0x282
#define MOTOR2_STATUS_FEEDBACK_ID    0x283

/* TPDO 别名 */
#define MOTOR1_TPDO1_ID    MOTOR1_ACTUAL_VELOCITY_ID
#define MOTOR1_TPDO2_ID    MOTOR1_ERROR_FEEDBACK_ID
#define MOTOR1_TPDO3_ID    MOTOR1_STATUS_FEEDBACK_ID

#define MOTOR2_TPDO1_ID    MOTOR2_ACTUAL_VELOCITY_ID
#define MOTOR2_TPDO2_ID    MOTOR2_ERROR_FEEDBACK_ID
#define MOTOR2_TPDO3_ID    MOTOR2_STATUS_FEEDBACK_ID

/* CAN 接收过滤器范围 */
#define CAN_FILTER_START   0x181
#define CAN_FILTER_END     0x284

/* ============ 电机伺服数据结构体 ============ */

typedef struct
{
    /* 速度和电流数据 */
    int32_t actual_velocity;     // 实际速度原始 DEC 值
    int16_t actual_current;      // 实际电流原始值
    float actual_rpm;            // 实际转速 RPM
    float current_amps;          // 实际电流 A
    uint32_t timestamp;          // 最近一次更新时间

    /* 错误字 */
    uint16_t error_word;
    uint8_t error_bits[16];

    uint8_t internal_error;
    uint8_t encoder_abz_alarm;
    uint8_t encoder_uvw_alarm;
    uint8_t encoder_count_alarm;
    uint8_t driver_over_temp;
    uint8_t driver_over_voltage;
    uint8_t driver_under_voltage;
    uint8_t driver_over_current;
    uint8_t absorption_resistor_alarm;
    uint8_t position_error;
    uint8_t logic_low_voltage;
    uint8_t motor_driver_iit_alarm;
    uint8_t pulse_freq_high;
    uint8_t motor_over_temp;
    uint8_t motor_excitation;
    uint8_t memory_alarm;

    /* 状态字 */
    uint16_t status_word;
    uint8_t status_bits[16];

    uint8_t ready_to_switch_on;
    uint8_t switched_on;
    uint8_t enabled;
    uint8_t fault;
    uint8_t voltage_disabled;
    uint8_t quick_stop;
    uint8_t switch_on_disabled;
    uint8_t warning;
    uint8_t remote_control;
    uint8_t target_reached;
    uint8_t internal_limit;
    uint8_t pulse_response;
    uint8_t following_error;
    uint8_t motor_excited;
    uint8_t homing_found;

} ServoData_t;

/* ============ 全局变量声明 ============ */

extern volatile ServoData_t g_servo_data[MAX_MOTORS];
extern volatile uint8_t g_servo_data_ready[MAX_MOTORS];

/* 向后兼容：默认指向电机1 */
extern volatile ServoData_t g_servo_latest;
extern volatile uint8_t g_servo_data_ready_compat;

/* ============ 初始化函数 ============ */

void FDCAN2_UserInit(void);
void FDCAN1_UserInit(void);        // 兼容旧函数名，内部实际调用 FDCAN2_UserInit
void servo_monitor_init(void);

/* ============ CAN 发送函数 ============ */

HAL_StatusTypeDef CAN_Send_StdDataFrame(FDCAN_HandleTypeDef *hfdcan,
                                        uint32_t Identifier,
                                        uint8_t *msg);

HAL_StatusTypeDef CAN_Send_StartNode(FDCAN_HandleTypeDef *hfdcan);


void can_tx(uint32_t id,
            uint8_t Data0,
            uint8_t Data1,
            uint8_t Data2,
            uint8_t Data3,
            uint8_t Data4,
            uint8_t Data5,
            uint8_t Data6,
            uint8_t Data7);

void Can_Send_Dec_Little_Endian(int32_t dec_value, uint16_t can_id);

/* 保留旧函数名，防止 wheel.c 里面已经调用这个名字 */
void Can_Send_Dec_Big_Endian(int32_t dec_value, uint16_t can_id);

/* ============ 数据转换和解析函数 ============ */

float dec_to_rpm(int32_t vel);

void parse_error_word(uint16_t err, ServoData_t *servo);
void parse_status_word(uint16_t sw, ServoData_t *servo);

/* ============ 电机数据读取接口 ============ */

uint8_t is_motor_ready(uint8_t motor_id);
uint8_t is_motor_fault(uint8_t motor_id);
uint8_t has_motor_error(uint8_t motor_id);

float get_motor_actual_rpm(uint8_t motor_id);
float get_motor_current_amps(uint8_t motor_id);
int32_t get_motor_actual_velocity(uint8_t motor_id);
int16_t get_motor_actual_current_raw(uint8_t motor_id);

ServoData_t get_motor_servo_data(uint8_t motor_id);

uint8_t is_motor_feedback_timeout(uint8_t motor_id, uint32_t timeout_ms);

/* ============ 自动任务 ============ */

void can_auto_monitor_task(void);

/* ============ FDCAN 接收回调 ============ */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,

                               uint32_t RxFifo0ITs);
/* ============ 电机启动节点发送 ============ */
void Motor_Start_Task(void);

#endif
