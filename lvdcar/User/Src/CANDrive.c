#include "CANDrive.h"
#include "main.h"
#include "fdcan.h"
#include <string.h>

/* ============ 全局变量定义 ============ */

volatile ServoData_t g_servo_data[MAX_MOTORS];
volatile uint8_t g_servo_data_ready[MAX_MOTORS];

/* 向后兼容：默认指向电机1 */
volatile ServoData_t g_servo_latest;
volatile uint8_t g_servo_data_ready_compat;

/* ============ 内部函数声明 ============ */

static void CAN_Parse_TPDO1(uint8_t motor_id, uint8_t *RxData);
static void CAN_Parse_TPDO2(uint8_t motor_id, uint8_t *RxData);
static void CAN_Parse_TPDO3(uint8_t motor_id, uint8_t *RxData);


/* ============ FDCAN 初始化函数 ============ */

/**
  * @brief  FDCAN 用户初始化
  * @note   配置过滤器、启动FDCAN1、打开FIFO0新消息中断
  */
void FDCAN1_UserInit(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = CAN_FILTER_START;
    sFilterConfig.FilterID2 = CAN_FILTER_END;

    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

    HAL_FDCAN_Start(&hfdcan1);

    HAL_FDCAN_ActivateNotification(&hfdcan1,
                                    FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                    0);
}

/* ============ CAN 发送函数 ============ */

/**
  * @brief  发送标准 CAN 数据帧，固定8字节
  * @param  hfdcan     FDCAN句柄，例如 &hfdcan2
  * @param  Identifier 标准帧ID
  * @param  msg        8字节数据数组
  * @retval HAL状态
  */
HAL_StatusTypeDef CAN_Send_StdDataFrame(FDCAN_HandleTypeDef *hfdcan,
                                        uint32_t Identifier,
                                        uint8_t *msg)
{
    if ((hfdcan == NULL) || (msg == NULL))
    {
        return HAL_ERROR;
    }

    FDCAN_TxHeaderTypeDef CAN_Tx = {
        .Identifier = Identifier,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .FDFormat = FDCAN_CLASSIC_CAN,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0
    };

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &CAN_Tx, msg);
}

/* ============ CAN 接收回调函数 ============ */

/**
  * @brief  FDCAN FIFO0 接收回调
  * @note   这里只负责接收、解析、记录数据，不做串口打印
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET)
    {
        return;
    }

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        return;
    }

    switch (RxHeader.Identifier)
    {
        /* 电机1 */
        case MOTOR1_TPDO1_ID:
            CAN_Parse_TPDO1(0, RxData);
            break;

        case MOTOR1_TPDO2_ID:
            CAN_Parse_TPDO2(0, RxData);
            break;

        case MOTOR1_TPDO3_ID:
            CAN_Parse_TPDO3(0, RxData);
            break;

        /* 电机2 */
        case MOTOR2_TPDO1_ID:
            CAN_Parse_TPDO1(1, RxData);
            break;

        case MOTOR2_TPDO2_ID:
            CAN_Parse_TPDO2(1, RxData);
            break;

        case MOTOR2_TPDO3_ID:
            CAN_Parse_TPDO3(1, RxData);
            break;

        default:
            break;
    }

    /* 兼容旧接口：默认指向电机1 */
    g_servo_latest = g_servo_data[0];
    g_servo_data_ready_compat = g_servo_data_ready[0];
}

/**
  * @brief  启动 CANopen 节点
  * @param  hfdcan  FDCAN句柄
  * @param  node_id 节点ID
  * @retval HAL状态
  *
  * @note   发送：
  *         ID      = 0x000
  *         DATA[0] = 0x01
  *         DATA[1] = 0x01
  *         DATA[2]~DATA[7] = 0
  */
HAL_StatusTypeDef CAN_Send_StartNode(FDCAN_HandleTypeDef *hfdcan)
{
    uint8_t data[8] = {
        0x01,
        0x01,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00
    };

    return CAN_Send_StdDataFrame(hfdcan, 0x000, data);
}

/**
  * @brief  兼容旧代码的 CAN 发送函数
  * @note   固定使用 hfdcan1
  */
void can_tx(uint32_t id,
            uint8_t Data0,
            uint8_t Data1,
            uint8_t Data2,
            uint8_t Data3,
            uint8_t Data4,
            uint8_t Data5,
            uint8_t Data6,
            uint8_t Data7)
{
    uint8_t TxData[8];

    TxData[0] = Data0;
    TxData[1] = Data1;
    TxData[2] = Data2;
    TxData[3] = Data3;
    TxData[4] = Data4;
    TxData[5] = Data5;
    TxData[6] = Data6;
    TxData[7] = Data7;

    (void)CAN_Send_StdDataFrame(&hfdcan1, id, TxData);
}

/**
  * @brief  发送 32位 DEC 值，小端格式
  * @param  dec_value 需要发送的32位数据
  * @param  can_id    CAN ID
  *
  * @note   数据格式：
  *         DATA[0] = 0x03
  *         DATA[1] = 0x0F
  *         DATA[2] = 0x00
  *         DATA[3] = 低字节
  *         DATA[4] = 次低字节
  *         DATA[5] = 次高字节
  *         DATA[6] = 高字节
  *         DATA[7] = 0x00
  */
void Can_Send_Dec_Little_Endian(int32_t dec_value, uint16_t can_id)
{
    uint32_t u = (uint32_t)dec_value;

    uint8_t b0 = (uint8_t)(u & 0xFF);
    uint8_t b1 = (uint8_t)((u >> 8) & 0xFF);
    uint8_t b2 = (uint8_t)((u >> 16) & 0xFF);
    uint8_t b3 = (uint8_t)((u >> 24) & 0xFF);

    can_tx(can_id, 0x03, 0x0F, 0x00, b0, b1, b2, b3, 0x00);
}

/**
  * @brief  保留旧函数名，内部实际按小端发送
  */
void Can_Send_Dec_Big_Endian(int32_t dec_value, uint16_t can_id)
{
    Can_Send_Dec_Little_Endian(dec_value, can_id);
}


/* ============ 数据转换函数 ============ */

/**
  * @brief  将电机反馈的 DEC 速度值转换为 RPM
  * @param  vel 电机反馈的速度原始值
  * @retval rpm
  */
float dec_to_rpm(int32_t vel)
{
    float factor_reverse = 1875.0f / (512.0f * ENCODER_RES);
    return vel * factor_reverse;
}

/* ============ 错误字解析函数 ============ */

/**
  * @brief  解析电机错误字
  * @param  err   16位错误字
  * @param  servo 电机数据结构体指针
  */
void parse_error_word(uint16_t err, ServoData_t *servo)
{
    if (servo == NULL)
    {
        return;
    }

    servo->error_word = err;

    for (int i = 0; i < 16; i++)
    {
        servo->error_bits[i] = (err >> i) & 0x01;
    }

    servo->internal_error            = servo->error_bits[0];
    servo->encoder_abz_alarm         = servo->error_bits[1];
    servo->encoder_uvw_alarm         = servo->error_bits[2];
    servo->encoder_count_alarm       = servo->error_bits[3];
    servo->driver_over_temp          = servo->error_bits[4];
    servo->driver_over_voltage       = servo->error_bits[5];
    servo->driver_under_voltage      = servo->error_bits[6];
    servo->driver_over_current       = servo->error_bits[7];
    servo->absorption_resistor_alarm = servo->error_bits[8];
    servo->position_error            = servo->error_bits[9];
    servo->logic_low_voltage         = servo->error_bits[10];
    servo->motor_driver_iit_alarm    = servo->error_bits[11];
    servo->pulse_freq_high           = servo->error_bits[12];
    servo->motor_over_temp           = servo->error_bits[13];
    servo->motor_excitation          = servo->error_bits[14];
    servo->memory_alarm              = servo->error_bits[15];
}

/* ============ 状态字解析函数 ============ */

/**
  * @brief  解析电机状态字
  * @param  sw    16位状态字
  * @param  servo 电机数据结构体指针
  */
void parse_status_word(uint16_t sw, ServoData_t *servo)
{
    if (servo == NULL)
    {
        return;
    }

    servo->status_word = sw;

    for (int i = 0; i < 16; i++)
    {
        servo->status_bits[i] = (sw >> i) & 0x01;
    }

    servo->ready_to_switch_on = servo->status_bits[0];
    servo->switched_on        = servo->status_bits[1];
    servo->enabled            = servo->status_bits[2];
    servo->fault              = servo->status_bits[3];
    servo->voltage_disabled   = servo->status_bits[4];
    servo->quick_stop         = servo->status_bits[5];
    servo->switch_on_disabled = servo->status_bits[6];
    servo->warning            = servo->status_bits[7];

    /* bit8 保留 */

    servo->remote_control     = servo->status_bits[9];
    servo->target_reached     = servo->status_bits[10];
    servo->internal_limit     = servo->status_bits[11];
    servo->pulse_response     = servo->status_bits[12];
    servo->following_error    = servo->status_bits[13];
    servo->motor_excited      = servo->status_bits[14];
    servo->homing_found       = servo->status_bits[15];
}

/* ============ CAN 接收数据内部解析函数 ============ */

/**
  * @brief  解析 TPDO1：速度 + 电流
  * @param  motor_id 电机编号，0=电机1，1=电机2
  * @param  RxData   CAN接收到的8字节数据
  */
static void CAN_Parse_TPDO1(uint8_t motor_id, uint8_t *RxData)
{
    if ((motor_id >= MAX_MOTORS) || (RxData == NULL))
    {
        return;
    }

    int32_t vel_raw = (int32_t)((uint32_t)RxData[0] |
                               ((uint32_t)RxData[1] << 8) |
                               ((uint32_t)RxData[2] << 16) |
                               ((uint32_t)RxData[3] << 24));

    int16_t current_raw = (int16_t)((uint16_t)RxData[4] |
                                   ((uint16_t)RxData[5] << 8));

    g_servo_data[motor_id].actual_velocity = vel_raw;
    g_servo_data[motor_id].actual_current  = current_raw;
    g_servo_data[motor_id].actual_rpm      = dec_to_rpm(vel_raw);
    g_servo_data[motor_id].current_amps    = (float)current_raw * 0.1f;
    g_servo_data[motor_id].timestamp       = HAL_GetTick();

    g_servo_data_ready[motor_id] = 1;
}

/**
  * @brief  解析 TPDO2：错误字
  * @param  motor_id 电机编号，0=电机1，1=电机2
  * @param  RxData   CAN接收到的8字节数据
  */
static void CAN_Parse_TPDO2(uint8_t motor_id, uint8_t *RxData)
{
    if ((motor_id >= MAX_MOTORS) || (RxData == NULL))
    {
        return;
    }

    uint16_t err = (uint16_t)((uint16_t)RxData[0] |
                             ((uint16_t)RxData[1] << 8));

    parse_error_word(err, (ServoData_t *)&g_servo_data[motor_id]);

    g_servo_data[motor_id].timestamp = HAL_GetTick();
    g_servo_data_ready[motor_id] = 1;
}

/**
  * @brief  解析 TPDO3：状态字
  * @param  motor_id 电机编号，0=电机1，1=电机2
  * @param  RxData   CAN接收到的8字节数据
  */
static void CAN_Parse_TPDO3(uint8_t motor_id, uint8_t *RxData)
{
    if ((motor_id >= MAX_MOTORS) || (RxData == NULL))
    {
        return;
    }

    uint16_t sw = (uint16_t)((uint16_t)RxData[0] |
                            ((uint16_t)RxData[1] << 8));

    parse_status_word(sw, (ServoData_t *)&g_servo_data[motor_id]);

    g_servo_data[motor_id].timestamp = HAL_GetTick();
    g_servo_data_ready[motor_id] = 1;
}


/* ============ 伺服监控初始化 ============ */

/**
  * @brief  初始化伺服反馈数据
  */
void servo_monitor_init(void)
{
    for (int i = 0; i < MAX_MOTORS; i++)
    {
        memset((void *)&g_servo_data[i], 0, sizeof(ServoData_t));
        g_servo_data_ready[i] = 0;
    }

    memset((void *)&g_servo_latest, 0, sizeof(ServoData_t));
    g_servo_data_ready_compat = 0;
}

/**
  * @brief  自动监控任务
  * @note   当前版本不打印，只维护兼容接口
  */
void can_auto_monitor_task(void)
{
    g_servo_latest = g_servo_data[0];
    g_servo_data_ready_compat = g_servo_data_ready[0];
}

/* ============ 电机状态读取接口 ============ */

/**
  * @brief  判断电机是否准备好运行
  * @param  motor_id 0=电机1，1=电机2
  */
uint8_t is_motor_ready(uint8_t motor_id)
{
    if (motor_id >= MAX_MOTORS)
    {
        return 0;
    }

    return (g_servo_data[motor_id].enabled &&
            !g_servo_data[motor_id].fault);
}

/**
  * @brief  判断电机是否故障
  */
uint8_t is_motor_fault(uint8_t motor_id)
{
    if (motor_id >= MAX_MOTORS)
    {
        return 0;
    }

    return g_servo_data[motor_id].fault;
}

/**
  * @brief  判断电机是否有错误字报警
  */
uint8_t has_motor_error(uint8_t motor_id)
{
    if (motor_id >= MAX_MOTORS)
    {
        return 0;
    }

    return (g_servo_data[motor_id].error_word != 0);
}

/**
  * @brief  获取电机实际转速 RPM
  */
float get_motor_actual_rpm(uint8_t motor_id)
{
    if (motor_id >= MAX_MOTORS)
    {
        return 0.0f;
    }

    return g_servo_data[motor_id].actual_rpm;
}

/**
  * @brief  获取电机实际电流 A
  */
float get_motor_current_amps(uint8_t motor_id)
{
    if (motor_id >= MAX_MOTORS)
    {
        return 0.0f;
    }

    return g_servo_data[motor_id].current_amps;
}

/**
  * @brief  获取电机实际速度原始值
  */
int32_t get_motor_actual_velocity(uint8_t motor_id)
{
    if (motor_id >= MAX_MOTORS)
    {
        return 0;
    }

    return g_servo_data[motor_id].actual_velocity;
}

/**
  * @brief  获取电机实际电流原始值
  */
int16_t get_motor_actual_current_raw(uint8_t motor_id)
{
    if (motor_id >= MAX_MOTORS)
    {
        return 0;
    }

    return g_servo_data[motor_id].actual_current;
}

/**
  * @brief  获取完整电机数据结构体
  * @note   返回的是副本，不会直接修改全局变量
  */
ServoData_t get_motor_servo_data(uint8_t motor_id)
{
    ServoData_t empty_data;

    memset(&empty_data, 0, sizeof(ServoData_t));

    if (motor_id >= MAX_MOTORS)
    {
        return empty_data;
    }

    return g_servo_data[motor_id];
}

/**
  * @brief  判断电机反馈是否超时
  * @param  motor_id    0=电机1，1=电机2
  * @param  timeout_ms  超时时间，单位ms
  * @retval 1=超时，0=未超时
  */
uint8_t is_motor_feedback_timeout(uint8_t motor_id, uint32_t timeout_ms)
{
    if (motor_id >= MAX_MOTORS)
    {
        return 1;
    }

    if (g_servo_data[motor_id].timestamp == 0)
    {
        return 1;
    }

    if ((HAL_GetTick() - g_servo_data[motor_id].timestamp) > timeout_ms)
    {
        return 1;
    }

    return 0;
}

/**
  * @brief  保证发送启动节点到电机
  * @param  void
  */
void Motor_Start_Task(void)
{
    static uint32_t last_send_time = 0;
    static uint8_t motor_started = 0;

    if (motor_started)
        return;

    // 已收到电机反馈，说明电机在线
    if (!is_motor_feedback_timeout(0, 500))
    {
        motor_started = 1;
        return;
    }

    // 每 200ms 重发一次启动节点
    if (HAL_GetTick() - last_send_time >= 200)
    {
        last_send_time = HAL_GetTick();
        CAN_Send_StartNode(&hfdcan1);
    }
}
