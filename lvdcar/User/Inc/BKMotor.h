#ifndef __BKMOTOR_H
#define __BKMOTOR_H

#include <stdint.h>

/* ============ 轮子控制模块配置参数 ============ */
#define WHEEL_MAX_RPM     3000.0f      // 最大速度 ±3000rpm
#define WHEEL_MAX_RAD			0.157f			//最大旋转角速度 9°每秒
#define DEAD_ZONE         50.0f       // 遥杆死区
#define SMOOTH_ALPHA      0.1f        // 平滑滤波系数（越小越慢）
//#define ENCODER_RES       10000      //伺服驱动器精度
#define LEFT_MOTOR_DIR   (-1)   // 左侧电机：+1 表示软件正 = 电机正转 = 车向前
#define RIGHT_MOTOR_DIR  (-1)   // 右侧电机：-1 表示软件正要让电机反转以实现车向前
#define LEFT_MOTOR_DI   (+1)   // 左侧电机：+1 表示软件正 = 电机正转 = 车向前
#define RIGHT_MOTOR_DI  (+1)   // 右侧电机：-1 表示软件正要让电机反转以实现车向前
#define STEP_RPM_PER_SEC  800.0f   // 每秒最大加速/减速能力（你可根据电机改）
#define DIFF_GAIN    0.7f   // 差速比例系数，可调：0.0 ~ 2.0
#define ONE_AROUND_DISTANCE 0.028 //电机转一圈车前进0.02777米

/* ============ 优化方案2：预计算常量 ============ */
#define SMOOTH_ALPHA_INV    (1.0f - SMOOTH_ALPHA)    // 0.9f 预计算
#define DT_FIXED           0.02f   // 固定20ms（对应新的执行频率）
#define MAX_DELTA_FIXED    (STEP_RPM_PER_SEC * DT_FIXED)  // 预计算：30 * 0.02 = 0.6f

int32_t rpm_to_dec(float rpm);

float map_sbus_to_rpm(int16_t ch_val, int16_t ch_min, int16_t ch_center, int16_t ch_max, float max_rpm, float deadzone);

void Main_Loop(void);


#endif

