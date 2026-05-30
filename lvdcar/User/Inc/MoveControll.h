#ifndef __MOVECONTROLL_H
#define __MOVECONTROLL_H

#include "main.h"

#define PI 3.14159265358f
#define Chassis_Parameter_L 1.0f
#define WHEEL_RADIUS	0.20f
#define RC_INIT_STABILIZATION_MS    200     // 初始化稳定期时间(ms)
#define RC_DATA_CHANGE_THRESHOLD    10      // 数据变化检测阈值

//CH8: 系统开关二段 
#define CH8_SYSTEM_LOAD_START   200     // 档位1: RTK接收开始
#define CH8_SYSTEM_LOAD_STOP    1800    // 档位2: RTK停止接收
#define CH8_SYSTEM_TOLERANCE    300     // 档位容差（±300识别为有效档位）

//CH4: 运动模式三段开关 
#define CH4_MOVE_CONTROLLER     200     // 档位1: 差速（遥控器）
#define CH4_MOVE_LIDAR    			1000    // 档位2: 差速 (雷达)
#define CH4_MOVE_GPS      			1800    // 档位3: 差速（GPS）
#define CH4_MOVE_TOLERANCE      300     // 档位容差（±300识别为有效档位）

//CH9: 系统开关二段 
#define CH9_SYSTEM_LOCKED       200     // 档位1: 锁定模式
#define CH9_SYSTEM_MOVING       1800    // 档位2: 运动模式
#define CH9_SYSTEM_TOLERANCE    300     // 档位容差（±300识别为有效档位）

#define CH0_STEERING_CENTER     1000    // 中位值（左右居中）
#define CH1_THROTTLE_CENTER     983     // 中位值 (1766+200)/2 ≈ 983（停止位置）
#define CH2_MOVE_CENTER         1000    // 中位值（估算，需实测确认）
#define	CH3_DIFF_CENTER					1000		// 中位值

// 系统工作模式枚举（修改：移除断电模式）
typedef enum 
{
    SYS_LOCKED = 0,        // 锁定模式：系统上电但运动锁定，急停模式
    SYS_MOVING             // 运动模式：系统正在执行运动指令
} SystemMode_t;


// 运动模式枚举（修改：添加旋转模式）
typedef enum
{
    MOVE_Diff_Controller = 0,   // 差速（遥控器）
    MOVE_Diff_Lidar = 1, 				// 差速 (雷达)
		MOVE_Diff_GPS = 2    			  // 差速（GPS）
} MoveMode_t;

// 运动控制模式枚举
typedef enum
{
    RTK_LOAD_START = 0,   // RTK数据接收开始
    RTK_LOAD_STOP = 1,    // RTK数据停止接收
} RtkMode_t;

// MovingWheel结构体
typedef struct MovingWheel_t_Struct
{
    // ========== 系统状态 ==========
    SystemMode_t SystemMode;       // 系统工作模式
    MoveMode_t MoveMode;           // 运动模式（仅在SYS_MOVING时有效）
		RtkMode_t RTKMode; 				 		 // RTK数据接收模式
    uint32_t RCIdleTimeMs;         // 遥控器空闲时间计数器(毫秒)
 
    // ========== 驱动轮目标速度 ==========
    float L_Vel,                   // 左轮目标速度(RPM) 
					R_Vel;                   // 右轮目标速度(RPM) 
    
    // ========== 驱动轮反馈速度 ==========
    float L_Vel_FB,                // 左轮实际速度(RPM) 
					R_Vel_FB;                // 右轮实际速度(RPM) 
          
} MovingWheel_t;



typedef struct RemoteControl_Struct
{
    // ========== 连接状态监控 ==========
    uint8_t IsRCConnected;          // 遥控器连接状态 (0=未连接, 1=已连接)
    uint8_t RCStatic;               // 遥控器静态状态 (0=有变化, 1=静止)
    uint32_t LastUpdateTime;        // 最后数据更新时间戳(ms) - 用于失联检测
    uint8_t InitializationComplete; // 初始化完成标志
    
    // ========== SBUS原始通道数据 ==========
    uint16_t CH0_Steering;          // CH0: 右摇杆X轴 - 左右转向控制
    uint16_t CH1_Throttle;          // CH1: 左摇杆Y轴 - 油门/前进后退
    uint16_t CH2_ForwardBackward;   // CH2: 右摇杆Y轴 - 前进后退方向
		uint16_t CH3_Diff; 						  // CH2: 右摇杆Y轴 - 前进后退方向
    uint16_t CH8_RTKDataMode;   		// CH8: RTK手动接收模式调整
    uint16_t CH4_MoveMode;          // CH4: 运动模式三段开关（遥控器/雷达/GPS）
    uint16_t CH9_SystemMode;        // CH9: 系统模式二段开关（锁定/运动）
    
    // ========== 归一化后的摇杆数据 ==========
    float LeftY;                    // 左摇杆Y轴归一化值 (-1.0~+1.0) - 油门控制
    float RightX;                   // 右摇杆X轴归一化值 (-1.0~+1.0) - 转向控制
    float RightY;                   // 右摇杆Y轴归一化值 (-1.0~+1.0) - 前进后退
    float LeftX;                    // 左摇杆X轴归一化值 (-1.0~+1.0) - 
    // ========== 开关状态解析结果 ==========
    uint8_t RtkDataMode;        		// RTK数据接收模式 (0=开始, 1=停止，0xFF=无效)
    uint8_t SystemMode;             // 系统工作模式 (0=锁定, 1=运动, 0xFF=无效)
    uint8_t MoveMode;               // 运动模式 (0=遥控器, 1=雷达, 2=GPS, 0xFF=无效)
    
    // ========== 模式切换事件检测 ==========
    uint8_t RTKDataModeChanged; 		// Rtk模式变化标志 (0=无变化, 1=有变化)
    uint8_t SystemModeChanged;      // 系统模式变化标志
    uint8_t MoveModeChanged;        // 运动模式变化标志
    
 } RemoteControl;

extern MovingWheel_t g_moving_wheel;

uint8_t Sbus_GetMotorMode(uint16_t ch8_value);
uint8_t Sbus_GetMoveMode(uint16_t ch4_value);
uint8_t Sbus_GetSystemMode(uint16_t ch9_value);
void RCInit(void);
int mps_to_rpm(float mps);
void RCGetValue(void);
void ControlLoop_Init(void);
void MotorDriver_SetLock(void);
void DiffMove_Controller(void);
void DiffMove_LiDAR(void);
static uint8_t RC_ValidateMode(uint8_t mode, uint8_t max_value);
float rpm_to_mps(int rpm);


#endif 





