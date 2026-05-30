#include "MoveControll.h"
#include "sbus.h"
#include "BKMotor.h"
#include "stdlib.h"
#include "CANDrive.h"
#include "Navcmd.h"

RemoteControl RC;
MovingWheel_t g_moving_wheel;
static uint16_t LastCH0, LastCH1, LastCH2, LastCH3;
static uint8_t LastRtkDataMode, LastSystemMode, LastMoveMode;
static uint32_t InitStartTime = 0;
static uint8_t g_motion_reset_request = 0;

static void RC_EnterFailsafeMode(void)
{
    RC.IsRCConnected = 0;
    RC.RCStatic = 1;
    RC.SystemMode = SYS_LOCKED;
    RC.LeftY = 0.0f;
    RC.LeftX = 0.0f;
    RC.RightX = 0.0f;
    RC.RightY = 0.0f;

    g_moving_wheel.SystemMode = SYS_LOCKED;
    g_moving_wheel.RCIdleTimeMs = 0;
    MotorDriver_SetLock();
    Can_Send_Dec_Big_Endian(g_moving_wheel.L_Vel, 0x201);
    Can_Send_Dec_Big_Endian(g_moving_wheel.R_Vel, 0x202);
}


//CH8: 电机控制模式解析（保持不变）
uint8_t Sbus_GetMotorMode(uint16_t ch8_value)
{
    // 挡位1: 锁定模式（200 ± 300）
    if (abs((int)ch8_value - CH8_SYSTEM_LOAD_START) < CH8_SYSTEM_TOLERANCE) {
        return 0;  // 锁定 -> SYS_LOCKED
    }
    // 挡位2: 运动模式（1800 ± 300）
    else if (abs((int)ch8_value - CH8_SYSTEM_LOAD_STOP) < CH8_SYSTEM_TOLERANCE) {
        return 1;  // 运动 -> SYS_MOVING
    }
    
    return 0xFF;  // 无效值
}


//CH4: 运动模式三段开关（修改：原来的系统模式改为运动模式）
uint8_t Sbus_GetMoveMode(uint16_t ch4_value)
{
    // 挡位1: 差速（遥控器）（200 ± 300）
    if (abs((int)ch4_value - CH4_MOVE_CONTROLLER) < CH4_MOVE_TOLERANCE) {
        return 0;  // 
    }
    // 挡位2: 差速 (雷达)（1000 ± 300）
    else if (abs((int)ch4_value - CH4_MOVE_LIDAR) < CH4_MOVE_TOLERANCE) {
        return 1;  // 
    }
    // 挡位3: 差速（GPS）（1800 ± 300）（新增）
    else if (abs((int)ch4_value - CH4_MOVE_GPS) < CH4_MOVE_TOLERANCE) {
        return 2;  // 
    }
    
    return 0xFF;  // 无效值
}


//CH9: 系统开关二段（修改：原来的运动模式改为系统模式）
uint8_t Sbus_GetSystemMode(uint16_t ch9_value)
{
    // 挡位1: 锁定模式（200 ± 300）
    if (abs((int)ch9_value - CH9_SYSTEM_LOCKED) < CH9_SYSTEM_TOLERANCE) {
        return 0;  // 锁定 -> SYS_LOCKED
    }
    // 挡位2: 运动模式（1800 ± 300）
    else if (abs((int)ch9_value - CH9_SYSTEM_MOVING) < CH9_SYSTEM_TOLERANCE) {
        return 1;  // 运动 -> SYS_MOVING
    }
    
    return 0xFF;  // 无效值
}

// ========== 模式验证函数 ==========
static uint8_t RC_ValidateMode(uint8_t mode, uint8_t max_value)
{
    return (mode != 0xFF && mode <= max_value) ? mode : 0xFF;
}

// ========== 遥控器安全初始化 ==========
void RCInit(void)
{
    InitStartTime = HAL_GetTick();      // 记录初始化开始时间
    
    // ========== 基础状态初始化 ==========
    RC.IsRCConnected = 0;               // 假设未连接，等待验证
    RC.RCStatic = 1;                    // 初始静止状态
    RC.LastUpdateTime = InitStartTime;  // 设为初始化时间，避免立即超时
    RC.InitializationComplete = 0;      // 初始化未完成标志
    
    // ========== SBUS数据安全初始化 ==========
    RC.CH0_Steering = CH0_STEERING_CENTER;
    RC.CH1_Throttle = CH1_THROTTLE_CENTER;
    RC.CH2_ForwardBackward = CH2_MOVE_CENTER;
		RC.CH3_Diff = CH3_DIFF_CENTER;
    RC.CH8_RTKDataMode = CH8_SYSTEM_LOAD_STOP;    	// 默认模式一
    RC.CH4_MoveMode = CH4_MOVE_CONTROLLER;       // CH4默认遥控器模式
    RC.CH9_SystemMode = CH9_SYSTEM_LOCKED;      // CH9默认锁定模式
    
    // ========== 归一化数据清零 ==========
    RC.LeftY = 0.0f;
		RC.LeftX = 0.0f;
    RC.RightX = 0.0f;
    RC.RightY = 0.0f;
    
    // ========== 模式状态安全初始化 ==========
    RC.RtkDataMode = 0;             // 模式一
    RC.SystemMode = 0;                  // 锁定模式
    RC.MoveMode = 0;                    // 遥控器模式
    
    // ========== 系统控制结构体同步 ==========
    g_moving_wheel.RTKMode = RTK_LOAD_STOP;
    g_moving_wheel.SystemMode = SYS_LOCKED;
    g_moving_wheel.MoveMode = MOVE_Diff_Controller;
    g_moving_wheel.RCIdleTimeMs = 0;
    
    // ========== 事件标志初始化 ==========
    RC.RTKDataModeChanged = 0;
    RC.SystemModeChanged = 0;
    RC.MoveModeChanged = 0;
    
    // ========== 历史值初始化 ==========
    LastCH0 = CH0_STEERING_CENTER;
    LastCH1 = CH1_THROTTLE_CENTER;
    LastCH2 = CH2_MOVE_CENTER;
		LastCH3 = CH3_DIFF_CENTER;
    LastRtkDataMode = 0;
    LastSystemMode = 0;
    LastMoveMode = 0;
}

void RCGetValue(void)
{
    if ((sbus_data.link_lost != 0U) || (sbus_data.error != 0U))
    {
        RC_EnterFailsafeMode();
        return;
    }
    // 检查初始化稳定期
    if(!RC.InitializationComplete) {
        if(HAL_GetTick() - InitStartTime < RC_INIT_STABILIZATION_MS) {
            return; // 稳定期内不处理数据
        }
        RC.InitializationComplete = 1; // 标记初始化完成
    }
    
    // ========== 从SBUS通道数组读取原始数据 ==========
    uint16_t new_ch0 = sbus_data.analog[ 0];            // CH0: 转向
    uint16_t new_ch1 = sbus_data.analog[ 1];            // CH1: 油门
    uint16_t new_ch2 = sbus_data.analog[ 2];      			// CH2: 前进后退
		uint16_t new_ch3 = sbus_data.analog[ 3];     				// CH3: DIFF
    uint16_t new_ch8 = sbus_data.analog[ 8];         	  // CH8: RTK接收模式 
    uint16_t new_ch4 = sbus_data.analog[ 4];       	 	 	// CH4: 运动模式（三段开关）
    uint16_t new_ch9 = sbus_data.analog[ 9];      			// CH9: 系统模式（二段开关）

    // 数据有效性检查
    // SBUS标准范围200-1800
    if(new_ch0 < 150 || new_ch0 > 1850 ||
       new_ch1 < 150 || new_ch1 > 1850 ||
       new_ch2 < 150 || new_ch2 > 1850 ||
       new_ch3 < 150 || new_ch3 > 1850) {
        // 数据异常，触发失联保护
        RC_EnterFailsafeMode();
        return;
    }
    
    // 数据有效，更新到RC结构体
    RC.CH0_Steering = new_ch0;
    RC.CH1_Throttle = new_ch1;
    RC.CH2_ForwardBackward = new_ch2;
		RC.CH3_Diff = new_ch3;
    RC.CH8_RTKDataMode = new_ch8;
    RC.CH4_MoveMode = new_ch4;          // CH4的数据存到MoveMode变量
    RC.CH9_SystemMode = new_ch9;        // CH9的数据存到SystemMode变量
    
    // ========== 摇杆数据归一化处理 ==========
    RC.LeftY = map_sbus_to_rpm(RC.CH1_Throttle,200,1000,1800,1,50);       // 油门控制map_sbus(float in,float in_max,float in_min,float out_max,float out_min)
    RC.RightX = map_sbus_to_rpm(RC.CH0_Steering,200,1000,1800,1,50);      // 转向控制
    RC.RightY = map_sbus_to_rpm(RC.CH2_ForwardBackward,200,1000,1800,1,50);   // 前进后退
    RC.LeftX = map_sbus_to_rpm(RC.CH3_Diff,200,1000,1800,1,50);       //
    // ========== 开关状态解析（关键修正：CH4解析运动模式，CH9解析系统模式） ========== 
    uint8_t new_rtk_mode = Sbus_GetMotorMode(RC.CH8_RTKDataMode);  		  // CH8解析rtk模式
    uint8_t new_move_mode = Sbus_GetMoveMode(RC.CH4_MoveMode);          // CH4解析运动模式
    uint8_t new_system_mode = Sbus_GetSystemMode(RC.CH9_SystemMode);    // CH9解析系统模式
    
    // ========== 模式验证 ==========
    new_rtk_mode = RC_ValidateMode(new_rtk_mode, 1);    		// 电机模式：0-1
    new_system_mode = RC_ValidateMode(new_system_mode, 1);  // 系统模式：0-1
    new_move_mode = RC_ValidateMode(new_move_mode, 2);      // 运动模式：0-2
    // ========== 检测模式变化事件 ==========
    RC.RTKDataModeChanged = (new_rtk_mode != LastRtkDataMode && new_rtk_mode != 0xFF) ? 1 : 0;
    RC.SystemModeChanged = (new_system_mode != LastSystemMode && new_system_mode != 0xFF) ? 1 : 0;
    RC.MoveModeChanged = (new_move_mode != LastMoveMode && new_move_mode != 0xFF) ? 1 : 0;
    
    // ========== 模式状态更新 ==========
    if(new_rtk_mode != 0xFF) RC.RtkDataMode = new_rtk_mode;
    if(new_system_mode != 0xFF) RC.SystemMode = new_system_mode;
    if(new_move_mode != 0xFF) RC.MoveMode = new_move_mode;

    // ========== 系统模式同步 ==========
    if(RC.SystemModeChanged && RC.SystemMode == 1) { // 要求进入运动模式
        // 确保摇杆在中性位置，防止意外运动
        if(fabs(RC.LeftY) < 0.1f && fabs(RC.LeftX) < 0.1f && fabs(RC.RightX) < 0.1f ) {
            g_moving_wheel.SystemMode = SYS_MOVING; // 允许运动
        } else {
            RC.SystemMode = 0; // 摇杆不在中性位置，保持锁定
            g_moving_wheel.SystemMode = SYS_LOCKED;
        }
    } else {
        // 其他模式变化正常同步
        if(RC.SystemModeChanged) {
            g_moving_wheel.SystemMode = (SystemMode_t)RC.SystemMode;
        }
        
    }
		
		if(RC.RTKDataModeChanged) {
            g_moving_wheel.RTKMode = (RtkMode_t)RC.RtkDataMode;
        }
    
		if(RC.MoveModeChanged) {
            g_motion_reset_request = 1;
            g_moving_wheel.MoveMode = (MoveMode_t)RC.MoveMode;
        }		
				
    // ========== 数据变化检测 ==========
    if((abs((int)LastCH0 - (int)RC.CH0_Steering) > RC_DATA_CHANGE_THRESHOLD) ||
       (abs((int)LastCH1 - (int)RC.CH1_Throttle) > RC_DATA_CHANGE_THRESHOLD) ||
       (abs((int)LastCH2 - (int)RC.CH2_ForwardBackward) > RC_DATA_CHANGE_THRESHOLD) ||
			 (abs((int)LastCH3 - (int)RC.CH3_Diff) > RC_DATA_CHANGE_THRESHOLD) ||
       RC.RTKDataModeChanged || RC.SystemModeChanged || RC.MoveModeChanged)
    {
        RC.RCStatic = 0;                    // 有变化
        g_moving_wheel.RCIdleTimeMs = 0;    // 重置空闲时间
    }
    else
    {
        RC.RCStatic = 1;                    // 静止状态
        if(g_moving_wheel.RCIdleTimeMs < 65535) {
            g_moving_wheel.RCIdleTimeMs++;
        }
    }
    
    // ========== 连接状态更新 ==========
    RC.IsRCConnected = 1;
    RC.LastUpdateTime = HAL_GetTick();
    
    // ========== 保存历史值 ==========
    LastCH0 = RC.CH0_Steering;
    LastCH1 = RC.CH1_Throttle;
    LastCH2 = RC.CH2_ForwardBackward;
		LastCH3 = RC.CH3_Diff;
    LastRtkDataMode = RC.RtkDataMode;
    LastSystemMode = RC.SystemMode;
    LastMoveMode = RC.MoveMode;
		
	
}

void ControlLoop_Init(void)
{
/************ 计算每次循环的时间（dt） *************/
		static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();

    if (last_tick == 0) {
        last_tick = now;
        return;   // 第一次进来不做控制
    }

    float dt = (now - last_tick) * 0.001f;
    last_tick = now;

    // 防止异常 dt
    if (dt <= 0.0f || dt > 0.1f) {
        return;
    }
/************* 根据系统模式执行相应的控制逻辑 *************/
    switch (g_moving_wheel.SystemMode)
    {
        case SYS_LOCKED:
            // 锁定模式：转向轮保持X型，停止驱动
            MotorDriver_SetLock();
            break;
            
        case SYS_MOVING:
					
						 switch (g_moving_wheel.MoveMode)
							 {
										case MOVE_Diff_Controller://差速（遥控器）
											 DiffMove_Controller ();								
												break;
												
										case MOVE_Diff_Lidar://差速 (雷达)
												DiffMove_LiDAR();
												break;
												
										case MOVE_Diff_GPS://差速（GPS）
											DiffMove_Controller ();
											//	DiffMove_GPS ();
												break;
						
								}		
				     
/****************** 更新MW和DDSM电机控制 ********************/ 
            Can_Send_Dec_Big_Endian(g_moving_wheel.L_Vel, 0x201);
						Can_Send_Dec_Big_Endian(g_moving_wheel.R_Vel, 0x202);
            break;
    }
		
		switch (g_moving_wheel.RTKMode)
		{
			case RTK_LOAD_START:
				
			break;
			
			case RTK_LOAD_STOP:
				
			break;
			
			
		}
}

int mps_to_rpm(float mps)
{
    float wheel_perimeter = ONE_AROUND_DISTANCE;   // 米
    return mps * 60 / wheel_perimeter;
}

float rpm_to_mps(int rpm)
{
    float wheel_perimeter = ONE_AROUND_DISTANCE;   // 米
		return rpm / 60 * wheel_perimeter;
	
}

void MotorDriver_SetLock()
{
		
		g_moving_wheel.L_Vel  = rpm_to_dec(0);
    g_moving_wheel.R_Vel = rpm_to_dec(0);
    g_motion_reset_request = 1;

}
	

/* ============ 轮子控制任务 ============ */
void DiffMove_Controller(void)
{
    // 保留为 static 以便跨次调用记忆上次 ramp 值
    static float ramp_left  = 0.0f;
    static float ramp_right = 0.0f;
//	  static float smooth_left = 0, smooth_right = 0;
//	  float rpm_left_sbus = 0;   // 保存 SBUS 原始左轮 rpm
//    float rpm_right_sbus = 0;  // 保存 SBUS 原始右轮 rpm

    float rpm_left = 0.0f, rpm_right = 0.0f;
    float speed_x;   // 前进 / 后退速度
    float speed_w;   // 转向速度
		static float smaath_left = 0,smaath_right = 0;

    if (g_motion_reset_request != 0U)
    {
        ramp_left = 0.0f;
        ramp_right = 0.0f;
        smaath_left = 0.0f;
        smaath_right = 0.0f;
        g_motion_reset_request = 0U;
    }
	   /* 检查SBUS数据有效性（基于你的sbus_data_ready标志） */
//    if(!Sbus_Check_Connection())
//    {
//        // SBUS数据无效，进入安全模式
//        rpm_left = 0.0f;
//        rpm_right = 0.0f;
//        ramp_left = 0.0f;
//        ramp_right = 0.0f;
//        smaath_left = 0.0f;
//        smaath_right = 0.0f;
//			

//        // 发送停止指令
//        int32_t dec_left  = rpm_to_dec(0);
//        int32_t dec_right = rpm_to_dec(0);
//        Can_Send_Dec_Big_Endian(dec_left, 0x201);
//        Can_Send_Dec_Big_Endian(dec_right, 0x202);
//        return;
//    }


    /* 映射为 rpm（包含死区） */
//		RC.LeftY = 0.4;
//		RC.LeftX = 0.3;
    speed_x = RC.LeftY  * rpm_to_mps(WHEEL_MAX_RPM);
    speed_w = RC.LeftX  * WHEEL_MAX_RAD;

//    /* 小抖动死区（额外过滤） */
//    if (fabsf(speed_x) < 3.0f) speed_x = 0.0f;
//    if (fabsf(speed_w) < 1.0f) speed_w = 0.0f;
		
    rpm_left  = (speed_x - speed_w * Chassis_Parameter_L)  * (float)LEFT_MOTOR_DI;
    rpm_right = (speed_x + speed_w * Chassis_Parameter_L) * (float)RIGHT_MOTOR_DI;
		
		rpm_left = mps_to_rpm(rpm_left);
		rpm_right = mps_to_rpm(rpm_right);
    /* 一秒最大变化量 */
    float max_delta = MAX_DELTA_FIXED;  // 预计算的 30 * 0.02 = 0.6

    /* 梯度加速（ramp） - 对目标 rpm_left/right 做速率限制，更新持久 ramp 值 */
    float deltaL = rpm_left - ramp_left;
    if (fabsf(deltaL) <= max_delta) ramp_left = rpm_left;
    else ramp_left += (deltaL > 0.0f ? max_delta : -max_delta);

    float deltaR = rpm_right - ramp_right;
    if (fabsf(deltaR) <= max_delta) ramp_right = rpm_right;
    else ramp_right += (deltaR > 0.0f ? max_delta : -max_delta);

    /* 可选：对 ramp 做轻微平滑（用 ramp 而不是 rpm，避免覆盖差速）
       若你不想额外平滑，可以直接发送 ramp_* */
    smaath_left  = smaath_left * SMOOTH_ALPHA_INV + ramp_left * SMOOTH_ALPHA;
    smaath_right = smaath_right * SMOOTH_ALPHA_INV + ramp_right * SMOOTH_ALPHA;

    /* 限幅保护（基于 smooth 输出） */
    if (smaath_left >  WHEEL_MAX_RPM)  smaath_left =  WHEEL_MAX_RPM;
    if (smaath_left < -WHEEL_MAX_RPM)  smaath_left = -WHEEL_MAX_RPM;
    if (smaath_right >  WHEEL_MAX_RPM) smaath_right =  WHEEL_MAX_RPM;
    if (smaath_right < -WHEEL_MAX_RPM) smaath_right = -WHEEL_MAX_RPM;

    /* 转成整型并发送（注意：发送的是 smooth/ramp，不是原来的 rpm） */
    int16_t send_left  = (int16_t)roundf(smaath_left);
    int16_t send_right = (int16_t)roundf(smaath_right);

   /* ============ ★ 转成 DEC 并发送 CAN（关键补充）★ ============ */
    int32_t dec_left  = rpm_to_dec(send_left);
    int32_t dec_right = rpm_to_dec(send_right);
		
		g_moving_wheel.L_Vel = dec_left;
		g_moving_wheel.R_Vel = dec_right;
    

}

void DiffMove_LiDAR()
{

// 差速模式初始化
    static uint8_t diff_init_done = 0;
    static uint32_t init_start_time = 0;

	  static float ramp_left  = 0.0f;
    static float ramp_right = 0.0f;
	  static float smooth_left = 0;
		static float smooth_right = 0;	
		float rpm_left = 0.0f;
		float rpm_right = 0.0f;
		static float v_x = 0.0f;
    static float w_z = 0.0f;
		static uint32_t last_nav_time = 0;

    if (g_motion_reset_request != 0U)
    {
        diff_init_done = 0;
        init_start_time = 0;
        ramp_left = 0.0f;
        ramp_right = 0.0f;
        smooth_left = 0.0f;
        smooth_right = 0.0f;
        v_x = 0.0f;
        w_z = 0.0f;
        last_nav_time = 0;
        g_motion_reset_request = 0U;
    }
	
    if (!diff_init_done) {
        if (init_start_time == 0) {
            init_start_time = HAL_GetTick();
           
            // 停止所有驱动轮（安全）
            g_moving_wheel.L_Vel = 0.0f;
            g_moving_wheel.R_Vel = 0.0f;

        }
        
        // 等待电机停止（1.5秒）并确保摇杆在中性位置
        if (HAL_GetTick() - init_start_time >= 1500) {
            if (fabs(RC.LeftY) < 0.1f && fabs(RC.LeftX) < 0.1f ) {
                diff_init_done = 1;  // 初始化完成
                
            }
        }

        return;
    }
 
    //========== 差速控制逻辑 ==========

		if (SerialPose_TryGet(&NAV_Diff_CMD))
		{
				// 成功获取最新数据
				v_x = NAV_Diff_CMD.linear_x;
				w_z = NAV_Diff_CMD.angular_z;
				last_nav_time = HAL_GetTick();
		}
		else
		{
				if(HAL_GetTick() - last_nav_time > 1500)
				{
						v_x = 0.0f;
						w_z = 0.0f;
				}
		}
    
    // ----------------- LiDAR 输出 v, w -----------------
    float v = v_x;	//米每秒
    float w = w_z;	//弧度每秒

    // ----------------- v,w -> 左右轮理论速度 -----------------
    float v_left_target  = v - w * Chassis_Parameter_L;			//米每秒
    float v_right_target = v + w * Chassis_Parameter_L;			//米每秒

    float left_cmd  = v_left_target * (float)LEFT_MOTOR_DI;		//米每秒
    float right_cmd = v_right_target * (float)RIGHT_MOTOR_DI;		//米每秒
		
		rpm_left = mps_to_rpm (left_cmd);
		rpm_right = mps_to_rpm (right_cmd);
		
		/* 一秒最大变化量 */
    float max_delta = MAX_DELTA_FIXED;  // 预计算的 30 * 0.02 = 0.6

    /* 梯度加速（ramp） - 对目标 rpm_left/right 做速率限制，更新持久 ramp 值 */
    float deltaL = rpm_left - ramp_left;
    if (fabsf(deltaL) <= max_delta) ramp_left = rpm_left;
    else ramp_left += (deltaL > 0.0f ? max_delta : -max_delta);

    float deltaR = rpm_right - ramp_right;
    if (fabsf(deltaR) <= max_delta) ramp_right = rpm_right;
    else ramp_right += (deltaR > 0.0f ? max_delta : -max_delta);

    /* 可选：对 ramp 做轻微平滑（用 ramp 而不是 rpm，避免覆盖差速）
       若你不想额外平滑，可以直接发送 ramp_* */
    smooth_left  = smooth_left * SMOOTH_ALPHA_INV + ramp_left * SMOOTH_ALPHA;
    smooth_right = smooth_right * SMOOTH_ALPHA_INV + ramp_right * SMOOTH_ALPHA;

    /* 限幅保护（基于 smooth 输出） */
    if (smooth_left >  WHEEL_MAX_RPM)  smooth_left =  WHEEL_MAX_RPM;
    if (smooth_left < -WHEEL_MAX_RPM)  smooth_left = -WHEEL_MAX_RPM;
    if (smooth_right >  WHEEL_MAX_RPM) smooth_right =  WHEEL_MAX_RPM;
    if (smooth_right < -WHEEL_MAX_RPM) smooth_right = -WHEEL_MAX_RPM;

    /* 转成整型并发送（注意：发送的是 smooth/ramp，不是原来的 rpm） */
    int16_t send_left  = (int16_t)roundf(smooth_left);
    int16_t send_right = (int16_t)roundf(smooth_right);

   /* ============ ★ 转成 DEC 并发送 CAN（关键补充）★ ============ */
    int32_t dec_left  = rpm_to_dec(send_left);
    int32_t dec_right = rpm_to_dec(send_right);

    // ----------------- 分配给驱动轮 -----------------
    g_moving_wheel.L_Vel = dec_left; //	m/s	转换为	°/s ，0.314为2*pi*r（轮子半径）
    g_moving_wheel.R_Vel = dec_right;
   

}

