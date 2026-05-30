#include "BKMotor.h"
#include "CANDrive.h"
#include "fdcan.h"
#include "MoveControll.h"
#include "sbus.h"

/* ============ RPM到编码器值转换函数 ============ */
int32_t rpm_to_dec(float rpm)
{
    float factor = (512.0f * ENCODER_RES) / 1875.0f;
    float dec_f = rpm * factor;
    if (dec_f >= 0.0f) dec_f += 0.5f;
    else dec_f -= 0.5f;
    return (int32_t)dec_f;
}



/* ============ SBUS -> RPM 映射（包含死区） ============
   返回值：-max_rpm .. +max_rpm
   注意：传入的 ch_min/ch_center/ch_max 取决于具体通道标定
*/

float map_sbus_to_rpm(int16_t ch_val, int16_t ch_min, int16_t ch_center, int16_t ch_max, float max_rpm, float deadzone)
{
    float norm = 0.0f;

    // 若超出边界先裁剪（防止除零）
    if (ch_val > ch_max) ch_val = ch_max;
    if (ch_val < ch_min) ch_val = ch_min;

    // 死区处理：若在中间死区，直接返回 0
    if (fabsf((float)ch_val - (float)ch_center) <= deadzone)
        return 0.0f;

    if (ch_val > ch_center)
    {
        // 上半区：ch_center+deadzone .. ch_max  -> 0 .. +1
        float denom = (float)(ch_max - (ch_center + deadzone));
        if (denom <= 0.0f) return 0.0f;
        norm = (float)(ch_val - (ch_center + deadzone)) / denom;
    }
    else
    {
        // 下半区：ch_min .. ch_center-deadzone -> -1 .. 0
        float denom = (float)((ch_center - deadzone) - ch_min);
        if (denom <= 0.0f) return 0.0f;
        norm = (float)(ch_val - (ch_center - deadzone)) / denom; // 负值
    }

    // 限幅
    if (norm > 1.0f) norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;

    return norm * max_rpm;
}


void Main_Loop(void)
{
    static uint32_t last_ctrl_ms = 0;
    uint32_t now = HAL_GetTick();
	
	   // 处理SBUS数据（使用你的标志）
      Sbus_Process_Task();
	
    // 只在间隔达到 10ms 时才执行控制任务
    if (now - last_ctrl_ms >= 10)//100hz
    {
        last_ctrl_ms = now;
        ControlLoop_Init();
    }

    // 其他主循环任务...
}

