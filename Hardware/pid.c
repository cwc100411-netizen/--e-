#include "pid.h"

/**
  * 函    数：配置 PID 参数并清零 PID 状态
  * 参    数：Pid 要配置的 PID 结构体
  * 参    数：Kp 比例系数
  * 参    数：Ki 积分系数
  * 参    数：Kd 微分系数
  * 返 回 值：无
  */
void PID_Config(PID_TypeDef *Pid, float Kp, float Ki, float Kd)
{
	if (Pid == 0)
	{
		return;
	}

	Pid->Kp = Kp;
	Pid->Ki = Ki;
	Pid->Kd = Kd;
	PID_Reset(Pid);
}

/**
  * 函    数：清零 PID 历史误差和输出
  * 参    数：Pid 要复位的 PID 结构体
  * 返 回 值：无
  * 说    明：重新开始控制前调用，避免旧误差继续影响输出
  */
void PID_Reset(PID_TypeDef *Pid)
{
	if (Pid == 0)
	{
		return;
	}

	Pid->ErrorLast = 0.0f;
	Pid->ErrorLast2 = 0.0f;
	Pid->Output = 0.0f;
}

/**
  * 函    数：执行一次增量式 PID 计算
  * 参    数：Pid 要更新的 PID 结构体
  * 参    数：Error 当前误差
  * 返 回 值：PID 本次输出值
  */
float PID_Update(PID_TypeDef *Pid, float Error)
{
	float Delta;

	if (Pid == 0)
	{
		return 0.0f;
	}

	/* 增量式 PID：输出值由调用者按实际对象解释，例如 Tracking 中作为电机速度 */
	Delta = Pid->Kp * (Error - Pid->ErrorLast)
	      + Pid->Ki * Error
	      + Pid->Kd * (Error - 2.0f * Pid->ErrorLast + Pid->ErrorLast2);

	Pid->Output += Delta;
	Pid->ErrorLast2 = Pid->ErrorLast;
	Pid->ErrorLast = Error;

	return Pid->Output;
}
