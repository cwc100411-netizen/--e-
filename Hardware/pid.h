#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float ErrorLast;
	float ErrorLast2;
	float Output;
} PID_TypeDef;

/**
  * 函    数：配置 PID 参数并清零 PID 状态
  * 参    数：Pid 要配置的 PID 结构体
  * 参    数：Kp 比例系数
  * 参    数：Ki 积分系数
  * 参    数：Kd 微分系数
  * 返 回 值：无
  */
void PID_Config(PID_TypeDef *Pid, float Kp, float Ki, float Kd);

/**
  * 函    数：清零 PID 历史误差和输出
  * 参    数：Pid 要复位的 PID 结构体
  * 返 回 值：无
  */
void PID_Reset(PID_TypeDef *Pid);

/**
  * 函    数：执行一次增量式 PID 计算
  * 参    数：Pid 要更新的 PID 结构体
  * 参    数：Error 当前误差
  * 返 回 值：PID 本次输出值
  */
float PID_Update(PID_TypeDef *Pid, float Error);

#endif
