#ifndef __APP_MODE_CIRCLE_H
#define __APP_MODE_CIRCLE_H

#include "stm32f10x.h"

/**
  * 函    数：启动模式4边线内切圆运动
  * 参    数：无
  * 返 回 值：无
  */
void APP_MODE_CIRCLE_Start(void);

/**
  * 函    数：停止模式4边线内切圆运动
  * 参    数：无
  * 返 回 值：无
  */
void APP_MODE_CIRCLE_Stop(void);

/**
  * 函    数：模式4周期任务
  * 参    数：无
  * 返 回 值：无
  * 说    明：由 AppRun 每 10ms 调用一次
  */
void APP_MODE_CIRCLE_Task(void);

/**
  * 函    数：读取模式4是否正在运行
  * 参    数：无
  * 返 回 值：1 正在运行，0 未运行
  */
uint8_t APP_MODE_CIRCLE_IsRunning(void);

/**
  * 函    数：设置圆形运动参数
  * 参    数：CenterX, CenterY 圆心坐标
  * 参    数：StartX, StartY 圆上的起始点坐标
  * 返 回 值：无
  */
void APP_MODE_CIRCLE_SetCircle(int16_t CenterX, int16_t CenterY, int16_t StartX, int16_t StartY);

#endif
