#ifndef __APP_MODE_EDGE_FAST_H
#define __APP_MODE_EDGE_FAST_H

#include "stm32f10x.h"

/**
  * 函    数：启动模式1高速边线运动
  * 参    数：无
  * 返 回 值：无
  */
void APP_MODE_EDGE_FAST_Start(void);

/**
  * 函    数：停止模式1高速边线运动
  * 参    数：无
  * 返 回 值：无
  */
void APP_MODE_EDGE_FAST_Stop(void);

/**
  * 函    数：模式1高速边线运动周期任务
  * 参    数：无
  * 返 回 值：无
  * 说    明：由 AppRun 每 10ms 调用一次
  */
void APP_MODE_EDGE_FAST_Task(void);

/**
  * 函    数：读取模式1是否正在运行
  * 参    数：无
  * 返 回 值：1 正在运行，0 未运行
  */
uint8_t APP_MODE_EDGE_FAST_IsRunning(void);

#endif
