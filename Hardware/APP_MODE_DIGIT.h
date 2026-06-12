#ifndef __APP_MODE_DIGIT_H
#define __APP_MODE_DIGIT_H

#include "stm32f10x.h"

/**
  * 函    数：启动模式5数字顺序运动
  * 参    数：无
  * 返 回 值：无
  */
void APP_MODE_DIGIT_Start(void);

/**
  * 函    数：停止模式5数字顺序运动
  * 参    数：无
  * 返 回 值：无
  */
void APP_MODE_DIGIT_Stop(void);

/**
  * 函    数：模式5周期任务
  * 参    数：无
  * 返 回 值：无
  * 说    明：由 AppRun 每 10ms 调用一次
  */
void APP_MODE_DIGIT_Task(void);

/**
  * 函    数：读取模式5是否正在运行
  * 参    数：无
  * 返 回 值：1 正在运行，0 未运行
  */
uint8_t APP_MODE_DIGIT_IsRunning(void);

#endif
