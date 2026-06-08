#ifndef __APP_RUN_H
#define __APP_RUN_H

#include "stm32f10x.h"

/**
  * 函    数：应用任务调度
  * 参    数：无
  * 返 回 值：无
  * 说    明：放在 main 的 while 循环中反复调用
  */
void App_Run(void);

/**
  * 函    数：获取当前模式
  * 参    数：无
  * 返 回 值：0 空闲，1 高速边线，2 正矩形，3 任意角度矩形，4 内切圆，5 数字顺序
  */
uint8_t App_GetMode(void);

#endif
