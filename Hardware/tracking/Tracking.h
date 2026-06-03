#ifndef __TRACKING_H
#define __TRACKING_H

#include "stm32f10x.h"

#define TRACKING_CONTROL_PERIOD_MS    10

/**
  * 函    数：初始化追踪模块
  * 参    数：无
  * 返 回 值：无
  */
void Tracking_Init(void);

/**
  * 函    数：设置追踪目标坐标
  * 参    数：TargetX 目标 x 坐标
  * 参    数：TargetY 目标 y 坐标
  * 返 回 值：无
  */
void Tracking_SetTarget(int16_t TargetX, int16_t TargetY);

/**
  * 函    数：开启或关闭追踪
  * 参    数：Enable 1 表示开启，0 表示关闭
  * 返 回 值：无
  */
void Tracking_Enable(uint8_t Enable);

/**
  * 函    数：追踪任务函数
  * 参    数：无
  * 返 回 值：无
  * 说    明：需要在主循环中周期调用，推荐周期为 TRACKING_CONTROL_PERIOD_MS
  */
void Tracking_Task(void);

/**
  * 函    数：获取最近一次有效激光点坐标
  * 参    数：X 保存 x 坐标的指针，可传入 0
  * 参    数：Y 保存 y 坐标的指针，可传入 0
  * 返 回 值：1 表示坐标有效，0 表示当前没有有效激光点
  */
uint8_t Tracking_GetLaserPoint(uint8_t *X, uint8_t *Y);

/**
  * 函    数：获取当前目标坐标
  * 参    数：X 保存目标 x 坐标的指针，可传入 0
  * 参    数：Y 保存目标 y 坐标的指针，可传入 0
  * 返 回 值：无
  */
void Tracking_GetTarget(uint8_t *X, uint8_t *Y);

#endif
