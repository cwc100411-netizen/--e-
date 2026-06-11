#ifndef __TRACKING_H
#define __TRACKING_H

#include "stm32f10x.h"

/**
  * 函    数：初始化追踪模块
  * 参    数：无
  * 返 回 值：无
  */
void Tracking_Init(void);

/**
  * 函    数：追踪任务函数
  * 参    数：无
  * 返 回 值：无
  * 说    明：需要在主循环中按 10ms 左右周期调用
  */
void Tracking_Task(void);

/**
  * 函    数：开启或关闭追踪
  * 参    数：Enable 1 表示开启，0 表示关闭
  * 返 回 值：无
  */
void Tracking_Enable(uint8_t Enable);

/**
  * 函    数：开启或关闭四边形循迹目标生成
  * 参    数：Enable 1 表示开启，0 表示关闭
  * 返 回 值：无
  */
void Tracking_EnableQuadrilateral(uint8_t Enable);

/**
  * 函    数：读取四边形循迹是否已经完成一圈
  * 参    数：无
  * 返 回 值：1 表示已完成，0 表示未完成
  */
uint8_t Tracking_IsQuadrilateralFinished(void);

/**
  * 函    数：开启或关闭圆形循迹目标生成
  * 参    数：Enable 1 表示开启，0 表示关闭
  * 返 回 值：无
  */
void Tracking_EnableCircle(uint8_t Enable);

/**
  * 函    数：开启或关闭数字顺序击打
  * 参    数：Enable 1 表示开启，0 表示关闭
  * 返 回 值：无
  * 说    明：新增功能，等待 MaixCam 发送 1~5 中心点后按顺序击打
  */
void Tracking_EnableDigit(uint8_t Enable);

/**
  * 函    数：设置圆形循迹参数
  * 参    数：CenterX, CenterY 圆心坐标
  * 参    数：StartX, StartY 圆上的起始点坐标
  * 返 回 值：无
  * 说    明：支持任意圆上一点作为起点，例如圆心(133,130)、起点(131,56)
  */
void Tracking_SetCircle(int16_t CenterX, int16_t CenterY, int16_t StartX, int16_t StartY);

/**
  * 函    数：设置四边形每条边的分段数
  * 参    数：Section 每条边分成多少小段，数值越大循迹越慢
  * 返 回 值：无
  */
void Tracking_SetQuadrilateralSection(uint16_t Section);

/**
  * 函    数：重新等待串口矩形锁定
  * 参    数：无
  * 返 回 值：无
  * 说    明：更换矩形框或需要重新识别路径时调用
  */
void Tracking_ResetQuadrilateralLock(void);

/**
  * 函    数：手动设置四边形循迹的四个顶点
  * 参    数：X1~Y4 四个顶点坐标
  * 返 回 值：无
  * 说    明：请按循迹顺序输入四个点，程序按 P1->P2->P3->P4->P1 运行
  */
void Tracking_SetQuadrilateral(int16_t X1, int16_t Y1,
                               int16_t X2, int16_t Y2,
                               int16_t X3, int16_t Y3,
                               int16_t X4, int16_t Y4);

/**
  * 函    数：设置固定追踪目标坐标
  * 参    数：TargetX 目标 x 坐标
  * 参    数：TargetY 目标 y 坐标
  * 返 回 值：无
  */
void Tracking_SetTarget(int16_t TargetX, int16_t TargetY);

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
