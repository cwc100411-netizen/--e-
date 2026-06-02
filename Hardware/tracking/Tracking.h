#ifndef __TRACKING_H
#define __TRACKING_H

#include "stm32f10x.h"

#define TRACKING_CONTROL_PERIOD_MS    10

void Tracking_Init(void);
void Tracking_SetTarget(int16_t TargetX, int16_t TargetY);
void Tracking_Enable(uint8_t Enable);
void Tracking_Task(void);
uint8_t Tracking_GetLaserPoint(uint8_t *X, uint8_t *Y);
void Tracking_GetTarget(uint8_t *X, uint8_t *Y);

#endif
