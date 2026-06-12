#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

void LED_Init(void);
void LED1_ON(void);
void LED1_OFF(void);
void LED1_Turn(void);
void LED2_ON(void);
void LED2_OFF(void);
void LED2_Turn(void);
void Laser_Init(void);
void Laser_ON(void);
void Laser_OFF(void);
void Laser_Set(uint8_t State);

#endif
