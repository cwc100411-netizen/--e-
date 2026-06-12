#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

#define KEY1_GPIO_RCC       RCC_APB2Periph_GPIOA
#define KEY1_GPIO           GPIOA
#define KEY1_PIN            GPIO_Pin_0

#define KEY2_GPIO_RCC       RCC_APB2Periph_GPIOA
#define KEY2_GPIO           GPIOA
#define KEY2_PIN            GPIO_Pin_3

#define KEY3_GPIO_RCC       RCC_APB2Periph_GPIOA
#define KEY3_GPIO           GPIOA
#define KEY3_PIN            GPIO_Pin_6

#define KEY4_GPIO_RCC       RCC_APB2Periph_GPIOB
#define KEY4_GPIO           GPIOB
#define KEY4_PIN            GPIO_Pin_11

void Key_Init(void);
uint8_t Key_GetNum(void);
void Key_Tick(void);

#endif
