#ifndef __STEPPER_H
#define __STEPPER_H

#include "stm32f10x.h"

/* 两路步进电机编号 */
#define STEPPER_MOTOR_1               0
#define STEPPER_MOTOR_2               1

/* 步进电机驱动器引脚配置：默认使用 STEP/DIR/EN 三线驱动器
   STEP 使用定时器 PWM 输出，DIR 和 EN 仍然使用普通 GPIO 输出 */
#define STEPPER_STEP_GPIO_RCC         RCC_APB2Periph_GPIOA
#define STEPPER_STEP_GPIO             GPIOA
#define STEPPER_DIR_EN_GPIO_RCC       RCC_APB2Periph_GPIOB
#define STEPPER_DIR_EN_GPIO           GPIOB

/* 电机1引脚：PA1 STEP(TIM2_CH2)，PB7 DIR，PB8 EN */
#define STEPPER1_STEP_PIN             GPIO_Pin_1
#define STEPPER1_DIR_PIN              GPIO_Pin_7
#define STEPPER1_EN_PIN               GPIO_Pin_8

/* 电机2引脚：PA7 STEP(TIM3_CH2)，PB1 DIR，PB3 EN */
#define STEPPER2_STEP_PIN             GPIO_Pin_7
#define STEPPER2_DIR_PIN              GPIO_Pin_1
#define STEPPER2_EN_PIN               GPIO_Pin_3

/* STEP PWM 定时器配置：PA1 对应 TIM2_CH2，PA7 对应 TIM3_CH2 */
#define STEPPER1_PWM_TIM              TIM2
#define STEPPER2_PWM_TIM              TIM3
#define STEPPER1_PWM_RCC              RCC_APB1Periph_TIM2
#define STEPPER2_PWM_RCC              RCC_APB1Periph_TIM3
#define STEPPER_PWM_TIM_PRESCALER     71

/* 当前驱动模块按高电平使能处理，如模块相反请改为 Bit_RESET */
#define STEPPER1_ENABLE_LEVEL         Bit_SET
#define STEPPER2_ENABLE_LEVEL         Bit_SET

/* 正转方向电平，方向相反时把 Bit_SET 改为 Bit_RESET */
#define STEPPER1_DIR_CW_LEVEL         Bit_SET
#define STEPPER2_DIR_CW_LEVEL         Bit_SET

/* 42 步进电机常见步距角为 1.8 度，这里用 0.1 度为单位保存 */
#define STEPPER_BASE_STEP_ANGLE_X10   18

/* 默认细分数，必须和驱动器拨码或 MS 引脚设置一致 */
#define STEPPER_DEFAULT_MICROSTEP     32

/* STEP 高低电平各保持的时间，数值越大转速越慢 */
#define STEPPER_DEFAULT_PULSE_US      500

/* DIR 方向信号建立时间，方向切换后等待一小段时间再发送 STEP 脉冲 */
#define STEPPER_DIR_SETUP_DELAY_US    10

#define STEPPER_DIR_CW                0
#define STEPPER_DIR_CCW               1

void Stepper_Init(void);
void Stepper_Enable(uint8_t Motor);
void Stepper_Disable(uint8_t Motor);
void Stepper_SetDir(uint8_t Motor, uint8_t Dir);
void Stepper_SetMicroStep(uint8_t Motor, uint16_t MicroStep);
void Stepper_SetPulseUs(uint8_t Motor, uint16_t PulseUs);
void Stepper_RunSteps(uint8_t Motor, uint32_t StepNum);
void Stepper_RunStepsBoth(uint32_t Motor1StepNum, uint32_t Motor2StepNum);
void Stepper_Stop(uint8_t Motor);
void Stepper_StopBoth(void);
void Stepper_SetSpeed(uint8_t Motor, int16_t Speed);
void Stepper_TurnAngle(uint8_t Motor, int32_t Angle);
void Stepper_TurnAngleX10(uint8_t Motor, int32_t AngleX10);
void Stepper_TurnAngleBoth(int32_t Motor1Angle, int32_t Motor2Angle);
uint32_t Stepper_AngleToStepsX10(uint8_t Motor, uint32_t AngleX10);

#endif
