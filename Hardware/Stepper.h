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
#define STEPPER1_DIR_PIN              GPIO_Pin_4
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

/**
  * 函    数：初始化步进电机模块
  * 参    数：无
  * 返 回 值：无
  */
void Stepper_Init(void);

/**
  * 函    数：使能指定步进电机驱动器
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 返 回 值：无
  */
void Stepper_Enable(uint8_t Motor);

/**
  * 函    数：关闭指定步进电机驱动器
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 返 回 值：无
  */
void Stepper_Disable(uint8_t Motor);

/**
  * 函    数：设置指定步进电机方向
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：Dir 方向，STEPPER_DIR_CW 正转，STEPPER_DIR_CCW 反转
  * 返 回 值：无
  */
void Stepper_SetDir(uint8_t Motor, uint8_t Dir);

/**
  * 函    数：设置驱动器细分数
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：MicroStep 细分数，需要和驱动器拨码一致
  * 返 回 值：无
  */
void Stepper_SetMicroStep(uint8_t Motor, uint16_t MicroStep);

/**
  * 函    数：设置固定步数模式下的 STEP 半周期
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：PulseUs STEP 高低电平各保持的时间，单位 us
  * 返 回 值：无
  */
void Stepper_SetPulseUs(uint8_t Motor, uint16_t PulseUs);

/**
  * 函    数：按指定步数转动单个电机
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：StepNum 要输出的 STEP 脉冲数量
  * 返 回 值：无
  * 说    明：该函数会阻塞等待电机走完
  */
void Stepper_RunSteps(uint8_t Motor, uint32_t StepNum);

/**
  * 函    数：同时按指定步数转动两个电机
  * 参    数：Motor1StepNum 电机1要输出的 STEP 脉冲数量
  * 参    数：Motor2StepNum 电机2要输出的 STEP 脉冲数量
  * 返 回 值：无
  * 说    明：该函数会阻塞等待两个电机都走完
  */
void Stepper_RunStepsBoth(uint32_t Motor1StepNum, uint32_t Motor2StepNum);

/**
  * 函    数：停止指定电机输出 STEP 脉冲
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 返 回 值：无
  */
void Stepper_Stop(uint8_t Motor);

/**
  * 函    数：同时停止两个电机输出 STEP 脉冲
  * 参    数：无
  * 返 回 值：无
  */
void Stepper_StopBoth(void);

/**
  * 函    数：按速度连续驱动指定电机
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：Speed 速度，单位 step/s，正数正转，负数反转，0 停止
  * 返 回 值：无
  * 说    明：该函数不阻塞，适合追踪闭环反复调用
  */
void Stepper_SetSpeed(uint8_t Motor, int16_t Speed);

/**
  * 函    数：按整数角度转动单个电机
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：Angle 要转动的角度，单位度，正数正转，负数反转
  * 返 回 值：无
  */
void Stepper_TurnAngle(uint8_t Motor, int32_t Angle);

/**
  * 函    数：按 0.1 度精度转动单个电机
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：AngleX10 要转动的角度，单位 0.1 度
  * 返 回 值：无
  */
void Stepper_TurnAngleX10(uint8_t Motor, int32_t AngleX10);

/**
  * 函    数：同时按整数角度转动两个电机
  * 参    数：Motor1Angle 电机1要转动的角度，单位度，正数正转，负数反转
  * 参    数：Motor2Angle 电机2要转动的角度，单位度，正数正转，负数反转
  * 返 回 值：无
  */
void Stepper_TurnAngleBoth(int32_t Motor1Angle, int32_t Motor2Angle);

/**
  * 函    数：将角度换算成 STEP 脉冲数量
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：AngleX10 角度绝对值，单位 0.1 度
  * 返 回 值：对应的 STEP 脉冲数量
  */
uint32_t Stepper_AngleToStepsX10(uint8_t Motor, uint32_t AngleX10);

#endif
