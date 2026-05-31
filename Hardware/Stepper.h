#ifndef __STEPPER_H
#define __STEPPER_H

#include "stm32f10x.h"

/* 步进电机驱动器引脚配置：默认使用 STEP/DIR/EN 三线驱动器 */
#define STEPPER_GPIO_RCC              RCC_APB2Periph_GPIOB
#define STEPPER_GPIO                  GPIOB
#define STEPPER_STEP_PIN              GPIO_Pin_6
#define STEPPER_DIR_PIN               GPIO_Pin_7
#define STEPPER_EN_PIN                GPIO_Pin_8

/* 当前驱动模块按高电平使能处理，如模块相反请改为 Bit_RESET */
#define STEPPER_ENABLE_LEVEL          Bit_SET

/* 正转方向电平，方向相反时把 Bit_SET 改为 Bit_RESET */
#define STEPPER_DIR_CW_LEVEL          Bit_SET

/* 42 步进电机常见步距角为 1.8 度，这里用 0.1 度为单位保存 */
#define STEPPER_BASE_STEP_ANGLE_X10   18

/* 细分数必须和驱动器拨码或 MS 引脚设置一致 */
#define STEPPER_DEFAULT_MICROSTEP     1

/* STEP 高低电平各保持的时间，数值越大转速越慢 */
#define STEPPER_DEFAULT_PULSE_US      500

#define STEPPER_DIR_CW                0
#define STEPPER_DIR_CCW               1

void Stepper_Init(void);
void Stepper_Enable(void);
void Stepper_Disable(void);
void Stepper_SetDir(uint8_t Dir);
void Stepper_SetMicroStep(uint16_t MicroStep);
void Stepper_SetPulseUs(uint16_t PulseUs);
void Stepper_RunSteps(uint32_t StepNum);
void Stepper_TurnAngle(int32_t Angle);
void Stepper_TurnAngleX10(int32_t AngleX10);
uint32_t Stepper_AngleToStepsX10(uint32_t AngleX10);

#endif
