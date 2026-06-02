#include "stm32f10x.h"                  // Device header
#include "Stepper.h"
#include "Delay.h"

static uint16_t Stepper1_MicroStep = STEPPER_DEFAULT_MICROSTEP;
static uint16_t Stepper2_MicroStep = STEPPER_DEFAULT_MICROSTEP;
static uint16_t Stepper1_PulseUs = STEPPER_DEFAULT_PULSE_US;
static uint16_t Stepper2_PulseUs = STEPPER_DEFAULT_PULSE_US;

static volatile uint32_t Stepper1_TargetStepNum = 0;
static volatile uint32_t Stepper2_TargetStepNum = 0;
static volatile uint32_t Stepper1_CurrentStepNum = 0;
static volatile uint32_t Stepper2_CurrentStepNum = 0;
static volatile uint8_t Stepper1_Busy = 0;
static volatile uint8_t Stepper2_Busy = 0;

/**
  * 函    数：限制 PWM 周期范围
  * 参    数：PeriodUs PWM 周期，单位 us
  * 返 回 值：定时器可使用的周期，单位 us
  */
static uint16_t Stepper_LimitPeriodUs(uint32_t PeriodUs)
{
	if (PeriodUs < 4)
	{
		PeriodUs = 4;
	}
	if (PeriodUs > 65535)
	{
		PeriodUs = 65535;
	}
	return (uint16_t)PeriodUs;
}

/**
  * 函    数：初始化单个 PWM 定时器
  * 参    数：TIMx 要初始化的定时器
  * 返 回 值：无
  */
static void Stepper_PWMInitTimer(TIM_TypeDef *TIMx)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* 定时器 72MHz 分频到 1MHz，计数 1 次就是 1us */
	TIM_TimeBaseInitStructure.TIM_Prescaler = STEPPER_PWM_TIM_PRESCALER;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = STEPPER_DEFAULT_PULSE_US * 2 - 1;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseInitStructure);

	/* PWM1 模式：计数值小于 CCR2 时输出高电平，其余时间输出低电平 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC2Init(TIMx, &TIM_OCInitStructure);

	TIM_ITConfig(TIMx, TIM_IT_Update, DISABLE);
	TIM_Cmd(TIMx, DISABLE);
	TIM_ClearITPendingBit(TIMx, TIM_IT_Update);
}

/**
  * 函    数：停止指定电机的 PWM 输出
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 返 回 值：无
  */
static void Stepper_StopPwm(uint8_t Motor)
{
	TIM_TypeDef *TIMx;

	if (Motor == STEPPER_MOTOR_2)
	{
		TIMx = STEPPER2_PWM_TIM;
	}
	else
	{
		TIMx = STEPPER1_PWM_TIM;
	}

	TIM_Cmd(TIMx, DISABLE);
	TIM_ITConfig(TIMx, TIM_IT_Update, DISABLE);
	TIM_SetCompare2(TIMx, 0);
	TIM_SetCounter(TIMx, 0);
	TIM_ClearITPendingBit(TIMx, TIM_IT_Update);

	if (Motor == STEPPER_MOTOR_2)
	{
		Stepper2_Busy = 0;
		Stepper2_TargetStepNum = 0;
		Stepper2_CurrentStepNum = 0;
	}
	else
	{
		Stepper1_Busy = 0;
		Stepper1_TargetStepNum = 0;
		Stepper1_CurrentStepNum = 0;
	}
}

/**
  * 函    数：启动指定电机的 PWM 脉冲输出
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：StepNum 要输出的 STEP 脉冲数量
  * 参    数：PeriodUs PWM 周期，单位 us
  * 返 回 值：无
  */
static void Stepper_StartPwm(uint8_t Motor, uint32_t StepNum, uint32_t PeriodUs)
{
	TIM_TypeDef *TIMx;
	uint16_t TimerPeriodUs;

	if (StepNum == 0)
	{
		return;
	}

	if (Motor == STEPPER_MOTOR_2)
	{
		TIMx = STEPPER2_PWM_TIM;
	}
	else
	{
		TIMx = STEPPER1_PWM_TIM;
	}

	TimerPeriodUs = Stepper_LimitPeriodUs(PeriodUs);

	Stepper_StopPwm(Motor);

	TIM_SetAutoreload(TIMx, TimerPeriodUs - 1);
	TIM_SetCompare2(TIMx, TimerPeriodUs / 2);
	TIM_SetCounter(TIMx, 0);
	/* ARR 和 CCR2 未开启预装载，直接写入即可；不主动产生更新事件，避免影响步数统计 */
	TIM_ClearITPendingBit(TIMx, TIM_IT_Update);

	if (Motor == STEPPER_MOTOR_2)
	{
		Stepper2_TargetStepNum = StepNum;
		Stepper2_CurrentStepNum = 0;
		Stepper2_Busy = 1;
	}
	else
	{
		Stepper1_TargetStepNum = StepNum;
		Stepper1_CurrentStepNum = 0;
		Stepper1_Busy = 1;
	}

	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIMx, ENABLE);
}

/**
  * 函    数：TIM2 中断函数，用于统计电机1 PWM 脉冲数量
  * 参    数：无
  * 返 回 值：无
  * 注意事项：中断中只计数和停止 PWM，不写阻塞延时
  */
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(STEPPER1_PWM_TIM, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(STEPPER1_PWM_TIM, TIM_IT_Update);

		if (Stepper1_Busy)
		{
			Stepper1_CurrentStepNum ++;
			if (Stepper1_CurrentStepNum >= Stepper1_TargetStepNum)
			{
				Stepper_StopPwm(STEPPER_MOTOR_1);
			}
		}
	}
}

/**
  * 函    数：TIM3 中断函数，用于统计电机2 PWM 脉冲数量
  * 参    数：无
  * 返 回 值：无
  * 注意事项：中断中只计数和停止 PWM，不写阻塞延时
  */
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(STEPPER2_PWM_TIM, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(STEPPER2_PWM_TIM, TIM_IT_Update);

		if (Stepper2_Busy)
		{
			Stepper2_CurrentStepNum ++;
			if (Stepper2_CurrentStepNum >= Stepper2_TargetStepNum)
			{
				Stepper_StopPwm(STEPPER_MOTOR_2);
			}
		}
	}
}

/**
  * 函    数：步进电机模块初始化
  * 参    数：无
  * 返 回 值：无
  */
void Stepper_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 开启 GPIO、AFIO 和 PWM 定时器时钟，PB3 默认是 JTAG 引脚，需要关闭 JTAG 后才能作为普通 GPIO 使用 */
	RCC_APB2PeriphClockCmd(STEPPER_STEP_GPIO_RCC | STEPPER_DIR_EN_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(STEPPER1_PWM_RCC | STEPPER2_PWM_RCC, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	/* STEP 引脚配置为复用推挽输出，由 TIM2_CH2 和 TIM3_CH2 输出 PWM 脉冲 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = STEPPER1_STEP_PIN | STEPPER2_STEP_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(STEPPER_STEP_GPIO, &GPIO_InitStructure);

	/* DIR、EN 引脚配置为普通推挽输出 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = STEPPER1_DIR_PIN | STEPPER1_EN_PIN |
	                              STEPPER2_DIR_PIN | STEPPER2_EN_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(STEPPER_DIR_EN_GPIO, &GPIO_InitStructure);

	Stepper_PWMInitTimer(STEPPER1_PWM_TIM);
	Stepper_PWMInitTimer(STEPPER2_PWM_TIM);

	/* TIM2 和 TIM3 中断只负责统计 STEP 脉冲数，到达目标步数后停止 PWM */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* 默认细分和速度统一放在初始化中，main.c 不需要重复设置 */
	Stepper_SetMicroStep(STEPPER_MOTOR_1, STEPPER_DEFAULT_MICROSTEP);
	Stepper_SetMicroStep(STEPPER_MOTOR_2, STEPPER_DEFAULT_MICROSTEP);
	Stepper_SetPulseUs(STEPPER_MOTOR_1, STEPPER_DEFAULT_PULSE_US);
	Stepper_SetPulseUs(STEPPER_MOTOR_2, STEPPER_DEFAULT_PULSE_US);

	GPIO_ResetBits(STEPPER_STEP_GPIO, STEPPER1_STEP_PIN | STEPPER2_STEP_PIN);
	Stepper_SetDir(STEPPER_MOTOR_1, STEPPER_DIR_CW);
	Stepper_SetDir(STEPPER_MOTOR_2, STEPPER_DIR_CW);
	Stepper_Enable(STEPPER_MOTOR_1);
	Stepper_Enable(STEPPER_MOTOR_2);
}

/**
  * 函    数：使能步进电机驱动器
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 返 回 值：无
  */
void Stepper_Enable(uint8_t Motor)
{
	if (Motor == STEPPER_MOTOR_2)
	{
		GPIO_WriteBit(STEPPER_DIR_EN_GPIO, STEPPER2_EN_PIN, STEPPER2_ENABLE_LEVEL);
	}
	else
	{
		GPIO_WriteBit(STEPPER_DIR_EN_GPIO, STEPPER1_EN_PIN, STEPPER1_ENABLE_LEVEL);
	}
}

/**
  * 函    数：关闭步进电机驱动器
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 返 回 值：无
  */
void Stepper_Disable(uint8_t Motor)
{
	Stepper_StopPwm(Motor);

	if (Motor == STEPPER_MOTOR_2)
	{
		GPIO_WriteBit(STEPPER_DIR_EN_GPIO, STEPPER2_EN_PIN, (STEPPER2_ENABLE_LEVEL == Bit_SET) ? Bit_RESET : Bit_SET);
	}
	else
	{
		GPIO_WriteBit(STEPPER_DIR_EN_GPIO, STEPPER1_EN_PIN, (STEPPER1_ENABLE_LEVEL == Bit_SET) ? Bit_RESET : Bit_SET);
	}
}

/**
  * 函    数：设置步进电机方向
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：Dir 方向，STEPPER_DIR_CW 为正转，STEPPER_DIR_CCW 为反转
  * 返 回 值：无
  */
void Stepper_SetDir(uint8_t Motor, uint8_t Dir)
{
	uint16_t DirPin;
	BitAction DirLevel;

	if (Motor == STEPPER_MOTOR_2)
	{
		DirPin = STEPPER2_DIR_PIN;
		DirLevel = STEPPER2_DIR_CW_LEVEL;
	}
	else
	{
		DirPin = STEPPER1_DIR_PIN;
		DirLevel = STEPPER1_DIR_CW_LEVEL;
	}

	if (Dir != STEPPER_DIR_CW)
	{
		DirLevel = (DirLevel == Bit_SET) ? Bit_RESET : Bit_SET;
	}

	GPIO_WriteBit(STEPPER_DIR_EN_GPIO, DirPin, DirLevel);

	/* 方向电平改变后等待一小段时间，再输出 STEP 脉冲 */
	Delay_us(STEPPER_DIR_SETUP_DELAY_US);
}

/**
  * 函    数：设置驱动器细分数
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：MicroStep 细分数，例如 1、2、4、8、16、32
  * 返 回 值：无
  * 注意事项：此值必须和驱动器硬件拨码或 MS 引脚配置一致，否则角度会不准
  */
void Stepper_SetMicroStep(uint8_t Motor, uint16_t MicroStep)
{
	if (MicroStep == 0)
	{
		MicroStep = 1;
	}

	if (Motor == STEPPER_MOTOR_2)
	{
		Stepper2_MicroStep = MicroStep;
	}
	else
	{
		Stepper1_MicroStep = MicroStep;
	}
}

/**
  * 函    数：设置 STEP 脉冲间隔
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：PulseUs STEP 高低电平各保持的时间，单位 us
  * 返 回 值：无
  */
void Stepper_SetPulseUs(uint8_t Motor, uint16_t PulseUs)
{
	if (PulseUs < 2)
	{
		PulseUs = 2;
	}

	if (Motor == STEPPER_MOTOR_2)
	{
		Stepper2_PulseUs = PulseUs;
	}
	else
	{
		Stepper1_PulseUs = PulseUs;
	}
}

/**
  * 函    数：按指定步数转动
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：StepNum 要输出的 STEP 脉冲数量
  * 返 回 值：无
  */
void Stepper_RunSteps(uint8_t Motor, uint32_t StepNum)
{
	uint32_t PeriodUs;
	uint16_t PulseUs;

	if (StepNum == 0)
	{
		return;
	}

	if (Motor == STEPPER_MOTOR_2)
	{
		PulseUs = Stepper2_PulseUs;
	}
	else
	{
		PulseUs = Stepper1_PulseUs;
	}

	/* PulseUs 表示高低电平各保持的时间，所以一个 PWM 周期为 2 * PulseUs */
	PeriodUs = (uint32_t)PulseUs * 2;
	Stepper_StartPwm(Motor, StepNum, PeriodUs);

	if (Motor == STEPPER_MOTOR_2)
	{
		while (Stepper2_Busy)
		{
		}
	}
	else
	{
		while (Stepper1_Busy)
		{
		}
	}
}

/**
  * 函    数：两路电机同时按指定步数转动
  * 参    数：Motor1StepNum 电机1要输出的 STEP 脉冲数量
  * 参    数：Motor2StepNum 电机2要输出的 STEP 脉冲数量
  * 返 回 值：无
  */
void Stepper_RunStepsBoth(uint32_t Motor1StepNum, uint32_t Motor2StepNum)
{
	uint32_t MaxStepNum;
	uint32_t BasePeriodUs;
	uint32_t Motor1PeriodUs;
	uint32_t Motor2PeriodUs;
	uint16_t PulseUs;

	if (Motor1StepNum > Motor2StepNum)
	{
		MaxStepNum = Motor1StepNum;
	}
	else
	{
		MaxStepNum = Motor2StepNum;
	}

	if (MaxStepNum == 0)
	{
		return;
	}

	/* 同时运行时使用较慢的基础速度，保证两个驱动器都能可靠识别 STEP */
	if (Stepper1_PulseUs > Stepper2_PulseUs)
	{
		PulseUs = Stepper1_PulseUs;
	}
	else
	{
		PulseUs = Stepper2_PulseUs;
	}

	BasePeriodUs = (uint32_t)PulseUs * 2;
	Motor1PeriodUs = BasePeriodUs;
	Motor2PeriodUs = BasePeriodUs;

	/* 两个电机步数不同时，少步数电机降低 PWM 频率，尽量让两个电机同时结束 */
	if (Motor1StepNum != 0)
	{
		Motor1PeriodUs = (uint32_t)(((uint64_t)BasePeriodUs * MaxStepNum + Motor1StepNum / 2) / Motor1StepNum);
		Stepper_StartPwm(STEPPER_MOTOR_1, Motor1StepNum, Motor1PeriodUs);
	}

	if (Motor2StepNum != 0)
	{
		Motor2PeriodUs = (uint32_t)(((uint64_t)BasePeriodUs * MaxStepNum + Motor2StepNum / 2) / Motor2StepNum);
		Stepper_StartPwm(STEPPER_MOTOR_2, Motor2StepNum, Motor2PeriodUs);
	}

	while (Stepper1_Busy || Stepper2_Busy)
	{
	}
}

/**
  * 函    数：将角度换算为步数
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：AngleX10 角度绝对值，单位为 0.1 度，例如 900 表示 90.0 度
  * 返 回 值：对应的 STEP 脉冲数量
  */
uint32_t Stepper_AngleToStepsX10(uint8_t Motor, uint32_t AngleX10)
{
	uint16_t MicroStep;
	uint32_t StepNum;

	if (Motor == STEPPER_MOTOR_2)
	{
		MicroStep = Stepper2_MicroStep;
	}
	else
	{
		MicroStep = Stepper1_MicroStep;
	}

	StepNum = (uint32_t)(((uint64_t)AngleX10 * MicroStep + STEPPER_BASE_STEP_ANGLE_X10 / 2) / STEPPER_BASE_STEP_ANGLE_X10);
	return StepNum;
}

/**
  * 函    数：按整数角度转动
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：Angle 要转动的角度，单位为度，正数正转，负数反转
  * 返 回 值：无
  */
void Stepper_TurnAngle(uint8_t Motor, int32_t Angle)
{
	int32_t AngleX10;

	/* 防止极端大角度执行 Angle * 10 时发生有符号溢出 */
	if (Angle > 214748364)
	{
		AngleX10 = 2147483640L;
	}
	else if (Angle < -214748364)
	{
		AngleX10 = -2147483640L;
	}
	else
	{
		AngleX10 = Angle * 10;
	}

	Stepper_TurnAngleX10(Motor, AngleX10);
}

/**
  * 函    数：按 0.1 度精度转动
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：AngleX10 要转动的角度，单位为 0.1 度，正数正转，负数反转
  * 返 回 值：无
  */
void Stepper_TurnAngleX10(uint8_t Motor, int32_t AngleX10)
{
	uint32_t StepNum;
	uint32_t AngleAbsX10;

	if (AngleX10 == 0)
	{
		return;
	}

	if (AngleX10 > 0)
	{
		Stepper_SetDir(Motor, STEPPER_DIR_CW);
		AngleAbsX10 = (uint32_t)AngleX10;
	}
	else
	{
		Stepper_SetDir(Motor, STEPPER_DIR_CCW);
		AngleAbsX10 = 0 - (uint32_t)AngleX10;
	}

	StepNum = Stepper_AngleToStepsX10(Motor, AngleAbsX10);
	Stepper_RunSteps(Motor, StepNum);
}

/**
  * 函    数：两路电机同时按整数角度转动
  * 参    数：Motor1Angle 电机1要转动的角度，单位为度，正数正转，负数反转
  * 参    数：Motor2Angle 电机2要转动的角度，单位为度，正数正转，负数反转
  * 返 回 值：无
  */
void Stepper_TurnAngleBoth(int32_t Motor1Angle, int32_t Motor2Angle)
{
	uint32_t Motor1StepNum;
	uint32_t Motor2StepNum;
	uint32_t Motor1AngleAbsX10 = 0;
	uint32_t Motor2AngleAbsX10 = 0;

	if (Motor1Angle > 0)
	{
		Stepper_SetDir(STEPPER_MOTOR_1, STEPPER_DIR_CW);
		Motor1AngleAbsX10 = (uint32_t)((uint64_t)Motor1Angle * 10);
	}
	else if (Motor1Angle < 0)
	{
		Stepper_SetDir(STEPPER_MOTOR_1, STEPPER_DIR_CCW);
		Motor1AngleAbsX10 = (uint32_t)((uint64_t)(0 - (uint32_t)Motor1Angle) * 10);
	}

	if (Motor2Angle > 0)
	{
		Stepper_SetDir(STEPPER_MOTOR_2, STEPPER_DIR_CW);
		Motor2AngleAbsX10 = (uint32_t)((uint64_t)Motor2Angle * 10);
	}
	else if (Motor2Angle < 0)
	{
		Stepper_SetDir(STEPPER_MOTOR_2, STEPPER_DIR_CCW);
		Motor2AngleAbsX10 = (uint32_t)((uint64_t)(0 - (uint32_t)Motor2Angle) * 10);
	}

	Motor1StepNum = Stepper_AngleToStepsX10(STEPPER_MOTOR_1, Motor1AngleAbsX10);
	Motor2StepNum = Stepper_AngleToStepsX10(STEPPER_MOTOR_2, Motor2AngleAbsX10);
	Stepper_RunStepsBoth(Motor1StepNum, Motor2StepNum);
}
