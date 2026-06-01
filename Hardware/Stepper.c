#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Stepper.h"

static uint16_t Stepper1_MicroStep = STEPPER_DEFAULT_MICROSTEP;
static uint16_t Stepper2_MicroStep = STEPPER_DEFAULT_MICROSTEP;
static uint16_t Stepper1_PulseUs = STEPPER_DEFAULT_PULSE_US;
static uint16_t Stepper2_PulseUs = STEPPER_DEFAULT_PULSE_US;

/**
  * 函    数：取反 BitAction 电平
  * 参    数：BitVal 要取反的电平
  * 返 回 值：取反后的电平
  */
static BitAction Stepper_ReverseBit(BitAction BitVal)
{
	if (BitVal == Bit_SET)
	{
		return Bit_RESET;
	}
	else
	{
		return Bit_SET;
	}
}

/**
  * 函    数：获取指定电机的 STEP 引脚
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 返 回 值：STEP 引脚
  */
static uint16_t Stepper_GetStepPin(uint8_t Motor)
{
	if (Motor == STEPPER_MOTOR_2)
	{
		return STEPPER2_STEP_PIN;
	}
	else
	{
		return STEPPER1_STEP_PIN;
	}
}

/**
  * 函    数：获取指定电机的 DIR 引脚
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 返 回 值：DIR 引脚
  */
static uint16_t Stepper_GetDirPin(uint8_t Motor)
{
	if (Motor == STEPPER_MOTOR_2)
	{
		return STEPPER2_DIR_PIN;
	}
	else
	{
		return STEPPER1_DIR_PIN;
	}
}

/**
  * 函    数：获取指定电机的 EN 引脚
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 返 回 值：EN 引脚
  */
static uint16_t Stepper_GetEnPin(uint8_t Motor)
{
	if (Motor == STEPPER_MOTOR_2)
	{
		return STEPPER2_EN_PIN;
	}
	else
	{
		return STEPPER1_EN_PIN;
	}
}

/**
  * 函    数：获取指定电机的使能有效电平
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 返 回 值：使能有效电平
  */
static BitAction Stepper_GetEnableLevel(uint8_t Motor)
{
	if (Motor == STEPPER_MOTOR_2)
	{
		return STEPPER2_ENABLE_LEVEL;
	}
	else
	{
		return STEPPER1_ENABLE_LEVEL;
	}
}

/**
  * 函    数：获取指定电机的正转方向电平
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 返 回 值：正转方向电平
  */
static BitAction Stepper_GetDirCwLevel(uint8_t Motor)
{
	if (Motor == STEPPER_MOTOR_2)
	{
		return STEPPER2_DIR_CW_LEVEL;
	}
	else
	{
		return STEPPER1_DIR_CW_LEVEL;
	}
}

/**
  * 函    数：获取指定电机的细分数
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 返 回 值：细分数
  */
static uint16_t Stepper_GetMicroStep(uint8_t Motor)
{
	if (Motor == STEPPER_MOTOR_2)
	{
		return Stepper2_MicroStep;
	}
	else
	{
		return Stepper1_MicroStep;
	}
}

/**
  * 函    数：获取指定电机的 STEP 脉冲时间
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 返 回 值：STEP 高低电平各保持的时间，单位 us
  */
static uint16_t Stepper_GetPulseUs(uint8_t Motor)
{
	if (Motor == STEPPER_MOTOR_2)
	{
		return Stepper2_PulseUs;
	}
	else
	{
		return Stepper1_PulseUs;
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

	/* 开启 GPIO 和 AFIO 时钟，PB3 默认是 JTAG 引脚，需要关闭 JTAG 后才能作为普通 GPIO 使用 */
	RCC_APB2PeriphClockCmd(STEPPER_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	/* STEP、DIR、EN 引脚配置为推挽输出 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = STEPPER1_STEP_PIN | STEPPER1_DIR_PIN | STEPPER1_EN_PIN |
	                              STEPPER2_STEP_PIN | STEPPER2_DIR_PIN | STEPPER2_EN_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(STEPPER_GPIO, &GPIO_InitStructure);

	GPIO_ResetBits(STEPPER_GPIO, STEPPER1_STEP_PIN | STEPPER2_STEP_PIN);
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
	GPIO_WriteBit(STEPPER_GPIO, Stepper_GetEnPin(Motor), Stepper_GetEnableLevel(Motor));
}

/**
  * 函    数：关闭步进电机驱动器
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 返 回 值：无
  */
void Stepper_Disable(uint8_t Motor)
{
	GPIO_WriteBit(STEPPER_GPIO, Stepper_GetEnPin(Motor), Stepper_ReverseBit(Stepper_GetEnableLevel(Motor)));
}

/**
  * 函    数：设置步进电机方向
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：Dir 方向，STEPPER_DIR_CW 为正转，STEPPER_DIR_CCW 为反转
  * 返 回 值：无
  */
void Stepper_SetDir(uint8_t Motor, uint8_t Dir)
{
	if (Dir == STEPPER_DIR_CW)
	{
		GPIO_WriteBit(STEPPER_GPIO, Stepper_GetDirPin(Motor), Stepper_GetDirCwLevel(Motor));
	}
	else
	{
		GPIO_WriteBit(STEPPER_GPIO, Stepper_GetDirPin(Motor), Stepper_ReverseBit(Stepper_GetDirCwLevel(Motor)));
	}
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
  * 函    数：输出一个 STEP 脉冲
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 返 回 值：无
  */
static void Stepper_Pulse(uint8_t Motor)
{
	uint16_t StepPin;
	uint16_t PulseUs;

	StepPin = Stepper_GetStepPin(Motor);
	PulseUs = Stepper_GetPulseUs(Motor);

	GPIO_SetBits(STEPPER_GPIO, StepPin);
	Delay_us(PulseUs);
	GPIO_ResetBits(STEPPER_GPIO, StepPin);
	Delay_us(PulseUs);
}

/**
  * 函    数：同时输出一组 STEP 脉冲
  * 参    数：StepPinMask 需要输出脉冲的 STEP 引脚组合
  * 参    数：PulseUs STEP 高低电平各保持的时间，单位 us
  * 返 回 值：无
  */
static void Stepper_PulsePins(uint16_t StepPinMask, uint16_t PulseUs)
{
	if (StepPinMask == 0)
	{
		return;
	}

	GPIO_SetBits(STEPPER_GPIO, StepPinMask);
	Delay_us(PulseUs);
	GPIO_ResetBits(STEPPER_GPIO, StepPinMask);
	Delay_us(PulseUs);
}

/**
  * 函    数：按指定步数转动
  * 参    数：Motor 电机编号，STEPPER_MOTOR_1 或 STEPPER_MOTOR_2
  * 参    数：StepNum 要输出的 STEP 脉冲数量
  * 返 回 值：无
  */
void Stepper_RunSteps(uint8_t Motor, uint32_t StepNum)
{
	uint32_t i;

	for (i = 0; i < StepNum; i ++)
	{
		Stepper_Pulse(Motor);
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
	uint32_t i;
	uint32_t MaxStepNum;
	uint32_t Motor1Count = 0;
	uint32_t Motor2Count = 0;
	uint16_t StepPinMask;
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

	/* 同时运行时使用较慢的脉冲时间，保证两个驱动器都能可靠识别 STEP */
	if (Stepper1_PulseUs > Stepper2_PulseUs)
	{
		PulseUs = Stepper1_PulseUs;
	}
	else
	{
		PulseUs = Stepper2_PulseUs;
	}

	for (i = 0; i < MaxStepNum; i ++)
	{
		StepPinMask = 0;

		Motor1Count += Motor1StepNum;
		if (Motor1Count >= MaxStepNum)
		{
			Motor1Count -= MaxStepNum;
			StepPinMask |= STEPPER1_STEP_PIN;
		}

		Motor2Count += Motor2StepNum;
		if (Motor2Count >= MaxStepNum)
		{
			Motor2Count -= MaxStepNum;
			StepPinMask |= STEPPER2_STEP_PIN;
		}

		Stepper_PulsePins(StepPinMask, PulseUs);
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
	uint32_t StepNum;

	StepNum = (AngleX10 * Stepper_GetMicroStep(Motor) + STEPPER_BASE_STEP_ANGLE_X10 / 2) / STEPPER_BASE_STEP_ANGLE_X10;
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
	Stepper_TurnAngleX10(Motor, Angle * 10);
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

	if (AngleX10 >= 0)
	{
		Stepper_SetDir(Motor, STEPPER_DIR_CW);
		AngleAbsX10 = (uint32_t)AngleX10;
	}
	else
	{
		Stepper_SetDir(Motor, STEPPER_DIR_CCW);
		AngleAbsX10 = (uint32_t)(-AngleX10);
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
	uint32_t Motor1AngleAbsX10;
	uint32_t Motor2AngleAbsX10;

	if (Motor1Angle >= 0)
	{
		Stepper_SetDir(STEPPER_MOTOR_1, STEPPER_DIR_CW);
		Motor1AngleAbsX10 = (uint32_t)(Motor1Angle * 10);
	}
	else
	{
		Stepper_SetDir(STEPPER_MOTOR_1, STEPPER_DIR_CCW);
		Motor1AngleAbsX10 = (uint32_t)(-Motor1Angle * 10);
	}

	if (Motor2Angle >= 0)
	{
		Stepper_SetDir(STEPPER_MOTOR_2, STEPPER_DIR_CW);
		Motor2AngleAbsX10 = (uint32_t)(Motor2Angle * 10);
	}
	else
	{
		Stepper_SetDir(STEPPER_MOTOR_2, STEPPER_DIR_CCW);
		Motor2AngleAbsX10 = (uint32_t)(-Motor2Angle * 10);
	}

	Motor1StepNum = Stepper_AngleToStepsX10(STEPPER_MOTOR_1, Motor1AngleAbsX10);
	Motor2StepNum = Stepper_AngleToStepsX10(STEPPER_MOTOR_2, Motor2AngleAbsX10);
	Stepper_RunStepsBoth(Motor1StepNum, Motor2StepNum);
}
