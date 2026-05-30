#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Stepper.h"

static uint16_t Stepper_MicroStep = STEPPER_DEFAULT_MICROSTEP;
static uint16_t Stepper_PulseUs = STEPPER_DEFAULT_PULSE_US;

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
  * 函    数：步进电机模块初始化
  * 参    数：无
  * 返 回 值：无
  */
void Stepper_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 开启 GPIO 时钟 */
	RCC_APB2PeriphClockCmd(STEPPER_GPIO_RCC, ENABLE);

	/* STEP、DIR、EN 引脚配置为推挽输出 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = STEPPER_STEP_PIN | STEPPER_DIR_PIN | STEPPER_EN_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(STEPPER_GPIO, &GPIO_InitStructure);

	GPIO_ResetBits(STEPPER_GPIO, STEPPER_STEP_PIN);
	Stepper_SetDir(STEPPER_DIR_CW);
	Stepper_Enable();
}

/**
  * 函    数：使能步进电机驱动器
  * 参    数：无
  * 返 回 值：无
  */
void Stepper_Enable(void)
{
	GPIO_WriteBit(STEPPER_GPIO, STEPPER_EN_PIN, STEPPER_ENABLE_LEVEL);
}

/**
  * 函    数：关闭步进电机驱动器
  * 参    数：无
  * 返 回 值：无
  */
void Stepper_Disable(void)
{
	GPIO_WriteBit(STEPPER_GPIO, STEPPER_EN_PIN, Stepper_ReverseBit(STEPPER_ENABLE_LEVEL));
}

/**
  * 函    数：设置步进电机方向
  * 参    数：Dir 方向，STEPPER_DIR_CW 为正转，STEPPER_DIR_CCW 为反转
  * 返 回 值：无
  */
void Stepper_SetDir(uint8_t Dir)
{
	if (Dir == STEPPER_DIR_CW)
	{
		GPIO_WriteBit(STEPPER_GPIO, STEPPER_DIR_PIN, STEPPER_DIR_CW_LEVEL);
	}
	else
	{
		GPIO_WriteBit(STEPPER_GPIO, STEPPER_DIR_PIN, Stepper_ReverseBit(STEPPER_DIR_CW_LEVEL));
	}
}

/**
  * 函    数：设置驱动器细分数
  * 参    数：MicroStep 细分数，例如 1、2、4、8、16、32
  * 返 回 值：无
  * 注意事项：此值必须和驱动器硬件拨码或 MS 引脚配置一致，否则角度会不准
  */
void Stepper_SetMicroStep(uint16_t MicroStep)
{
	if (MicroStep == 0)
	{
		MicroStep = 1;
	}

	Stepper_MicroStep = MicroStep;
}

/**
  * 函    数：设置 STEP 脉冲间隔
  * 参    数：PulseUs STEP 高低电平各保持的时间，单位 us
  * 返 回 值：无
  */
void Stepper_SetPulseUs(uint16_t PulseUs)
{
	if (PulseUs < 2)
	{
		PulseUs = 2;
	}

	Stepper_PulseUs = PulseUs;
}

/**
  * 函    数：输出一个 STEP 脉冲
  * 参    数：无
  * 返 回 值：无
  */
static void Stepper_Pulse(void)
{
	GPIO_SetBits(STEPPER_GPIO, STEPPER_STEP_PIN);
	Delay_us(Stepper_PulseUs);
	GPIO_ResetBits(STEPPER_GPIO, STEPPER_STEP_PIN);
	Delay_us(Stepper_PulseUs);
}

/**
  * 函    数：按指定步数转动
  * 参    数：StepNum 要输出的 STEP 脉冲数量
  * 返 回 值：无
  */
void Stepper_RunSteps(uint32_t StepNum)
{
	uint32_t i;

	for (i = 0; i < StepNum; i ++)
	{
		Stepper_Pulse();
	}
}

/**
  * 函    数：将角度换算为步数
  * 参    数：AngleX10 角度绝对值，单位为 0.1 度，例如 900 表示 90.0 度
  * 返 回 值：对应的 STEP 脉冲数量
  */
uint32_t Stepper_AngleToStepsX10(uint32_t AngleX10)
{
	uint32_t StepNum;

	StepNum = (AngleX10 * Stepper_MicroStep + STEPPER_BASE_STEP_ANGLE_X10 / 2) / STEPPER_BASE_STEP_ANGLE_X10;
	return StepNum;
}

/**
  * 函    数：按整数角度转动
  * 参    数：Angle 要转动的角度，单位为度，正数正转，负数反转
  * 返 回 值：无
  */
void Stepper_TurnAngle(int32_t Angle)
{
	Stepper_TurnAngleX10(Angle * 10);
}
