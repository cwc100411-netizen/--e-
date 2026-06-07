#include "stm32f10x.h"
#include "Key.h"

static volatile uint8_t Key_Num;

void Key_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* PA0 配置为上拉输入，按键按下时接地为低电平 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

uint8_t Key_GetNum(void)
{
	uint8_t Temp;

	__disable_irq();
	if (Key_Num)
	{
		Temp = Key_Num;
		Key_Num = 0;
		__enable_irq();
		return Temp;
	}
	__enable_irq();
	return 0;
}

static uint8_t Key_GetState(void)
{
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0)
	{
		return 1;
	}
	return 0;
}

void Key_Tick(void)
{
	static uint8_t Count;
	static uint8_t CurrState, PrevState;

	Count ++;
	/* TIM4 每 10ms 调用一次 Key_Tick，计数 2 次约为 20ms 消抖 */
	if (Count >= 2)
	{
		Count = 0;

		PrevState = CurrState;
		CurrState = Key_GetState();

		/* 检测松开到按下，按键稳定按下后立即形成一次事件 */
		if (CurrState != 0 && PrevState == 0)
		{
			Key_Num = 1;
		}
	}
}
