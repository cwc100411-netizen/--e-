#include "stm32f10x.h"                  // Device header
#include "Timer.h"
#include "Key.h"

static volatile uint8_t Timer_FlagCount = 0;

/**
  * 函    数：初始化 TIM4 定时中断
  * 参    数：无
  * 返 回 值：无
  * 说    明：TIM4 每 10ms 进入一次中断，中断里只置位标志，不执行耗时任务
  */
void Timer_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM4 属于 APB1 外设，这里只开启 TIM4 时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_InternalClockConfig(TIM4);

	/* 72MHz / 72 = 1MHz，计数 10000 次为 10ms */
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM4, ENABLE);
}

/**
  * 函    数：读取 TIM4 定时标志
  * 参    数：无
  * 返 回 值：1 表示到达一次 10ms 周期，0 表示还未到周期
  */
uint8_t Timer_GetFlag(void)
{
	uint8_t Flag = 0;

	__disable_irq();
	if (Timer_FlagCount > 0)
	{
		Timer_FlagCount--;
		Flag = 1;
	}
	__enable_irq();

	return Flag;
}

/**
  * 函    数：TIM4 中断函数
  * 参    数：无
  * 返 回 值：无
  * 说    明：中断中只记录节拍，不调用 Tracking_Task，避免在中断中执行耗时操作
  */
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		/* 定时扫描按键，函数内部不能有阻塞延时 */
		Key_Tick();

		if (Timer_FlagCount < 255)
		{
			Timer_FlagCount++;
		}

		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}
