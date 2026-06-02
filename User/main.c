#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Serial.h"
#include "Key.h"
#include "Stepper.h"
#include "Tracking.h"

int main(void)
{
//  Key_Init();                 // PB1 已作为电机2 DIR，测试双电机时先不要初始化按键1
	Serial_Init();
	Stepper_Init();
	Tracking_Init();
	Tracking_SetTarget(120, 120);
	Tracking_Enable(1);

	while (1)
	{
		Tracking_Task();
		Delay_ms(TRACKING_CONTROL_PERIOD_MS);
	}
}
