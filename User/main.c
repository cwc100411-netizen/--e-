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

	/* 按循迹顺序设置四个点：P1->P2->P3->P4->P1，可直接修改这 8 个坐标做测试 */
	Tracking_SetQuadrilateral(120, 30, 210, 120, 120, 210, 30, 120);
	/* 每条边分成 200 段，主循环 10ms 调用一次时，一圈大约 8 秒；启动后会先在 P1 停留约 2 秒 */
	Tracking_SetQuadrilateralSection(200);
	Tracking_EnableQuadrilateral(1);
	Tracking_Enable(1);
//	Stepper_SetSpeed(STEPPER_MOTOR_2, 200);
//	Delay_ms(1000);
//	Stepper_StopBoth();
//	Stepper_TurnAngleBoth(90, 0);
	while (1)
	{
		
		Tracking_Task();
		Delay_ms(TRACKING_CONTROL_PERIOD_MS);
	}
}
