#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Serial.h"
#include "Key.h"
#include "Stepper.h"
#include "Tracking.h"
#include "Timer.h"

int main(void)
{
//  Key_Init();                 // PB1 已作为电机2 DIR，测试双电机时先不要初始化按键1
	Serial_Init();
	Stepper_Init();
	Tracking_Init();

	/* 串口矩形锁定模式下等待摄像头发送矩形坐标；手动测试固定四边形时再打开下一行 */
//	Tracking_SetQuadrilateral(120, 30, 210, 120, 120, 210, 30, 1+++++++++++++++++++++++++20);
	/* 每条边分成 200 段，主循环 10ms 调用一次时，一圈大约 8 秒；启动后会先在 P1 停留约 2 秒 */
	Tracking_SetQuadrilateralSection(200);
	Tracking_EnableQuadrilateral(1);
	Tracking_Enable(1);
	Timer_Init();
//	Stepper_SetSpeed(STEPPER_MOTOR_2, 200);
//	Delay_ms(1000);
//	Stepper_StopBoth();
//	Stepper_TurnAngleBoth(90, 0);
	while (1)
	{
		if (Timer_GetFlag())
		{
			Tracking_Task();
		}
	}
}
