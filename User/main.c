#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"
#include "Stepper.h"

int main(void)
{
//  OLED_Init();
//  Key_Init();                 // PB1 已作为电机2 DIR，测试双电机时先不要初始化按键1
    Serial_Init();

    Stepper_Init();              // 初始化 GPIO、PWM、默认细分和默认速度，并默认使能电机
	Stepper_TurnAngleBoth(30, 0);     // 水平和竖直方向电机同时正转 90 度
	Stepper_TurnAngleBoth(0, 30);
	Stepper_TurnAngleBoth(-30, 0);
	Stepper_TurnAngleBoth(0, -30);

	
    while (1)	
    {
//        Stepper_TurnAngleBoth(90, 90);     // 两个电机同时正转 90 度
//        Delay_ms(1000);

//        Stepper_TurnAngleBoth(-90, -90);   // 两个电机同时反转 90 度
//        Delay_ms(1000);
    }
}
