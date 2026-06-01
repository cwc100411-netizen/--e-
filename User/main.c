#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"
#include "Stepper.h"

int main(void)
{
//    OLED_Init();
//    Key_Init();                 // PB1 已作为电机2 DIR，测试双电机时先不要初始化按键1
    Serial_Init();

    Stepper_Init();              // 初始化 GPIO，并默认使能电机
    Stepper_SetMicroStep(STEPPER_MOTOR_1, 16);    // 电机1细分，必须和驱动器拨码一致
    Stepper_SetMicroStep(STEPPER_MOTOR_2, 16);    // 电机2细分，必须和驱动器拨码一致
    Stepper_SetPulseUs(STEPPER_MOTOR_1, 500);     // 电机1速度，数值越大越慢
    Stepper_SetPulseUs(STEPPER_MOTOR_2, 500);     // 电机2速度，数值越大越慢
	Stepper_TurnAngleBoth(90, 90);     // 两个电机同时正转 90 度

    while (1)
    {
//        Stepper_TurnAngleBoth(90, 90);     // 两个电机同时正转 90 度
//        Delay_ms(1000);

//        Stepper_TurnAngleBoth(-90, -90);   // 两个电机同时反转 90 度
//        Delay_ms(1000);
    }
}
