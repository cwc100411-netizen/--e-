#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"
#include "Stepper.h"

int main(void)
{
//    OLED_Init();
    Key_Init();
    Serial_Init();

    Stepper_Init();              // 初始化 GPIO，并默认使能电机
    Stepper_SetMicroStep(16);     // 设置细分，必须和驱动器拨码一致
    Stepper_SetPulseUs(100);     // 设置速度，数值越大越慢

    while (1)
    {
        Stepper_TurnAngle(90);   // 正转 90 度
        Delay_ms(1000);

        Stepper_TurnAngle(-90);  // 反转 90 度
        Delay_ms(1000);
    }
}
