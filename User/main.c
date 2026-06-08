#include "stm32f10x.h"
#include "Stepper.h"
#include "Key.h"
#include "Timer.h"
#include "Serial.h"
#include "Tracking.h"
#include "AppRun.h"

int main(void)
{
    Stepper_Init();
    Key_Init();
    Timer_Init();
    Serial_Init();
    Tracking_Init();

    /* 上电后默认不运动，等待按键启动 */
    Tracking_EnableQuadrilateral(0);
    Tracking_Enable(0);
    Stepper_StopBoth();

    while (1)
    {
        App_Run();
    }
}
