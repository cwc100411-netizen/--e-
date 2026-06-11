#include "stm32f10x.h"
#include "Stepper.h"
#include "Timer.h"
#include "Serial.h"
#include "Key.h"
#include "OLED.h"
#include "Tracking.h"
#include "AppRun.h"

static void Main_ShowMode(void);

int main(void)
{
    Stepper_Init();
    Serial_Init();
    Key_Init();
    OLED_Init();
    Timer_Init();
    Tracking_Init();

    /* 上电先停止，避免电机误动作 */
    Tracking_Enable(0);
    Tracking_EnableQuadrilateral(0);
    Tracking_EnableCircle(0);
    Tracking_EnableDigit(0);
    Stepper_StopBoth();

    Main_ShowMode();

    while (1)
    {
        App_Run();
        Main_ShowMode();
    }
}

static void Main_ShowMode(void)
{
    /* 当前 OLED 字库只支持常用 ASCII 字符，所以模式名使用英文 */
    switch (App_GetMode())
    {
        case 0:
            OLED_ShowString(1, 1, "mode:idle       ");
            break;

        case 1:
            OLED_ShowString(1, 1, "mode:edge_fast  ");
            break;

        case 2:
            OLED_ShowString(1, 1, "mode:rect_normal");
            break;

        case 3:
            OLED_ShowString(1, 1, "mode:rect_any   ");
            break;

        case 4:
            OLED_ShowString(1, 1, "mode:circle     ");
            break;

        case 5:
            OLED_ShowString(1, 1, "mode:digit      ");
            break;

        default:
            OLED_ShowString(1, 1, "mode:unknown    ");
            break;
    }
}
