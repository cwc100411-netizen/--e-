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
    /* 当前模式名继续使用英文，方便保持原来的显示内容 */
    uint8_t Mode;
    static uint8_t LastMode = 0xFF;

    Mode = App_GetMode();
    if (Mode == LastMode)
    {
        return;
    }
    LastMode = Mode;

    /* 新版 OLED 库先写入显存，最后必须调用 OLED_Update 才会刷新到屏幕 */
    OLED_Clear();

    switch (Mode)
    {
        case 0:
            OLED_ShowString(0, 0, "mode:idle       ", OLED_8X16);
            break;

        case 1:
            OLED_ShowString(0, 0, "mode:edge_fast  ", OLED_8X16);
            break;

        case 2:
            OLED_ShowString(0, 0, "mode:rect_normal", OLED_8X16);
            break;

        case 3:
            OLED_ShowString(0, 0, "mode:rect_any   ", OLED_8X16);
            break;

        case 4:
            OLED_ShowString(0, 0, "mode:circle     ", OLED_8X16);
            break;

        case 5:
            OLED_ShowString(0, 0, "mode:digit      ", OLED_8X16);
            break;

        default:
            OLED_ShowString(0, 0, "mode:unknown    ", OLED_8X16);
            break;
    }

    OLED_Update();
}
