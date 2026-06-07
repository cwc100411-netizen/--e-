#include "stm32f10x.h"
#include "Stepper.h"
#include "Key.h"
#include "Timer.h"
#include "Delay.h"

static void Square_OpenLoop_RunOnce(uint32_t XSteps, uint32_t YSteps)
{
    /* 第一条边：左上 -> 右上，电机1控制水平轴 */
    Stepper_SetDir(STEPPER_MOTOR_1, STEPPER_DIR_CW);
    Stepper_RunSteps(STEPPER_MOTOR_1, XSteps);
    Delay_ms(100);

    /* 第二条边：右上 -> 右下，电机2控制竖直轴 */
    Stepper_SetDir(STEPPER_MOTOR_2, STEPPER_DIR_CW);
    Stepper_RunSteps(STEPPER_MOTOR_2, YSteps);
    Delay_ms(100);

    /* 第三条边：右下 -> 左下 */
    Stepper_SetDir(STEPPER_MOTOR_1, STEPPER_DIR_CCW);
    Stepper_RunSteps(STEPPER_MOTOR_1, XSteps);
    Delay_ms(100);

    /* 第四条边：左下 -> 左上 */
    Stepper_SetDir(STEPPER_MOTOR_2, STEPPER_DIR_CCW);
    Stepper_RunSteps(STEPPER_MOTOR_2, YSteps);

    Stepper_StopBoth();
}

int main(void)
{
    uint32_t SquareXSteps = 500;   /* 水平方向步数，需要按实际屏幕标定 */
    uint32_t SquareYSteps = 500;   /* 竖直方向步数，需要按实际屏幕标定 */
    uint16_t PulseUs = 1000;        /* 速度参数，数值越小速度越快 */

    Stepper_Init();
    Key_Init();
    Timer_Init();

    Stepper_SetPulseUs(STEPPER_MOTOR_1, PulseUs);
    Stepper_SetPulseUs(STEPPER_MOTOR_2, PulseUs);
    Stepper_StopBoth();

    while (1)
    {
        /* PA0 按键由 TIM4 中断扫描，按下一次运行一圈开环矩形框 */
        if (Key_GetNum() == 1)
        {
            Square_OpenLoop_RunOnce(SquareXSteps, SquareYSteps);
        }
    }
}
