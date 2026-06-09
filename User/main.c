#include "stm32f10x.h"
#include "Stepper.h"
#include "Timer.h"
#include "Serial.h"
#include "Delay.h"
#include "Tracking.h"

int main(void)
{
    Stepper_Init();
    Timer_Init();
    Serial_Init();
    Tracking_Init();

    /* 上电先停止，避免电机误动作 */
    Tracking_Enable(0);
    Tracking_EnableQuadrilateral(0);
    Stepper_StopBoth();

    /* 使能两个步进电机 */
    Stepper_Enable(STEPPER_MOTOR_1);
    Stepper_Enable(STEPPER_MOTOR_2);
    Delay_ms(300);

    /* 清掉等待期间产生的定时标志 */
    while (Timer_GetFlag())
    {
    }

    /* 重新等待摄像头发送矩形黑框四个角点 */
    Tracking_ResetQuadrilateralLock();

    /* 设置矩形每条边分段数，数值越小走得越快 */
    Tracking_SetQuadrilateralSection(10);

    /* 开启矩形黑框循迹 */
    Tracking_EnableQuadrilateral(1);
    Tracking_Enable(1);

    while (1)
    {
        /* TIM4 每 10ms 置位一次，周期调用循迹任务 */
        if (Timer_GetFlag())
        {
            Tracking_Task();
        }
    }
}
