#include "AppRun.h"
#include "Stepper.h"
#include "Key.h"
#include "Timer.h"
#include "Delay.h"
#include "Tracking.h"

static void App_StartRectangleTracking(void)
{
    /* 重新启动前先停止旧任务，避免电机沿旧速度继续运动 */
    Tracking_Enable(0);
    Tracking_EnableQuadrilateral(0);
    Stepper_StopBoth();

    /* 使能两个电机，等待驱动和电机吸合稳定 */
    Stepper_Enable(STEPPER_MOTOR_1);
    Stepper_Enable(STEPPER_MOTOR_2);
    Delay_ms(300);

    /* 清掉等待期间积累的 10ms 定时标志 */
    while (Timer_GetFlag())
    {
    }

    /* 重新等待摄像头串口发送的矩形胶带黑框坐标 */
    Tracking_ResetQuadrilateralLock();

    /* 设置每条边分段数，数值越大循迹越慢，越小越快 */
    Tracking_SetQuadrilateralSection(100);

    Tracking_EnableQuadrilateral(1);
    Tracking_Enable(1);
}

void App_Run(void)
{
    /* PA0 按下一次，开始使用摄像头识别到的矩形胶带黑框循迹 */
    if (Key_GetNum() == 1)
    {
        App_StartRectangleTracking();
    }

    /* TIM4 每 10ms 置位一次标志，主循环里执行追踪任务 */
    if (Timer_GetFlag())
    {
        Tracking_Task();
    }
}
