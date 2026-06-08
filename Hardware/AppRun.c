#include "AppRun.h"
#include "Stepper.h"
#include "Key.h"
#include "Timer.h"
#include "Delay.h"
#include "Tracking.h"

typedef enum
{
    APP_MODE_IDLE = 0,          /* 空闲 */
    APP_MODE_EDGE_FAST,         /* 高速边线运动 */
    APP_MODE_RECT_NORMAL,       /* 正着矩形胶带黑框循迹 */
    APP_MODE_RECT_ANY,          /* 任意角度矩形胶带黑框循迹 */
    APP_MODE_CIRCLE,            /* 边线内切圆运动 */
    APP_MODE_DIGIT,             /* 识别数字顺序运动 */
    APP_MODE_NUM
} App_ModeTypeDef;

static App_ModeTypeDef App_Mode = APP_MODE_IDLE;

static void App_StopAll(void);
static void App_PrepareTracking(void);
static void App_SwitchMode(void);
static void App_StartCurrentMode(void);
static void App_StartEdgeFast(void);
static void App_StartRectangleTracking(uint16_t Section);
static void App_ResetToCenter(void);

static void App_StopAll(void)
{
    Tracking_Enable(0);
    Tracking_EnableQuadrilateral(0);
    Stepper_StopBoth();
}

static void App_PrepareTracking(void)
{
    /* 重新启动前先停止旧任务，避免电机沿旧速度继续运动 */
    App_StopAll();

    /* 使能两个电机，等待驱动和电机吸合稳定 */
    Stepper_Enable(STEPPER_MOTOR_1);
    Stepper_Enable(STEPPER_MOTOR_2);
    Delay_ms(300);

    /* 清掉等待期间积累的 10ms 定时标志 */
    while (Timer_GetFlag())
    {
    }
}

static void App_SwitchMode(void)
{
    /* 切换模式时先停机，避免旧模式继续运动 */
    App_StopAll();

    App_Mode++;
    if (App_Mode >= APP_MODE_NUM)
    {
        App_Mode = APP_MODE_IDLE;
    }
}

static void App_StartCurrentMode(void)
{
    switch (App_Mode)
    {
        case APP_MODE_EDGE_FAST:
            App_StartEdgeFast();
            break;

        case APP_MODE_RECT_NORMAL:
            App_StartRectangleTracking(100);
            break;

        case APP_MODE_RECT_ANY:
            App_StartRectangleTracking(100);
            break;

        case APP_MODE_CIRCLE:
            /* 圆轨迹需要后续增加圆形目标点生成，当前先安全停机 */
            App_StopAll();
            break;

        case APP_MODE_DIGIT:
            /* 数字顺序运动需要后续扩展摄像头串口协议，当前先安全停机 */
            App_StopAll();
            break;

        default:
            App_StopAll();
            break;
    }
}

static void App_StartEdgeFast(void)
{
    App_PrepareTracking();

    /* 屏幕中心正方形边线测试点，后续可按实际标定结果微调 */
    Tracking_SetQuadrilateralSection(80);
    Tracking_SetQuadrilateral(60, 60, 180, 60, 180, 180, 60, 180);
    Tracking_EnableQuadrilateral(1);
    Tracking_Enable(1);
}

static void App_StartRectangleTracking(uint16_t Section)
{
    App_PrepareTracking();

    /* 重新等待摄像头串口发送的矩形胶带黑框坐标 */
    Tracking_ResetQuadrilateralLock();

    /* 设置每条边分段数，数值越大循迹越慢，越小越快 */
    Tracking_SetQuadrilateralSection(Section);

    Tracking_EnableQuadrilateral(1);
    Tracking_Enable(1);
}

static void App_ResetToCenter(void)
{
    App_PrepareTracking();

    /* 复位命令独立于当前模式，目标点固定为屏幕中心 */
    Tracking_SetTarget(120, 120);
    Tracking_EnableQuadrilateral(0);
    Tracking_Enable(1);
}

uint8_t App_GetMode(void)
{
    return (uint8_t)App_Mode;
}

void App_Run(void)
{
    uint8_t KeyNum;

    KeyNum = Key_GetNum();
    if (KeyNum == 1)
    {
        App_SwitchMode();
    }
    else if (KeyNum == 2)
    {
        App_StartCurrentMode();
    }
    else if (KeyNum == 3)
    {
        App_ResetToCenter();
    }
    else if (KeyNum == 4)
    {
        /* 紧急制动不再单独占用状态，直接回到空闲模式 */
        App_Mode = APP_MODE_IDLE;
        App_StopAll();
    }

    /* TIM4 每 10ms 置位一次标志，主循环里执行追踪任务 */
    if (Timer_GetFlag())
    {
        Tracking_Task();
    }
}
