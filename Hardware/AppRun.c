#include "AppRun.h"
#include "Stepper.h"
#include "Key.h"
#include "Timer.h"
#include "Delay.h"
#include "Tracking.h"
#include "APP_MODE_EDGE_FAST.h"
#include "APP_MODE_CIRCLE.h"
#include "APP_MODE_DIGIT.h"

typedef enum
{
    APP_MODE_IDLE = 0,          /* 模式0：空闲 */
    APP_MODE_EDGE_FAST = 1,     /* 模式1：高速边线运动 */
    APP_MODE_RECT_NORMAL = 2,   /* 模式2：正着矩形胶带黑框循迹 */
    APP_MODE_RECT_ANY = 3,      /* 模式3：任意角度矩形胶带黑框循迹 */
    APP_MODE_CIRCLE = 4,        /* 模式4：边线内切圆运动 */
    APP_MODE_DIGIT = 5,         /* 模式5：识别数字顺序运动 */
    APP_MODE_NUM
} App_ModeTypeDef;

/* 默认模式只改这里，注意不要设置为 APP_MODE_NUM */
#define APP_DEFAULT_MODE    APP_MODE_IDLE	

static App_ModeTypeDef App_Mode = APP_DEFAULT_MODE;

static void App_StopAll(void);
static void App_PrepareTracking(void);
static void App_SwitchMode(void);
static void App_StartCurrentMode(void);
static void App_StartEdgeFast(void);
static void App_StartCircle(void);
static void App_StartDigit(void);
static void App_StartRectangleTracking(uint16_t Section);
static void App_ResetToCenter(void);

static void App_StopAll(void)
{
    APP_MODE_EDGE_FAST_Stop();
    APP_MODE_CIRCLE_Stop();
    APP_MODE_DIGIT_Stop();
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
            App_StartRectangleTracking(20);
            break;

        case APP_MODE_RECT_ANY:
            App_StartRectangleTracking(40);
            break;

        case APP_MODE_CIRCLE:
            App_StartCircle();
            break;

        case APP_MODE_DIGIT:
            App_StartDigit();
            break;

        default:
            App_StopAll();
            break;
    }
}

static void App_StartEdgeFast(void)
{
    App_PrepareTracking();
    APP_MODE_EDGE_FAST_Start();
}

static void App_StartCircle(void)
{
    App_PrepareTracking();
    APP_MODE_CIRCLE_Start();
}

static void App_StartDigit(void)
{
    App_PrepareTracking();
    APP_MODE_DIGIT_Start();
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

    /* TIM4 每 10ms 置位一次标志，主循环里执行当前模式任务 */
    if (Timer_GetFlag())
    {
        if ((App_Mode == APP_MODE_EDGE_FAST) && (APP_MODE_EDGE_FAST_IsRunning() != 0))
        {
            APP_MODE_EDGE_FAST_Task();
            return;
        }
        if ((App_Mode == APP_MODE_CIRCLE) && (APP_MODE_CIRCLE_IsRunning() != 0))
        {
            APP_MODE_CIRCLE_Task();
            return;
        }
        if ((App_Mode == APP_MODE_DIGIT) && (APP_MODE_DIGIT_IsRunning() != 0))
        {
            APP_MODE_DIGIT_Task();
            return;
        }

        Tracking_Task();
        if (((App_Mode == APP_MODE_EDGE_FAST) ||
             (App_Mode == APP_MODE_RECT_NORMAL) ||
             (App_Mode == APP_MODE_RECT_ANY)) &&
            (Tracking_IsQuadrilateralFinished() != 0))
        {
            /* 四边形循迹完整跑完一圈后，回到空闲模式并停机 */
            App_Mode = APP_MODE_IDLE;
            App_StopAll();
        }
    }
}
