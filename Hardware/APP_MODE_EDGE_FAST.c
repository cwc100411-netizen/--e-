#include "APP_MODE_EDGE_FAST.h"
#include "Tracking.h"
#include "Stepper.h"

/* 左上角已标定目标点：启动后先闭环追踪到这里，再开环跑矩形 */
#define EDGE_FAST_START_X               65
#define EDGE_FAST_START_Y               59

/* 开环矩形速度，单位 step/s；方向反了只改对应正负号 */
#define EDGE_FAST_RIGHT_SPEED_X         100
#define EDGE_FAST_RIGHT_SPEED_Y         0
#define EDGE_FAST_DOWN_SPEED_X          0
#define EDGE_FAST_DOWN_SPEED_Y         -100
#define EDGE_FAST_LEFT_SPEED_X         -100
#define EDGE_FAST_LEFT_SPEED_Y          0
#define EDGE_FAST_UP_SPEED_X            0
#define EDGE_FAST_UP_SPEED_Y            100

/* 四条边各自运行时间，单位 ms；现场只需要调这四个时间就能调边长 */
#define EDGE_FAST_RIGHT_TIME_MS         4800
#define EDGE_FAST_DOWN_TIME_MS          4400
#define EDGE_FAST_LEFT_TIME_MS          4900
#define EDGE_FAST_UP_TIME_MS            4450

#define EDGE_FAST_RIGHT_END_MS          EDGE_FAST_RIGHT_TIME_MS
#define EDGE_FAST_DOWN_END_MS           (EDGE_FAST_RIGHT_END_MS + EDGE_FAST_DOWN_TIME_MS)
#define EDGE_FAST_LEFT_END_MS           (EDGE_FAST_DOWN_END_MS + EDGE_FAST_LEFT_TIME_MS)
#define EDGE_FAST_UP_END_MS             (EDGE_FAST_LEFT_END_MS + EDGE_FAST_UP_TIME_MS)

#define EDGE_FAST_PHASE_TRACK_START     0
#define EDGE_FAST_PHASE_RUN_RECT        1

static uint8_t EdgeFast_Running = 0;
static uint8_t EdgeFast_Phase = EDGE_FAST_PHASE_TRACK_START;
static uint32_t EdgeFast_Ticks = 0;

void APP_MODE_EDGE_FAST_Start(void)
{
    /* 第一步：先闭环追踪到左上角已标定点 */
    Tracking_SetTarget(EDGE_FAST_START_X, EDGE_FAST_START_Y);
    Tracking_Enable(1);

    EdgeFast_Running = 1;
    EdgeFast_Phase = EDGE_FAST_PHASE_TRACK_START;
    EdgeFast_Ticks = 0;
}

void APP_MODE_EDGE_FAST_Stop(void)
{
    EdgeFast_Running = 0;
    EdgeFast_Phase = EDGE_FAST_PHASE_TRACK_START;
    EdgeFast_Ticks = 0;

    Tracking_Enable(0);
    Stepper_StopBoth();
}

void APP_MODE_EDGE_FAST_Task(void)
{
    uint32_t ElapsedMs;
    int16_t SpeedX;
    int16_t SpeedY;

    if (EdgeFast_Running == 0)
    {
        return;
    }

    EdgeFast_Ticks++;

    if (EdgeFast_Phase == EDGE_FAST_PHASE_TRACK_START)
    {
        Tracking_Task();

        /* 确认到达左上角后，关闭闭环，第二步开始开环跑完整矩形 */
        if (Tracking_IsTargetReachedNow() != 0)
        {
            Tracking_Enable(0);
            Stepper_StopBoth();
            EdgeFast_Phase = EDGE_FAST_PHASE_RUN_RECT;
            EdgeFast_Ticks = 0;
        }
        return;
    }

    ElapsedMs = EdgeFast_Ticks * 10UL;
    SpeedX = 0;
    SpeedY = 0;

    if (ElapsedMs < EDGE_FAST_RIGHT_END_MS)
    {
        SpeedX = EDGE_FAST_RIGHT_SPEED_X;
        SpeedY = EDGE_FAST_RIGHT_SPEED_Y;
    }
    else if (ElapsedMs < EDGE_FAST_DOWN_END_MS)
    {
        SpeedX = EDGE_FAST_DOWN_SPEED_X;
        SpeedY = EDGE_FAST_DOWN_SPEED_Y;
    }
    else if (ElapsedMs < EDGE_FAST_LEFT_END_MS)
    {
        SpeedX = EDGE_FAST_LEFT_SPEED_X;
        SpeedY = EDGE_FAST_LEFT_SPEED_Y;
    }
    else if (ElapsedMs < EDGE_FAST_UP_END_MS)
    {
        SpeedX = EDGE_FAST_UP_SPEED_X;
        SpeedY = EDGE_FAST_UP_SPEED_Y;
    }
    else
    {
        APP_MODE_EDGE_FAST_Stop();
        return;
    }

    /* 当前接线：电机2控制水平轴，电机1控制竖直轴 */
    Stepper_SetSpeed(STEPPER_MOTOR_2, SpeedX);
    Stepper_SetSpeed(STEPPER_MOTOR_1, SpeedY);
}

uint8_t APP_MODE_EDGE_FAST_IsRunning(void)
{
    return EdgeFast_Running;
}
