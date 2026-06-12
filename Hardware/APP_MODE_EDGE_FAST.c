#include "APP_MODE_EDGE_FAST.h"
#include "Tracking.h"
#include "Stepper.h"

/* 高速边线模式参数，来自原边线工程，后续现场调边长和方向时优先改这里 */
#define EDGE_FAST_TARGET_X              76      /* 启动后先追踪到这个目标 x 坐标 */
#define EDGE_FAST_TARGET_Y              72      /* 启动后先追踪到这个目标 y 坐标 */
#define EDGE_FAST_CHECK_X               73      /* 判断已到起点时使用的激光 x 坐标 */
#define EDGE_FAST_CHECK_Y               65      /* 判断已到起点时使用的激光 y 坐标 */
#define EDGE_FAST_CHECK_RANGE           3       /* 起点允许误差，小于该值认为到位 */
#define EDGE_FAST_MIN_TRACK_TICKS       50      /* 至少闭环追踪 50 个 10ms 周期后才允许切到跑边 */
#define EDGE_FAST_TRACK_TIMEOUT_TICKS   500     /* 追踪起点超时保护，约 5 秒 */

/* 正方形补偿：调这 8 个速度值可以修正四条边的歪斜 */
#define EDGE_FAST_RIGHT_VX              100
#define EDGE_FAST_RIGHT_VY              0       /* 右边往下偏就改负，往上偏改正 */
#define EDGE_FAST_UP_VX                 0       /* 上边往右偏改正，往左偏改负 */
#define EDGE_FAST_UP_VY                -100
#define EDGE_FAST_LEFT_VX              -100
#define EDGE_FAST_LEFT_VY               0
#define EDGE_FAST_DOWN_VX               0
#define EDGE_FAST_DOWN_VY               100

/* 每条边的运行结束时间，单位 ms，按原边线工程参数保留 */
#define EDGE_FAST_RIGHT_END_MS          4800
#define EDGE_FAST_UP_END_MS             9200
#define EDGE_FAST_LEFT_END_MS           14100
#define EDGE_FAST_DOWN_END_MS           18550

#define EDGE_FAST_PHASE_TRACK_START     0
#define EDGE_FAST_PHASE_RUN_SQUARE      1

static uint8_t EdgeFast_Running = 0;
static uint8_t EdgeFast_Phase = EDGE_FAST_PHASE_TRACK_START;
static uint32_t EdgeFast_Ticks = 0;

static uint8_t EdgeFast_IsStartReached(void);
static int16_t EdgeFast_AbsInt16(int16_t Value);

void APP_MODE_EDGE_FAST_Start(void)
{
    /* 原边线工程的做法：先闭环追到固定起点，再切换到定时跑四条边 */
    Tracking_SetTarget(EDGE_FAST_TARGET_X, EDGE_FAST_TARGET_Y);
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

        /* 先追踪一小段时间；到达起点或超时后，关闭闭环并开始定时跑边 */
        if ((EdgeFast_Ticks > EDGE_FAST_MIN_TRACK_TICKS) &&
            ((EdgeFast_IsStartReached() != 0) ||
             (EdgeFast_Ticks > EDGE_FAST_TRACK_TIMEOUT_TICKS)))
        {
            Tracking_Enable(0);
            Stepper_StopBoth();
            EdgeFast_Phase = EDGE_FAST_PHASE_RUN_SQUARE;
            EdgeFast_Ticks = 0;
        }
        return;
    }

    ElapsedMs = EdgeFast_Ticks * 10UL;
    SpeedX = 0;
    SpeedY = 0;

    if (ElapsedMs < EDGE_FAST_RIGHT_END_MS)
    {
        SpeedX = EDGE_FAST_RIGHT_VX;
        SpeedY = EDGE_FAST_RIGHT_VY;
    }
    else if (ElapsedMs < EDGE_FAST_UP_END_MS)
    {
        SpeedX = EDGE_FAST_UP_VX;
        SpeedY = EDGE_FAST_UP_VY;
    }
    else if (ElapsedMs < EDGE_FAST_LEFT_END_MS)
    {
        SpeedX = EDGE_FAST_LEFT_VX;
        SpeedY = EDGE_FAST_LEFT_VY;
    }
    else if (ElapsedMs < EDGE_FAST_DOWN_END_MS)
    {
        SpeedX = EDGE_FAST_DOWN_VX;
        SpeedY = EDGE_FAST_DOWN_VY;
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

static uint8_t EdgeFast_IsStartReached(void)
{
    uint8_t LaserX;
    uint8_t LaserY;

    if (Tracking_GetLaserPoint(&LaserX, &LaserY) == 0)
    {
        return 0;
    }

    if ((EdgeFast_AbsInt16((int16_t)LaserX - EDGE_FAST_CHECK_X) < EDGE_FAST_CHECK_RANGE) &&
        (EdgeFast_AbsInt16((int16_t)LaserY - EDGE_FAST_CHECK_Y) < EDGE_FAST_CHECK_RANGE))
    {
        return 1;
    }

    return 0;
}

static int16_t EdgeFast_AbsInt16(int16_t Value)
{
    if (Value < 0)
    {
        return -Value;
    }
    return Value;
}
