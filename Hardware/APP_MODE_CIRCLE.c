#include "APP_MODE_CIRCLE.h"
#include "Tracking.h"
#include "Stepper.h"

#define CIRCLE_IMAGE_SIZE             240
#define CIRCLE_IMAGE_MAX              (CIRCLE_IMAGE_SIZE - 1)
#define CIRCLE_POINT_NUM              360      /* 一圈目标点数量：60/90/120/180/360 可选 */
#define CIRCLE_HOLD_TICKS             100     /* AppRun 每 10ms 调用一次，100 次约 1 秒 */
#define CIRCLE_CENTER_X               129     /* 默认圆心 x 坐标 */
#define CIRCLE_CENTER_Y               123     /* 默认圆心 y 坐标 */
#define CIRCLE_TOP_X                  129     /* 默认圆心正上方圆上点 x 坐标 */
#define CIRCLE_TOP_Y                  59      /* 默认圆心正上方圆上点 y 坐标 */

/* 使用固定点旋转生成圆上点，避免浮点三角函数和大表。 */
#define CIRCLE_OFFSET_SCALE           256
#define CIRCLE_ROTATE_SCALE           8192

/* 根据一圈目标点数量选择每一步的旋转角度，保证 PointNum 个点刚好走完 360 度。 */
#if (CIRCLE_POINT_NUM == 60)
#define CIRCLE_COS_STEP               8147    /* cos(6 度) * 8192 */
#define CIRCLE_SIN_STEP               856     /* sin(6 度) * 8192 */
#elif (CIRCLE_POINT_NUM == 90)
#define CIRCLE_COS_STEP               8172    /* cos(4 度) * 8192 */
#define CIRCLE_SIN_STEP               571     /* sin(4 度) * 8192 */
#elif (CIRCLE_POINT_NUM == 120)
#define CIRCLE_COS_STEP               8181    /* cos(3 度) * 8192 */
#define CIRCLE_SIN_STEP               429     /* sin(3 度) * 8192 */
#elif (CIRCLE_POINT_NUM == 180)
#define CIRCLE_COS_STEP               8187    /* cos(2 度) * 8192 */
#define CIRCLE_SIN_STEP               286     /* sin(2 度) * 8192 */
#elif (CIRCLE_POINT_NUM == 360)
#define CIRCLE_COS_STEP               8191    /* cos(1 度) * 8192 */
#define CIRCLE_SIN_STEP               143     /* sin(1 度) * 8192 */
#else
#error CIRCLE_POINT_NUM only supports 60, 90, 120, 180, or 360
#endif

static uint8_t Circle_Running = 0;
static uint8_t Circle_StartReached = 0;
static uint8_t Circle_Drawing = 0;
static uint16_t Circle_HoldTick = 0;
static uint16_t Circle_PointIndex = 0;

static uint8_t Circle_CenterX = CIRCLE_CENTER_X;
static uint8_t Circle_CenterY = CIRCLE_CENTER_Y;
static int16_t Circle_StartOffsetX = CIRCLE_TOP_X - CIRCLE_CENTER_X;
static int16_t Circle_StartOffsetY = CIRCLE_TOP_Y - CIRCLE_CENTER_Y;
static int32_t Circle_OffsetX = (CIRCLE_TOP_X - CIRCLE_CENTER_X) * CIRCLE_OFFSET_SCALE;
static int32_t Circle_OffsetY = (CIRCLE_TOP_Y - CIRCLE_CENTER_Y) * CIRCLE_OFFSET_SCALE;

static void Circle_ResetRun(void);
static void Circle_LoadStartOffset(void);
static void Circle_SetTargetByOffset(void);
static void Circle_MoveToNextPoint(void);
static void Circle_StopAtEnd(void);
static int32_t Circle_DivRound(int32_t Value, int32_t Divisor);
static uint8_t Circle_ClampPixel(int16_t Value);

void APP_MODE_CIRCLE_Start(void)
{
    Circle_ResetRun();
    Circle_Running = 1;

    /* 第一步先让激光移动到圆心正上方的圆上点。 */
    Circle_LoadStartOffset();
    Circle_SetTargetByOffset();
    Tracking_Enable(1);
}

void APP_MODE_CIRCLE_Stop(void)
{
    Circle_Running = 0;
    Circle_ResetRun();
    Tracking_Enable(0);
    Stepper_StopBoth();
}

void APP_MODE_CIRCLE_Task(void)
{
    if (Circle_Running == 0)
    {
        return;
    }

    if (Circle_StartReached == 0)
    {
        Tracking_Task();
        if (Tracking_IsTargetReachedNow() != 0)
        {
            /* 到达起点后先停机，之后只用计数等待，不用阻塞延时。 */
            Circle_StartReached = 1;
            Circle_HoldTick = 0;
            Tracking_Enable(0);
            Stepper_StopBoth();
        }
        return;
    }

    if (Circle_Drawing == 0)
    {
        if (Circle_HoldTick < CIRCLE_HOLD_TICKS)
        {
            Circle_HoldTick++;
            Stepper_StopBoth();
            return;
        }

        /* 起点已经到达并等待完成，从第 1 个圆周点开始画。 */
        Circle_Drawing = 1;
        Circle_PointIndex = 0;
        Circle_LoadStartOffset();
        Circle_MoveToNextPoint();
        Tracking_Enable(1);
        return;
    }

    Tracking_Task();
    if (Tracking_IsTargetReachedNow() == 0)
    {
        return;
    }

    if (Circle_PointIndex >= CIRCLE_POINT_NUM)
    {
        Circle_StopAtEnd();
        return;
    }

    Circle_MoveToNextPoint();
}

uint8_t APP_MODE_CIRCLE_IsRunning(void)
{
    return Circle_Running;
}

void APP_MODE_CIRCLE_SetCircle(int16_t CenterX, int16_t CenterY, int16_t TopX, int16_t TopY)
{
    Circle_CenterX = Circle_ClampPixel(CenterX);
    Circle_CenterY = Circle_ClampPixel(CenterY);
    TopX = Circle_ClampPixel(TopX);
    TopY = Circle_ClampPixel(TopY);

    Circle_StartOffsetX = TopX - (int16_t)Circle_CenterX;
    Circle_StartOffsetY = TopY - (int16_t)Circle_CenterY;
    if ((Circle_StartOffsetX == 0) && (Circle_StartOffsetY == 0))
    {
        Circle_StartOffsetY = -1;
    }

    Circle_ResetRun();
    Circle_LoadStartOffset();
    Circle_SetTargetByOffset();
}

static void Circle_ResetRun(void)
{
    Circle_StartReached = 0;
    Circle_Drawing = 0;
    Circle_HoldTick = 0;
    Circle_PointIndex = 0;
}

static void Circle_LoadStartOffset(void)
{
    Circle_OffsetX = (int32_t)Circle_StartOffsetX * CIRCLE_OFFSET_SCALE;
    Circle_OffsetY = (int32_t)Circle_StartOffsetY * CIRCLE_OFFSET_SCALE;
}

static void Circle_SetTargetByOffset(void)
{
    int16_t TargetX;
    int16_t TargetY;

    TargetX = (int16_t)Circle_CenterX
            + (int16_t)Circle_DivRound(Circle_OffsetX, CIRCLE_OFFSET_SCALE);
    TargetY = (int16_t)Circle_CenterY
            + (int16_t)Circle_DivRound(Circle_OffsetY, CIRCLE_OFFSET_SCALE);
    Tracking_UpdateTarget(Circle_ClampPixel(TargetX), Circle_ClampPixel(TargetY));
}

static void Circle_MoveToNextPoint(void)
{
    int32_t NewOffsetX;
    int32_t NewOffsetY;

    Circle_PointIndex++;
    if (Circle_PointIndex >= CIRCLE_POINT_NUM)
    {
        /* 最后一个目标点强制回到起点，避免连续旋转带来的微小累计误差。 */
        Circle_LoadStartOffset();
        Circle_SetTargetByOffset();
        return;
    }

    NewOffsetX = Circle_OffsetX * CIRCLE_COS_STEP
               - Circle_OffsetY * CIRCLE_SIN_STEP;
    NewOffsetY = Circle_OffsetX * CIRCLE_SIN_STEP
               + Circle_OffsetY * CIRCLE_COS_STEP;
    Circle_OffsetX = Circle_DivRound(NewOffsetX, CIRCLE_ROTATE_SCALE);
    Circle_OffsetY = Circle_DivRound(NewOffsetY, CIRCLE_ROTATE_SCALE);
    Circle_SetTargetByOffset();
}

static void Circle_StopAtEnd(void)
{
    Circle_Running = 0;
    Circle_Drawing = 0;
    Tracking_Enable(0);
    Stepper_StopBoth();
}

static int32_t Circle_DivRound(int32_t Value, int32_t Divisor)
{
    if (Value >= 0)
    {
        return (Value + Divisor / 2) / Divisor;
    }
    return (Value - Divisor / 2) / Divisor;
}

static uint8_t Circle_ClampPixel(int16_t Value)
{
    if (Value < 0)
    {
        return 0;
    }
    if (Value > CIRCLE_IMAGE_MAX)
    {
        return CIRCLE_IMAGE_MAX;
    }
    return (uint8_t)Value;
}
