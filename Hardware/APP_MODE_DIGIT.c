#include "APP_MODE_DIGIT.h"
#include "Tracking.h"
#include "Serial.h"
#include "Stepper.h"

#define DIGIT_IMAGE_SIZE                240
#define DIGIT_TARGET_NUM                5       /* 数字目标数量，固定为 1~5 */
#define DIGIT_HOLD_TICKS                100     /* 到达每个数字后停留 100 个 10ms 周期，约 1000ms */

typedef struct
{
    uint8_t X;
    uint8_t Y;
} Digit_PointTypeDef;

static Digit_PointTypeDef Digit_Target[DIGIT_TARGET_NUM];
static uint8_t Digit_TargetValid = 0;
static uint8_t Digit_Running = 0;
static uint8_t Digit_TrackingActive = 0;
static uint8_t Digit_Index = 0;
static uint8_t Digit_Finished = 0;
static uint16_t Digit_HoldCount = 0;

static void Digit_ReadTargetPacket(void);
static void Digit_StartCurrentTarget(void);
static void Digit_StartHold(void);
static void Digit_ProcessHold(void);
static uint8_t Digit_IsPointValid(Digit_PointTypeDef Point);

void APP_MODE_DIGIT_Start(void)
{
    Digit_Running = 1;
    Digit_TrackingActive = 0;
    Digit_Index = 0;
    Digit_Finished = 0;
    Digit_HoldCount = 0;

    if (Digit_TargetValid != 0)
    {
        Digit_StartCurrentTarget();
    }
}

void APP_MODE_DIGIT_Stop(void)
{
    Digit_Running = 0;
    Digit_TrackingActive = 0;
    Digit_HoldCount = 0;

    Tracking_Enable(0);
    Stepper_StopBoth();
}

void APP_MODE_DIGIT_Task(void)
{
    Digit_ReadTargetPacket();

    if (Digit_Running == 0)
    {
        return;
    }

    if (Digit_HoldCount > 0)
    {
        Digit_ProcessHold();
        return;
    }

    if ((Digit_TargetValid == 0) || (Digit_Finished != 0))
    {
        Tracking_Enable(0);
        Stepper_StopBoth();
        Digit_TrackingActive = 0;
        return;
    }

    if (Digit_TrackingActive == 0)
    {
        Digit_StartCurrentTarget();
    }

    Tracking_Task();
    if (Tracking_IsTargetReachedNow() != 0)
    {
        Digit_StartHold();
    }
}

uint8_t APP_MODE_DIGIT_IsRunning(void)
{
    return Digit_Running;
}

static void Digit_ReadTargetPacket(void)
{
    uint8_t i;
    Digit_PointTypeDef Temp[DIGIT_TARGET_NUM];

    if (Serial_PeekRxFlag() == 0)
    {
        return;
    }

    if (Serial_RxType != SERIAL_PACKET_TYPE_DIGIT_TARGETS)
    {
        return;
    }

    if (Serial_RxLength < SERIAL_RX_PACKET_LENGTH)
    {
        Serial_ClearRxFlag();
        return;
    }

    /* FD 包的 10 字节依次表示数字 1、2、3、4、5 的中心点 x/y */
    for (i = 0; i < DIGIT_TARGET_NUM; i++)
    {
        Temp[i].X = Serial_RxPacket[i * 2];
        Temp[i].Y = Serial_RxPacket[i * 2 + 1];
        if (Digit_IsPointValid(Temp[i]) == 0)
        {
            Serial_ClearRxFlag();
            return;
        }
    }

    for (i = 0; i < DIGIT_TARGET_NUM; i++)
    {
        Digit_Target[i] = Temp[i];
    }
    Digit_TargetValid = 1;
    Serial_ClearRxFlag();

    /* 运行中收到新坐标时，只更新当前目标，不重置数字进度 */
    if ((Digit_Running != 0) && (Digit_Finished == 0) && (Digit_Index < DIGIT_TARGET_NUM))
    {
        Tracking_UpdateTarget(Digit_Target[Digit_Index].X, Digit_Target[Digit_Index].Y);
    }
}

static void Digit_StartCurrentTarget(void)
{
    if (Digit_Index >= DIGIT_TARGET_NUM)
    {
        Digit_Index = DIGIT_TARGET_NUM - 1;
    }

    Tracking_SetTarget(Digit_Target[Digit_Index].X, Digit_Target[Digit_Index].Y);
    Tracking_Enable(1);
    Digit_TrackingActive = 1;
}

static void Digit_StartHold(void)
{
    Tracking_Enable(0);
    Stepper_StopBoth();
    Digit_TrackingActive = 0;
    Digit_HoldCount = DIGIT_HOLD_TICKS;
}

static void Digit_ProcessHold(void)
{
    Tracking_Enable(0);
    Stepper_StopBoth();
    Digit_TrackingActive = 0;

    if (Digit_HoldCount > 0)
    {
        Digit_HoldCount--;
    }
    if (Digit_HoldCount > 0)
    {
        return;
    }

    if (Digit_Index + 1 < DIGIT_TARGET_NUM)
    {
        Digit_Index++;
        Digit_StartCurrentTarget();
    }
    else
    {
        Digit_Finished = 1;
        APP_MODE_DIGIT_Stop();
    }
}

static uint8_t Digit_IsPointValid(Digit_PointTypeDef Point)
{
    if ((Point.X < DIGIT_IMAGE_SIZE) && (Point.Y < DIGIT_IMAGE_SIZE))
    {
        return 1;
    }
    return 0;
}
