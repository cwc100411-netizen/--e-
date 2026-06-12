#include "APP_MODE_CIRCLE.h"
#include "Tracking.h"
#include "Stepper.h"

#define CIRCLE_IMAGE_SIZE             240
#define CIRCLE_IMAGE_MAX              (CIRCLE_IMAGE_SIZE - 1)
#define CIRCLE_POINT_NUM              180     /* 圆形一圈 180 个插值点，每个点相差 2 度 */
#define CIRCLE_CENTER_X               133     /* 默认圆心 x 坐标 */
#define CIRCLE_CENTER_Y               130     /* 默认圆心 y 坐标 */
#define CIRCLE_START_X                131     /* 默认圆形起点 x 坐标 */
#define CIRCLE_START_Y                56      /* 默认圆形起点 y 坐标 */
#define CIRCLE_ADVANCE_TICKS          2       /* 目标每 2 个 10ms 周期前进一次，约 20ms */

/* 单位圆查表，数值放大 1000 倍。第 0 个点对应圆最上方，之后顺时针前进。 */
static const int16_t Circle_Table[CIRCLE_POINT_NUM][2] =
{
    {    0, -1000},
    {   35,  -999},
    {   70,  -998},
    {  105,  -995},
    {  139,  -990},
    {  174,  -985},
    {  208,  -978},
    {  242,  -970},
    {  276,  -961},
    {  309,  -951},
    {  342,  -940},
    {  375,  -927},
    {  407,  -914},
    {  438,  -899},
    {  469,  -883},
    {  500,  -866},
    {  530,  -848},
    {  559,  -829},
    {  588,  -809},
    {  616,  -788},
    {  643,  -766},
    {  669,  -743},
    {  695,  -719},
    {  719,  -695},
    {  743,  -669},
    {  766,  -643},
    {  788,  -616},
    {  809,  -588},
    {  829,  -559},
    {  848,  -530},
    {  866,  -500},
    {  883,  -469},
    {  899,  -438},
    {  914,  -407},
    {  927,  -375},
    {  940,  -342},
    {  951,  -309},
    {  961,  -276},
    {  970,  -242},
    {  978,  -208},
    {  985,  -174},
    {  990,  -139},
    {  995,  -105},
    {  998,   -70},
    {  999,   -35},
    { 1000,     0},
    {  999,    35},
    {  998,    70},
    {  995,   105},
    {  990,   139},
    {  985,   174},
    {  978,   208},
    {  970,   242},
    {  961,   276},
    {  951,   309},
    {  940,   342},
    {  927,   375},
    {  914,   407},
    {  899,   438},
    {  883,   469},
    {  866,   500},
    {  848,   530},
    {  829,   559},
    {  809,   588},
    {  788,   616},
    {  766,   643},
    {  743,   669},
    {  719,   695},
    {  695,   719},
    {  669,   743},
    {  643,   766},
    {  616,   788},
    {  588,   809},
    {  559,   829},
    {  530,   848},
    {  500,   866},
    {  469,   883},
    {  438,   899},
    {  407,   914},
    {  375,   927},
    {  342,   940},
    {  309,   951},
    {  276,   961},
    {  242,   970},
    {  208,   978},
    {  174,   985},
    {  139,   990},
    {  105,   995},
    {   70,   998},
    {   35,   999},
    {    0,  1000},
    {  -35,   999},
    {  -70,   998},
    { -105,   995},
    { -139,   990},
    { -174,   985},
    { -208,   978},
    { -242,   970},
    { -276,   961},
    { -309,   951},
    { -342,   940},
    { -375,   927},
    { -407,   914},
    { -438,   899},
    { -469,   883},
    { -500,   866},
    { -530,   848},
    { -559,   829},
    { -588,   809},
    { -616,   788},
    { -643,   766},
    { -669,   743},
    { -695,   719},
    { -719,   695},
    { -743,   669},
    { -766,   643},
    { -788,   616},
    { -809,   588},
    { -829,   559},
    { -848,   530},
    { -866,   500},
    { -883,   469},
    { -899,   438},
    { -914,   407},
    { -927,   375},
    { -940,   342},
    { -951,   309},
    { -961,   276},
    { -970,   242},
    { -978,   208},
    { -985,   174},
    { -990,   139},
    { -995,   105},
    { -998,    70},
    { -999,    35},
    {-1000,     0},
    { -999,   -35},
    { -998,   -70},
    { -995,  -105},
    { -990,  -139},
    { -985,  -174},
    { -978,  -208},
    { -970,  -242},
    { -961,  -276},
    { -951,  -309},
    { -940,  -342},
    { -927,  -375},
    { -914,  -407},
    { -899,  -438},
    { -883,  -469},
    { -866,  -500},
    { -848,  -530},
    { -829,  -559},
    { -809,  -588},
    { -788,  -616},
    { -766,  -643},
    { -743,  -669},
    { -719,  -695},
    { -695,  -719},
    { -669,  -743},
    { -643,  -766},
    { -616,  -788},
    { -588,  -809},
    { -559,  -829},
    { -530,  -848},
    { -500,  -866},
    { -469,  -883},
    { -438,  -899},
    { -407,  -914},
    { -375,  -927},
    { -342,  -940},
    { -309,  -951},
    { -276,  -961},
    { -242,  -970},
    { -208,  -978},
    { -174,  -985},
    { -139,  -990},
    { -105,  -995},
    {  -70,  -998},
    {  -35,  -999},
};

static uint8_t Circle_Running = 0;
static uint16_t Circle_Index = 0;
static uint16_t Circle_Tick = 0;
static uint8_t Circle_CenterX = CIRCLE_CENTER_X;
static uint8_t Circle_CenterY = CIRCLE_CENTER_Y;
static int16_t Circle_StartOffsetX = CIRCLE_START_X - CIRCLE_CENTER_X;
static int16_t Circle_StartOffsetY = CIRCLE_START_Y - CIRCLE_CENTER_Y;

static void Circle_UpdateTarget(void);
static void Circle_MoveTarget(void);
static uint8_t Circle_ClampPixel(int16_t Value);

void APP_MODE_CIRCLE_Start(void)
{
    Circle_Running = 1;
    Circle_Index = 0;
    Circle_Tick = 0;

    Circle_UpdateTarget();
    Tracking_Enable(1);
}

void APP_MODE_CIRCLE_Stop(void)
{
    Circle_Running = 0;
    Circle_Index = 0;
    Circle_Tick = 0;

    Tracking_Enable(0);
    Stepper_StopBoth();
}

void APP_MODE_CIRCLE_Task(void)
{
    if (Circle_Running == 0)
    {
        return;
    }

    Circle_UpdateTarget();
    Tracking_Task();

    Circle_Tick++;
    if (Circle_Tick >= CIRCLE_ADVANCE_TICKS)
    {
        Circle_MoveTarget();
    }
}

uint8_t APP_MODE_CIRCLE_IsRunning(void)
{
    return Circle_Running;
}

void APP_MODE_CIRCLE_SetCircle(int16_t CenterX, int16_t CenterY, int16_t StartX, int16_t StartY)
{
    int16_t OffsetX;
    int16_t OffsetY;

    Circle_CenterX = Circle_ClampPixel(CenterX);
    Circle_CenterY = Circle_ClampPixel(CenterY);

    /* 用“圆心 + 圆上一点”确定圆，首个目标点就是传入的 StartX、StartY。 */
    OffsetX = (int16_t)Circle_ClampPixel(StartX) - (int16_t)Circle_CenterX;
    OffsetY = (int16_t)Circle_ClampPixel(StartY) - (int16_t)Circle_CenterY;
    if ((OffsetX == 0) && (OffsetY == 0))
    {
        OffsetY = -1;
    }

    Circle_StartOffsetX = OffsetX;
    Circle_StartOffsetY = OffsetY;
    Circle_Index = 0;
    Circle_Tick = 0;
    Circle_UpdateTarget();
}

static void Circle_UpdateTarget(void)
{
    int32_t OffsetX;
    int32_t OffsetY;
    int32_t TargetX;
    int32_t TargetY;
    int32_t SinValue;
    int32_t NegCosValue;

    if (Circle_Index >= CIRCLE_POINT_NUM)
    {
        Circle_Index = 0;
    }

    /* 查表值放大了 1000 倍；把起点相对圆心的偏移按顺时针方向旋转。 */
    SinValue = Circle_Table[Circle_Index][0];
    NegCosValue = Circle_Table[Circle_Index][1];
    OffsetX = -((int32_t)Circle_StartOffsetX * NegCosValue)
            - ((int32_t)Circle_StartOffsetY * SinValue);
    OffsetY =  ((int32_t)Circle_StartOffsetX * SinValue)
            - ((int32_t)Circle_StartOffsetY * NegCosValue);

    if (OffsetX >= 0)
    {
        OffsetX = (OffsetX + 500) / 1000;
    }
    else
    {
        OffsetX = (OffsetX - 500) / 1000;
    }

    if (OffsetY >= 0)
    {
        OffsetY = (OffsetY + 500) / 1000;
    }
    else
    {
        OffsetY = (OffsetY - 500) / 1000;
    }

    TargetX = (int32_t)Circle_CenterX + OffsetX;
    TargetY = (int32_t)Circle_CenterY + OffsetY;
    Tracking_UpdateTarget((int16_t)TargetX, (int16_t)TargetY);
}

static void Circle_MoveTarget(void)
{
    Circle_Tick = 0;
    Circle_Index++;
    if (Circle_Index >= CIRCLE_POINT_NUM)
    {
        Circle_Index = 0;
    }
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
