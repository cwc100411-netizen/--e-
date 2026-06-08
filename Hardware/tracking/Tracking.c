#include "Tracking.h"
#include "Serial.h"
#include "Stepper.h"
#include "pid.h"

/* 常用调试参数：后续调循迹速度和稳定性时优先改这里 */
#define TRACKING_IMAGE_SIZE             240
#define TRACKING_CENTER                 120
#define TRACKING_DEAD_ZONE_PIXEL        1
#define TRACKING_MIN_SPEED              60
#define TRACKING_MAX_SPEED              500
#define TRACKING_LOST_LIMIT             20
#define TRACKING_QUAD_DEFAULT_SECTION   200
#define TRACKING_QUAD_START_HOLD        200
#define TRACKING_QUAD_LOCK_COUNT        5
#define TRACKING_MIN_QUAD_AREA          100

/* PID 参数：先只用 P，抖动明显时再小幅加 D */
#define TRACKING_PID_KP                 3.0f
#define TRACKING_PID_KI                 0.0f
#define TRACKING_PID_KD                 0.0f

/* 如果实际电机方向相反，把对应值改为 -1 */
#define TRACKING_MOTOR_X_DIR_SIGN       1
#define TRACKING_MOTOR_Y_DIR_SIGN       1

typedef struct
{
	uint8_t X;
	uint8_t Y;
} Tracking_PointTypeDef;

enum
{
	TRACKING_QUAD_POINT_NUM = 4,
	TRACKING_IMAGE_MAX = TRACKING_IMAGE_SIZE - 1
};

static PID_TypeDef Tracking_PidX;
static PID_TypeDef Tracking_PidY;

static Tracking_PointTypeDef Tracking_Target;
static Tracking_PointTypeDef Tracking_Laser;
static Tracking_PointTypeDef Tracking_Quad[TRACKING_QUAD_POINT_NUM];

static uint8_t Tracking_LaserValid;
static uint8_t Tracking_EnableFlag;
static uint8_t Tracking_QuadEnableFlag;
static uint8_t Tracking_QuadLocked;
static uint8_t Tracking_QuadLockCount;
static uint8_t Tracking_QuadEdge;
static uint16_t Tracking_NoPacketCount;
static uint16_t Tracking_QuadSection;
static uint16_t Tracking_QuadStep;
static uint16_t Tracking_QuadHoldCount;
static int16_t Tracking_LastSpeedX;
static int16_t Tracking_LastSpeedY;

static void Tracking_StopAndReset(void);
static void Tracking_SetMotorSpeed(int16_t SpeedX, int16_t SpeedY);
static void Tracking_ReadPacket(Tracking_PointTypeDef *Laser, Tracking_PointTypeDef Quad[]);
static void Tracking_ResetQuadProgress(uint16_t HoldCount);
static void Tracking_SetQuadPoints(Tracking_PointTypeDef Quad[]);
static void Tracking_UpdateQuadTarget(void);
static void Tracking_MoveQuadTarget(void);
static uint8_t Tracking_ClampPixel(int16_t Value);
static uint8_t Tracking_IsPointValid(Tracking_PointTypeDef Point);
static uint8_t Tracking_IsQuadValid(Tracking_PointTypeDef Quad[]);
static int16_t Tracking_CalcAxisSpeed(PID_TypeDef *Pid, int16_t Error);

/**
  * 函    数：初始化追踪模块
  * 参    数：无
  * 返 回 值：无
  */
void Tracking_Init(void)
{
	Tracking_Target.X = TRACKING_CENTER;
	Tracking_Target.Y = TRACKING_CENTER;
	Tracking_Laser.X = 0;
	Tracking_Laser.Y = 0;
	Tracking_LaserValid = 0;
	Tracking_EnableFlag = 0;
	Tracking_QuadEnableFlag = 0;
	Tracking_QuadLocked = 0;
	Tracking_QuadLockCount = 0;
	Tracking_QuadEdge = 0;
	Tracking_NoPacketCount = 0;
	Tracking_QuadSection = TRACKING_QUAD_DEFAULT_SECTION;
	Tracking_QuadStep = 0;
	Tracking_QuadHoldCount = 0;
	Tracking_LastSpeedX = 0;
	Tracking_LastSpeedY = 0;

	/* 默认矩形只用于手动测试，摄像头黑框模式会被串口坐标覆盖 */
	Tracking_Quad[0].X = 60;
	Tracking_Quad[0].Y = 60;
	Tracking_Quad[1].X = 180;
	Tracking_Quad[1].Y = 60;
	Tracking_Quad[2].X = 180;
	Tracking_Quad[2].Y = 180;
	Tracking_Quad[3].X = 60;
	Tracking_Quad[3].Y = 180;

	PID_Config(&Tracking_PidX, TRACKING_PID_KP, TRACKING_PID_KI, TRACKING_PID_KD);
	PID_Config(&Tracking_PidY, TRACKING_PID_KP, TRACKING_PID_KI, TRACKING_PID_KD);

	Stepper_StopBoth();
}

/**
  * 函    数：追踪任务函数
  * 参    数：无
  * 返 回 值：无
  * 说    明：主循环周期调用，读取摄像头坐标并更新两个轴电机速度
  */
void Tracking_Task(void)
{
	Tracking_PointTypeDef RxLaser;
	Tracking_PointTypeDef RxQuad[TRACKING_QUAD_POINT_NUM];
	int16_t ErrorX;
	int16_t ErrorY;
	int16_t SpeedX;
	int16_t SpeedY;

	if (Tracking_EnableFlag == 0)
	{
		Tracking_StopAndReset();
		return;
	}

	if (Serial_GetRxFlag() == 0)
	{
		if (Tracking_NoPacketCount < TRACKING_LOST_LIMIT)
		{
			Tracking_NoPacketCount++;
		}
		else
		{
			Tracking_LaserValid = 0;
			Tracking_StopAndReset();
		}
		return;
	}

	/* 数据包格式：激光 x,y + 四边形四个顶点 x,y，共 10 字节 */
	Tracking_ReadPacket(&RxLaser, RxQuad);
	if (Tracking_IsPointValid(RxLaser) == 0)
	{
		Tracking_LaserValid = 0;
		Tracking_NoPacketCount = 0;
		Tracking_StopAndReset();
		return;
	}

	if ((Tracking_QuadEnableFlag != 0) && (Tracking_QuadLocked == 0))
	{
		if (Tracking_IsQuadValid(RxQuad) == 0)
		{
			Tracking_LaserValid = 0;
			Tracking_NoPacketCount = 0;
			Tracking_QuadLockCount = 0;
			Tracking_StopAndReset();
			return;
		}

		/* 连续收到几帧有效黑框后再锁定，减少偶发误检 */
		Tracking_SetQuadPoints(RxQuad);
		if (Tracking_QuadLockCount < TRACKING_QUAD_LOCK_COUNT)
		{
			Tracking_QuadLockCount++;
		}
		if (Tracking_QuadLockCount >= TRACKING_QUAD_LOCK_COUNT)
		{
			Tracking_QuadLocked = 1;
			Tracking_ResetQuadProgress(TRACKING_QUAD_START_HOLD);
			PID_Reset(&Tracking_PidX);
			PID_Reset(&Tracking_PidY);
		}
	}

	Tracking_Laser = RxLaser;
	Tracking_LaserValid = 1;
	Tracking_NoPacketCount = 0;

	if (Tracking_QuadEnableFlag != 0)
	{
		Tracking_UpdateQuadTarget();
	}

	/* 图像 x 向右为正；图像 y 向下为正，所以竖直误差反向 */
	ErrorX = (int16_t)Tracking_Target.X - (int16_t)Tracking_Laser.X;
	ErrorY = (int16_t)Tracking_Laser.Y - (int16_t)Tracking_Target.Y;
	SpeedX = Tracking_CalcAxisSpeed(&Tracking_PidX, ErrorX);
	SpeedY = Tracking_CalcAxisSpeed(&Tracking_PidY, ErrorY);

	if (TRACKING_MOTOR_X_DIR_SIGN < 0)
	{
		SpeedX = -SpeedX;
	}
	if (TRACKING_MOTOR_Y_DIR_SIGN < 0)
	{
		SpeedY = -SpeedY;
	}

	Tracking_SetMotorSpeed(SpeedX, SpeedY);
	Tracking_MoveQuadTarget();
}

/**
  * 函    数：开启或关闭追踪
  * 参    数：Enable 1 开启，0 关闭
  * 返 回 值：无
  */
void Tracking_Enable(uint8_t Enable)
{
	if (Enable != 0)
	{
		Tracking_EnableFlag = 1;
		Tracking_NoPacketCount = 0;
		PID_Reset(&Tracking_PidX);
		PID_Reset(&Tracking_PidY);
	}
	else
	{
		Tracking_EnableFlag = 0;
		Tracking_StopAndReset();
	}
}

/**
  * 函    数：开启或关闭四边形循迹
  * 参    数：Enable 1 开启，0 关闭
  * 返 回 值：无
  */
void Tracking_EnableQuadrilateral(uint8_t Enable)
{
	if (Enable != 0)
	{
		Tracking_QuadEnableFlag = 1;
		Tracking_ResetQuadProgress(TRACKING_QUAD_START_HOLD);
		Tracking_UpdateQuadTarget();
	}
	else
	{
		Tracking_QuadEnableFlag = 0;
	}

	Tracking_StopAndReset();
}

/**
  * 函    数：重新等待摄像头发送有效黑框坐标
  * 参    数：无
  * 返 回 值：无
  */
void Tracking_ResetQuadrilateralLock(void)
{
	Tracking_QuadLocked = 0;
	Tracking_QuadLockCount = 0;
	Tracking_LaserValid = 0;
	Tracking_ResetQuadProgress(0);
	Tracking_StopAndReset();
}

/**
  * 函    数：设置四边形每条边的分段数
  * 参    数：Section 数值越大循迹越慢
  * 返 回 值：无
  */
void Tracking_SetQuadrilateralSection(uint16_t Section)
{
	if (Section == 0)
	{
		Section = 1;
	}

	Tracking_QuadSection = Section;
	Tracking_ResetQuadProgress(TRACKING_QUAD_START_HOLD);
	Tracking_UpdateQuadTarget();
	Tracking_StopAndReset();
}

/**
  * 函    数：手动设置四边形循迹顶点
  * 参    数：X1~Y4 四个顶点坐标
  * 返 回 值：无
  */
void Tracking_SetQuadrilateral(int16_t X1, int16_t Y1,
                               int16_t X2, int16_t Y2,
                               int16_t X3, int16_t Y3,
                               int16_t X4, int16_t Y4)
{
	Tracking_PointTypeDef Quad[TRACKING_QUAD_POINT_NUM];

	Quad[0].X = Tracking_ClampPixel(X1);
	Quad[0].Y = Tracking_ClampPixel(Y1);
	Quad[1].X = Tracking_ClampPixel(X2);
	Quad[1].Y = Tracking_ClampPixel(Y2);
	Quad[2].X = Tracking_ClampPixel(X3);
	Quad[2].Y = Tracking_ClampPixel(Y3);
	Quad[3].X = Tracking_ClampPixel(X4);
	Quad[3].Y = Tracking_ClampPixel(Y4);

	Tracking_SetQuadPoints(Quad);
	Tracking_QuadLocked = 1;
	Tracking_QuadLockCount = TRACKING_QUAD_LOCK_COUNT;
	Tracking_ResetQuadProgress(TRACKING_QUAD_START_HOLD);
	Tracking_UpdateQuadTarget();
	Tracking_StopAndReset();
}

/**
  * 函    数：设置固定追踪目标
  * 参    数：TargetX 目标 x 坐标
  * 参    数：TargetY 目标 y 坐标
  * 返 回 值：无
  */
void Tracking_SetTarget(int16_t TargetX, int16_t TargetY)
{
	Tracking_Target.X = Tracking_ClampPixel(TargetX);
	Tracking_Target.Y = Tracking_ClampPixel(TargetY);
	Tracking_StopAndReset();
}

/**
  * 函    数：获取最近一次有效激光点坐标
  * 参    数：X 保存 x 坐标，可传入 0
  * 参    数：Y 保存 y 坐标，可传入 0
  * 返 回 值：1 有效，0 无效
  */
uint8_t Tracking_GetLaserPoint(uint8_t *X, uint8_t *Y)
{
	if (Tracking_LaserValid == 0)
	{
		return 0;
	}

	if (X != 0)
	{
		*X = Tracking_Laser.X;
	}
	if (Y != 0)
	{
		*Y = Tracking_Laser.Y;
	}

	return 1;
}

/**
  * 函    数：获取当前目标坐标
  * 参    数：X 保存 x 坐标，可传入 0
  * 参    数：Y 保存 y 坐标，可传入 0
  * 返 回 值：无
  */
void Tracking_GetTarget(uint8_t *X, uint8_t *Y)
{
	if (X != 0)
	{
		*X = Tracking_Target.X;
	}
	if (Y != 0)
	{
		*Y = Tracking_Target.Y;
	}
}

static void Tracking_StopAndReset(void)
{
	Tracking_SetMotorSpeed(0, 0);
	PID_Reset(&Tracking_PidX);
	PID_Reset(&Tracking_PidY);
}

static void Tracking_SetMotorSpeed(int16_t SpeedX, int16_t SpeedY)
{
	/* 当前接线：电机2控制水平轴，电机1控制竖直轴 */
	if (SpeedX != Tracking_LastSpeedX)
	{
		Stepper_SetSpeed(STEPPER_MOTOR_2, SpeedX);
		Tracking_LastSpeedX = SpeedX;
	}

	if (SpeedY != Tracking_LastSpeedY)
	{
		Stepper_SetSpeed(STEPPER_MOTOR_1, SpeedY);
		Tracking_LastSpeedY = SpeedY;
	}
}

static void Tracking_ReadPacket(Tracking_PointTypeDef *Laser, Tracking_PointTypeDef Quad[])
{
	uint8_t i;

	Laser->X = Serial_RxPacket[0];
	Laser->Y = Serial_RxPacket[1];
	for (i = 0; i < TRACKING_QUAD_POINT_NUM; i++)
	{
		Quad[i].X = Serial_RxPacket[2 + i * 2];
		Quad[i].Y = Serial_RxPacket[3 + i * 2];
	}
}

static void Tracking_ResetQuadProgress(uint16_t HoldCount)
{
	Tracking_QuadEdge = 0;
	Tracking_QuadStep = 0;
	Tracking_QuadHoldCount = HoldCount;
}

static void Tracking_SetQuadPoints(Tracking_PointTypeDef Quad[])
{
	uint8_t i;

	for (i = 0; i < TRACKING_QUAD_POINT_NUM; i++)
	{
		Tracking_Quad[i] = Quad[i];
	}
}

static void Tracking_UpdateQuadTarget(void)
{
	uint8_t NextEdge;
	uint16_t SectionLeft;
	int32_t TargetX;
	int32_t TargetY;

	if (Tracking_QuadSection == 0)
	{
		Tracking_QuadSection = 1;
	}

	NextEdge = Tracking_QuadEdge + 1;
	if (NextEdge >= TRACKING_QUAD_POINT_NUM)
	{
		NextEdge = 0;
	}

	SectionLeft = Tracking_QuadSection - Tracking_QuadStep;
	TargetX = ((int32_t)Tracking_Quad[Tracking_QuadEdge].X * SectionLeft
	         + (int32_t)Tracking_Quad[NextEdge].X * Tracking_QuadStep
	         + Tracking_QuadSection / 2) / Tracking_QuadSection;
	TargetY = ((int32_t)Tracking_Quad[Tracking_QuadEdge].Y * SectionLeft
	         + (int32_t)Tracking_Quad[NextEdge].Y * Tracking_QuadStep
	         + Tracking_QuadSection / 2) / Tracking_QuadSection;

	Tracking_Target.X = Tracking_ClampPixel((int16_t)TargetX);
	Tracking_Target.Y = Tracking_ClampPixel((int16_t)TargetY);
}

static void Tracking_MoveQuadTarget(void)
{
	if ((Tracking_QuadEnableFlag == 0) || (Tracking_QuadLocked == 0))
	{
		return;
	}

	if (Tracking_QuadHoldCount > 0)
	{
		Tracking_QuadHoldCount--;
		return;
	}

	if (Tracking_QuadStep < Tracking_QuadSection)
	{
		Tracking_QuadStep++;
	}
	else
	{
		Tracking_QuadStep = 0;
		Tracking_QuadEdge++;
		if (Tracking_QuadEdge >= TRACKING_QUAD_POINT_NUM)
		{
			Tracking_QuadEdge = 0;
		}
	}
}

static uint8_t Tracking_ClampPixel(int16_t Value)
{
	if (Value < 0)
	{
		return 0;
	}
	if (Value > TRACKING_IMAGE_MAX)
	{
		return TRACKING_IMAGE_MAX;
	}
	return (uint8_t)Value;
}

static uint8_t Tracking_IsPointValid(Tracking_PointTypeDef Point)
{
	if ((Point.X < TRACKING_IMAGE_SIZE) && (Point.Y < TRACKING_IMAGE_SIZE))
	{
		return 1;
	}
	return 0;
}

static uint8_t Tracking_IsQuadValid(Tracking_PointTypeDef Quad[])
{
	uint8_t i;
	uint8_t Next;
	int32_t Area;

	Area = 0;
	for (i = 0; i < TRACKING_QUAD_POINT_NUM; i++)
	{
		if (Tracking_IsPointValid(Quad[i]) == 0)
		{
			return 0;
		}

		Next = i + 1;
		if (Next >= TRACKING_QUAD_POINT_NUM)
		{
			Next = 0;
		}

		Area += (int32_t)Quad[i].X * Quad[Next].Y
		      - (int32_t)Quad[Next].X * Quad[i].Y;
	}

	if (Area < 0)
	{
		Area = -Area;
	}

	if (Area < TRACKING_MIN_QUAD_AREA)
	{
		return 0;
	}
	return 1;
}

static int16_t Tracking_CalcAxisSpeed(PID_TypeDef *Pid, int16_t Error)
{
	float Output;
	int16_t Speed;

	if ((Error >= -TRACKING_DEAD_ZONE_PIXEL) && (Error <= TRACKING_DEAD_ZONE_PIXEL))
	{
		PID_Reset(Pid);
		return 0;
	}

	Output = PID_Update(Pid, (float)Error);
	if (Output > TRACKING_MAX_SPEED)
	{
		Speed = TRACKING_MAX_SPEED;
	}
	else if (Output < -TRACKING_MAX_SPEED)
	{
		Speed = -TRACKING_MAX_SPEED;
	}
	else
	{
		Speed = (int16_t)Output;
	}

	if ((Speed > 0) && (Speed < TRACKING_MIN_SPEED))
	{
		Speed = TRACKING_MIN_SPEED;
	}
	else if ((Speed < 0) && (Speed > -TRACKING_MIN_SPEED))
	{
		Speed = -TRACKING_MIN_SPEED;
	}

	return Speed;
}
