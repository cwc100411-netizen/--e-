#include "Tracking.h"
#include "Serial.h"
#include "Stepper.h"
#include "pid.h"

/* 常用调试参数：后续调循迹速度和稳定性时优先改这里 */
#define TRACKING_IMAGE_SIZE             240     /* 摄像头图像宽高，当前为 240x240 */
#define TRACKING_CENTER                 120     /* 默认目标中心坐标 */
#define TRACKING_DEAD_ZONE_PIXEL        1       /* 像素误差死区，x 和 y 都 <=2 时认为到达目标点 */
#define TRACKING_MIN_SPEED              20      /* 电机最小启动速度，单位 step/s */
#define TRACKING_MAX_SPEED              500     /* 电机最大速度限制，单位 step/s */
#define TRACKING_LOST_LIMIT             20      /* 连续丢包次数上限，超过后停止追踪 */
#define TRACKING_QUAD_DEFAULT_SECTION   20     /* 四边形每条边默认分段数，越大循迹越慢 */
#define TRACKING_QUAD_START_HOLD        50     /* 锁定黑框后原地等待的控制周期数 */
#define TRACKING_QUAD_LOCK_COUNT        5       /* 连续收到有效黑框的帧数，达到后锁定 */
#define TRACKING_MIN_QUAD_AREA          100     /* 有效四边形最小面积，过滤噪声点 */
#define TRACKING_QUAD_SHRINK_SCALE      0.99f   /* 参考工程 SC1，黑框目标点向中心收缩 */
#define TRACKING_QUAD_SHIFT_X           0.0f    /* 参考工程 shifting_x，收缩后 x 方向偏移 */
#define TRACKING_QUAD_SHIFT_Y           0.0f    /* 参考工程 shifting_y，收缩后 y 方向偏移 */

/* PID 参数：先只用 P，抖动明显时再小幅加 D */
#define TRACKING_PID_KP                 2.0f    /* 比例系数，越大响应越快，也更容易抖 */
#define TRACKING_PID_KI                 0.0f    /* 积分系数，当前不使用 */
#define TRACKING_PID_KD                 0.3f    /* 微分系数，抖动时可小幅增加 */

/* 如果实际电机方向相反，把对应值改为 -1 */
#define TRACKING_MOTOR_X_DIR_SIGN       1       /* 水平轴方向修正，反向时改为 -1 */
#define TRACKING_MOTOR_Y_DIR_SIGN       1       /* 竖直轴方向修正，反向时改为 -1 */

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

static PID_TypeDef Tracking_PidX;                         /* 水平轴 PID 控制器 */
static PID_TypeDef Tracking_PidY;                         /* 竖直轴 PID 控制器 */

static Tracking_PointTypeDef Tracking_Target;             /* 当前追踪目标坐标 */
static Tracking_PointTypeDef Tracking_Laser;              /* 最近一次激光点坐标 */
static Tracking_PointTypeDef Tracking_Quad[TRACKING_QUAD_POINT_NUM]; /* 四边形循迹顶点坐标 */

static uint8_t Tracking_LaserValid;                       /* 激光点坐标有效标志 */
static uint8_t Tracking_NewPacketFlag;                    /* 本次任务是否收到新的激光点 */
static uint8_t Tracking_EnableFlag;                       /* 追踪功能使能标志 */
static uint8_t Tracking_QuadEnableFlag;                   /* 四边形循迹使能标志 */
static uint8_t Tracking_QuadLocked;                       /* 四边形顶点锁定标志 */
static uint8_t Tracking_QuadLockCount;                    /* 连续有效四边形帧计数 */
static uint8_t Tracking_QuadFinished;                     /* 四边形是否已经完整循迹一圈 */
static uint8_t Tracking_QuadEdge;                         /* 当前正在循迹的边序号 */
static uint16_t Tracking_NoPacketCount;                   /* 连续未收到数据包计数 */
static uint16_t Tracking_QuadSection;                     /* 每条边的插值分段数 */
static uint16_t Tracking_QuadStep;                        /* 当前边上的插值步数 */
static uint16_t Tracking_QuadHoldCount;                   /* 锁定后原地等待计数 */
static int16_t Tracking_LastSpeedX;                       /* 上一次水平电机速度 */
static int16_t Tracking_LastSpeedY;                       /* 上一次竖直电机速度 */

static void Tracking_StopAndReset(void);
static void Tracking_SetMotorSpeed(int16_t SpeedX, int16_t SpeedY);
static void Tracking_ReadPacket(Tracking_PointTypeDef *Laser, Tracking_PointTypeDef Quad[]);
static void Tracking_ResetQuadProgress(uint16_t HoldCount);
static void Tracking_SetQuadPoints(Tracking_PointTypeDef Quad[]);
static void Tracking_ShrinkQuadPoints(Tracking_PointTypeDef Quad[]);
static void Tracking_UpdateQuadTarget(void);
static void Tracking_MoveQuadTarget(void);
static uint8_t Tracking_ClampPixel(int16_t Value);
static uint8_t Tracking_IsPointValid(Tracking_PointTypeDef Point);
static uint8_t Tracking_IsQuadValid(Tracking_PointTypeDef Quad[]);
static uint8_t Tracking_IsTargetReached(int16_t ErrorX, int16_t ErrorY);
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
	Tracking_NewPacketFlag = 0;
	Tracking_EnableFlag = 0;
	Tracking_QuadEnableFlag = 0;
	Tracking_QuadLocked = 0;
	Tracking_QuadLockCount = 0;
	Tracking_QuadFinished = 0;
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

	Tracking_NewPacketFlag = 0;

	if (Tracking_EnableFlag == 0)
	{
		Tracking_StopAndReset();
		return;
	}

	if ((Tracking_QuadEnableFlag != 0) && (Tracking_QuadFinished != 0))
	{
		Tracking_StopAndReset();
		return;
	}

	if (Serial_PeekRxFlag() == 0)
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

	if (Serial_RxType != SERIAL_PACKET_TYPE_TRACKING)
	{
		Serial_ClearRxFlag();
		return;
	}

	Serial_ClearRxFlag();

	/* 数据包格式：激光 x,y，或者激光 x,y + 四边形四个顶点 x,y */
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
	Tracking_NewPacketFlag = 1;
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

	if (Tracking_IsTargetReached(ErrorX, ErrorY))
	{
		/* 四边形模式到点后推进到下一个插值目标点；普通固定目标模式会直接返回。 */
		Tracking_MoveQuadTarget();
	}
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
		Tracking_QuadFinished = 0;
		Tracking_ResetQuadProgress(TRACKING_QUAD_START_HOLD);
		Tracking_UpdateQuadTarget();
	}
	else
	{
		Tracking_QuadEnableFlag = 0;
		/* 关闭四边形模式时清掉完成和锁定状态，避免下次切换模式后被旧完成标志拉回 idle */
		Tracking_QuadLocked = 0;
		Tracking_QuadLockCount = 0;
		Tracking_QuadFinished = 0;
		Tracking_ResetQuadProgress(0);
	}

	Tracking_StopAndReset();
}

uint8_t Tracking_IsQuadrilateralFinished(void)
{
	return Tracking_QuadFinished;
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
	Tracking_QuadFinished = 0;
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
	Tracking_QuadFinished = 0;
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
	Tracking_UpdateTarget(TargetX, TargetY);
	Tracking_StopAndReset();
}

/**
  * 函    数：只更新当前追踪目标
  * 参    数：TargetX 目标 x 坐标
  * 参    数：TargetY 目标 y 坐标
  * 返 回 值：无
  */
void Tracking_UpdateTarget(int16_t TargetX, int16_t TargetY)
{
	Tracking_Target.X = Tracking_ClampPixel(TargetX);
	Tracking_Target.Y = Tracking_ClampPixel(TargetY);
}

/**
  * 函    数：判断当前激光点是否到达当前目标
  * 参    数：无
  * 返 回 值：1 到达，0 未到达
  */
uint8_t Tracking_IsTargetReachedNow(void)
{
	int16_t ErrorX;
	int16_t ErrorY;

	if ((Tracking_LaserValid == 0) || (Tracking_NewPacketFlag == 0))
	{
		return 0;
	}

	ErrorX = (int16_t)Tracking_Target.X - (int16_t)Tracking_Laser.X;
	ErrorY = (int16_t)Tracking_Laser.Y - (int16_t)Tracking_Target.Y;
	return Tracking_IsTargetReached(ErrorX, ErrorY);
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
		if (Serial_RxLength >= SERIAL_RX_PACKET_LENGTH)
		{
			Quad[i].X = Serial_RxPacket[2 + i * 2];
			Quad[i].Y = Serial_RxPacket[3 + i * 2];
		}
		else
		{
			/* 短包只有激光点坐标，没有四边形顶点。 */
			Quad[i].X = 0;
			Quad[i].Y = 0;
		}
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

	Tracking_ShrinkQuadPoints(Quad);
	for (i = 0; i < TRACKING_QUAD_POINT_NUM; i++)
	{
		Tracking_Quad[i] = Quad[i];
	}
}

static void Tracking_ShrinkQuadPoints(Tracking_PointTypeDef Quad[])
{
	uint8_t i;
	int16_t CenterX;
	int16_t CenterY;
	float ShrinkX;
	float ShrinkY;

	/* 参考工程公式：点 = 中心 + SC1 * (点 - 中心) + 偏移 */
	CenterX = (Quad[0].X + Quad[1].X + Quad[2].X + Quad[3].X) / 4;
	CenterY = (Quad[0].Y + Quad[1].Y + Quad[2].Y + Quad[3].Y) / 4;

	for (i = 0; i < TRACKING_QUAD_POINT_NUM; i++)
	{
		ShrinkX = CenterX + TRACKING_QUAD_SHRINK_SCALE * (Quad[i].X - CenterX) + TRACKING_QUAD_SHIFT_X;
		ShrinkY = CenterY + TRACKING_QUAD_SHRINK_SCALE * (Quad[i].Y - CenterY) + TRACKING_QUAD_SHIFT_Y;
		Quad[i].X = Tracking_ClampPixel((int16_t)ShrinkX);
		Quad[i].Y = Tracking_ClampPixel((int16_t)ShrinkY);
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
			/* 矩形四条边全部走完并回到起点后，停止本次循迹 */
			Tracking_QuadEdge = 0;
			Tracking_QuadFinished = 1;
			Tracking_QuadEnableFlag = 0;
			Tracking_EnableFlag = 0;
			Tracking_StopAndReset();
			return;
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

static uint8_t Tracking_IsTargetReached(int16_t ErrorX, int16_t ErrorY)
{
	if (ErrorX < 0)
	{
		ErrorX = -ErrorX;
	}
	if (ErrorY < 0)
	{
		ErrorY = -ErrorY;
	}

	if ((ErrorX <= TRACKING_DEAD_ZONE_PIXEL) && (ErrorY <= TRACKING_DEAD_ZONE_PIXEL))
	{
		return 1;
	}
	return 0;
}

static int16_t Tracking_CalcAxisSpeed(PID_TypeDef *Pid, int16_t Error)
{
	float Output;
	int16_t Speed;

	if (Error == 0)
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
