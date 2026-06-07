#include "Tracking.h"
#include "Serial.h"
#include "Stepper.h"
#include "pid.h"

#define TRACKING_IMAGE_MAX_X          239
#define TRACKING_IMAGE_MAX_Y          239
#define TRACKING_DEFAULT_TARGET_X     120
#define TRACKING_DEFAULT_TARGET_Y     120

#define TRACKING_DEAD_ZONE_PIXEL       1
#define TRACKING_MIN_SPEED            60
#define TRACKING_MAX_SPEED            500
#define TRACKING_LOST_COUNT           20
#define TRACKING_QUAD_POINT_NUM        4
#define TRACKING_QUAD_DEFAULT_SECTION 200
#define TRACKING_QUAD_START_HOLD_COUNT 200
#define TRACKING_QUAD_LOCK_FRAME_COUNT 5
#define TRACKING_QUAD_SHRINK_PERCENT 0

/* 如果实际电机正方向与云台输入正方向相反，把对应值改为 -1 */
#define TRACKING_MOTOR_X_DIR_SIGN     1
#define TRACKING_MOTOR_Y_DIR_SIGN     1

static PID_TypeDef Tracking_PidX;
static PID_TypeDef Tracking_PidY;

static uint8_t Tracking_TargetX;
static uint8_t Tracking_TargetY;
static uint8_t Tracking_LaserX;
static uint8_t Tracking_LaserY;
static uint8_t Tracking_LaserValid;
static uint8_t Tracking_EnableFlag;
static uint16_t Tracking_NoPacketCount;
static int16_t Tracking_LastSpeedX;
static int16_t Tracking_LastSpeedY;
static uint8_t Tracking_QuadX[TRACKING_QUAD_POINT_NUM];
static uint8_t Tracking_QuadY[TRACKING_QUAD_POINT_NUM];
static uint16_t Tracking_QuadSection;
static uint16_t Tracking_QuadStep;
static uint8_t Tracking_QuadEdge;
static uint8_t Tracking_QuadEnableFlag;
static uint16_t Tracking_QuadHoldCount;
static uint8_t Tracking_QuadLocked;
static uint8_t Tracking_QuadLockCount;

/**
  * 函    数：求 16 位有符号数的绝对值
  * 参    数：Value 要处理的数值
  * 返 回 值：Value 的绝对值
  * 说    明：用于判断像素误差是否进入死区
  */
static int16_t Tracking_Abs16(int16_t Value)
{
	if (Value < 0)
	{
		return -Value;
	}
	return Value;
}

/**
  * 函    数：限制目标像素坐标范围
  * 参    数：Value 输入的目标坐标
  * 返 回 值：限制在图像范围内的坐标值
  * 说    明：当前摄像头分辨率为 240x240，合法坐标为 0~239
  */
static uint8_t Tracking_ClampPixel(int16_t Value)
{
	if (Value < 0)
	{
		return 0;
	}
	if (Value > TRACKING_IMAGE_MAX_X)
	{
		return TRACKING_IMAGE_MAX_X;
	}
	return (uint8_t)Value;
}

/**
  * 函    数：判断激光点坐标是否合法
  * 参    数：X 激光点 x 坐标
  * 参    数：Y 激光点 y 坐标
  * 返 回 值：1 表示坐标合法，0 表示坐标非法
  */
static uint8_t Tracking_IsPointValid(uint8_t X, uint8_t Y)
{
	if ((X <= TRACKING_IMAGE_MAX_X) && (Y <= TRACKING_IMAGE_MAX_Y))
	{
		return 1;
	}
	return 0;
}

/**
  * 函    数：判断四边形顶点坐标是否合法
  * 参    数：X1~Y4 四个顶点坐标
  * 返 回 值：1 表示合法，0 表示非法
  * 说    明：除坐标范围外，额外判断面积，避免全 0 或噪声点误触发循迹
  */
static uint8_t Tracking_IsQuadrilateralValid(uint8_t X1, uint8_t Y1,
                                             uint8_t X2, uint8_t Y2,
                                             uint8_t X3, uint8_t Y3,
                                             uint8_t X4, uint8_t Y4)
{
	int32_t Area2;

	if (Tracking_IsPointValid(X1, Y1) == 0 ||
	    Tracking_IsPointValid(X2, Y2) == 0 ||
	    Tracking_IsPointValid(X3, Y3) == 0 ||
	    Tracking_IsPointValid(X4, Y4) == 0)
	{
		return 0;
	}

	/* 四边形有向面积的 2 倍，阈值太小说明不是有效矩形框 */
	Area2 = (int32_t)X1 * Y2 - (int32_t)X2 * Y1
	      + (int32_t)X2 * Y3 - (int32_t)X3 * Y2
	      + (int32_t)X3 * Y4 - (int32_t)X4 * Y3
	      + (int32_t)X4 * Y1 - (int32_t)X1 * Y4;

	if (Area2 < 0)
	{
		Area2 = -Area2;
	}

	if (Area2 < 100)
	{
		return 0;
	}

	return 1;
}

/**
  * 函    数：计算目标点内缩偏移量
  * 参    数：Value 原始数值
  * 返 回 值：按固定比例缩小后的数值
  */
static int16_t Tracking_GetShrinkOffset(int16_t Value)
{
	int32_t Product;

	Product = (int32_t)Value * TRACKING_QUAD_SHRINK_PERCENT;

	/* 四舍五入，避免小比例收缩时误差全部被截断 */
	if (Product >= 0)
	{
		return (int16_t)((Product + 50) / 100);
	}

	return (int16_t)(-((-Product + 50) / 100));
}

/* 根据当前坐标和相邻两个坐标，计算内缩后的单个坐标 */
static uint8_t Tracking_ShrinkPixel(uint8_t Point, uint8_t Prev, uint8_t Next)
{
	int16_t Target;

	Target = Point;
	Target += Tracking_GetShrinkOffset((int16_t)Prev - (int16_t)Point);
	Target += Tracking_GetShrinkOffset((int16_t)Next - (int16_t)Point);

	return Tracking_ClampPixel(Target);
}

/**
  * 函    数：用串口收到的四个顶点更新四边形路径
  * 参    数：X1~Y4 四个顶点坐标
  * 返 回 值：无
  * 说    明：只更新路径顶点，不停止电机，不清 PID，避免每帧重启循迹
  */
static void Tracking_UpdateQuadrilateralPoints(uint8_t X1, uint8_t Y1,
                                               uint8_t X2, uint8_t Y2,
                                               uint8_t X3, uint8_t Y3,
                                               uint8_t X4, uint8_t Y4)
{
	/* 外框角点沿相邻两条边向内收缩，让目标点落在内外框之间的黑色胶带区域 */
	Tracking_QuadX[0] = Tracking_ShrinkPixel(X1, X4, X2);
	Tracking_QuadY[0] = Tracking_ShrinkPixel(Y1, Y4, Y2);
	Tracking_QuadX[1] = Tracking_ShrinkPixel(X2, X1, X3);
	Tracking_QuadY[1] = Tracking_ShrinkPixel(Y2, Y1, Y3);
	Tracking_QuadX[2] = Tracking_ShrinkPixel(X3, X2, X4);
	Tracking_QuadY[2] = Tracking_ShrinkPixel(Y3, Y2, Y4);
	Tracking_QuadX[3] = Tracking_ShrinkPixel(X4, X3, X1);
	Tracking_QuadY[3] = Tracking_ShrinkPixel(Y4, Y3, Y1);
}

/**
  * 函    数：限制 PID 输出速度范围
  * 参    数：Speed PID 计算出的速度，单位 step/s
  * 返 回 值：限制后的速度，范围为正负 TRACKING_MAX_SPEED
  */
static int16_t Tracking_LimitSpeed(float Speed)
{
	if (Speed > TRACKING_MAX_SPEED)
	{
		Speed = TRACKING_MAX_SPEED;
	}
	else if (Speed < -TRACKING_MAX_SPEED)
	{
		Speed = -TRACKING_MAX_SPEED;
	}

	return (int16_t)Speed;
}

/**
  * 函    数：给非零速度补偿最小启动速度
  * 参    数：Speed 限幅后的速度，单位 step/s
  * 返 回 值：补偿后的速度
  * 说    明：步进电机低于一定速度可能不易启动，因此小速度会提升到最小速度
  */
static int16_t Tracking_ApplyMinSpeed(int16_t Speed)
{
	if (Speed > 0 && Speed < TRACKING_MIN_SPEED)
	{
		return TRACKING_MIN_SPEED;
	}
	if (Speed < 0 && Speed > -TRACKING_MIN_SPEED)
	{
		return -TRACKING_MIN_SPEED;
	}
	return Speed;
}

/**
  * 函    数：根据方向修正系数调整速度正负号
  * 参    数：Speed 原始速度，单位 step/s
  * 参    数：DirSign 方向修正系数，1 保持方向，-1 反向
  * 返 回 值：修正方向后的速度
  */
static int16_t Tracking_ApplyDirSign(int16_t Speed, int8_t DirSign)
{
	if (DirSign < 0)
	{
		return -Speed;
	}
	return Speed;
}

/**
  * 函    数：根据 PID 输出计算电机速度
  * 参    数：Pid 要更新的 PID 结构体
  * 参    数：Error 当前轴的像素误差
  * 返 回 值：限幅并补偿最小启动速度后的电机速度，单位 step/s
  * 说    明：PID 模块只负责计算，Tracking 保留电机速度相关处理
  */
static int16_t Tracking_CalcPidSpeed(PID_TypeDef *Pid, int16_t Error)
{
	return Tracking_ApplyMinSpeed(Tracking_LimitSpeed(PID_Update(Pid, (float)Error)));
}

/**
  * 函    数：设置水平和竖直两个轴的电机速度
  * 参    数：SpeedX 水平轴速度，单位 step/s
  * 参    数：SpeedY 竖直轴速度，单位 step/s
  * 返 回 值：无
  * 说    明：只有速度变化时才重新设置电机，减少重复停止和启动 PWM
  */
static void Tracking_SetMotorSpeed(int16_t SpeedX, int16_t SpeedY)
{
	/* 实测电机2控制水平轴，电机1控制竖直轴 */
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

/**
  * 函    数：只更新目标点，不停止电机，不复位 PID
  * 参    数：TargetX 目标 x 坐标
  * 参    数：TargetY 目标 y 坐标
  * 返 回 值：无
  * 说    明：四边形循迹时目标点会连续移动，不能每次移动都清零 PID
  */
static void Tracking_UpdateTargetOnly(int16_t TargetX, int16_t TargetY)
{
	Tracking_TargetX = Tracking_ClampPixel(TargetX);
	Tracking_TargetY = Tracking_ClampPixel(TargetY);
}

/**
  * 函    数：根据当前边和当前插值步数更新四边形循迹目标点
  * 参    数：无
  * 返 回 值：无
  * 说    明：按 P1->P2->P3->P4->P1 的顺序在四条边上做线性插值
  */
static void Tracking_UpdateQuadrilateralTarget(void)
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

	/* 整数插值，避免使用浮点数，便于 F103 调试 */
	TargetX = ((int32_t)Tracking_QuadX[Tracking_QuadEdge] * SectionLeft
	         + (int32_t)Tracking_QuadX[NextEdge] * Tracking_QuadStep
	         + Tracking_QuadSection / 2) / Tracking_QuadSection;
	TargetY = ((int32_t)Tracking_QuadY[Tracking_QuadEdge] * SectionLeft
	         + (int32_t)Tracking_QuadY[NextEdge] * Tracking_QuadStep
	         + Tracking_QuadSection / 2) / Tracking_QuadSection;

	Tracking_UpdateTargetOnly((int16_t)TargetX, (int16_t)TargetY);
}

/**
  * 函    数：四边形循迹目标点前进一步
  * 参    数：无
  * 返 回 值：无
  * 说    明：每收到一次有效激光点后前进一步，速度由分段数和主循环周期共同决定
  */
static void Tracking_MoveQuadrilateralTarget(void)
{
	if (Tracking_QuadEnableFlag == 0)
	{
		return;
	}

	if (Tracking_QuadLocked == 0)
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

/**
  * 函    数：停止两个轴并复位 PID
  * 参    数：无
  * 返 回 值：无
  * 说    明：用于关闭追踪、丢失激光、坐标无效或切换目标后的安全停止
  */
static void Tracking_StopAndReset(void)
{
	Tracking_SetMotorSpeed(0, 0);
	PID_Reset(&Tracking_PidX);
	PID_Reset(&Tracking_PidY);
}

/**
  * 函    数：初始化追踪模块
  * 参    数：无
  * 返 回 值：无
  * 说    明：设置默认目标点、清空状态、配置 PID 参数并停止电机
  */
void Tracking_Init(void)
{
	Tracking_TargetX = TRACKING_DEFAULT_TARGET_X;
	Tracking_TargetY = TRACKING_DEFAULT_TARGET_Y;
	Tracking_LaserX = 0;
	Tracking_LaserY = 0;
	Tracking_LaserValid = 0;
	Tracking_EnableFlag = 0;
	Tracking_NoPacketCount = 0;
	Tracking_LastSpeedX = 0;
	Tracking_LastSpeedY = 0;
	Tracking_QuadSection = TRACKING_QUAD_DEFAULT_SECTION;
	Tracking_QuadStep = 0;
	Tracking_QuadEdge = 0;
	Tracking_QuadEnableFlag = 0;
	Tracking_QuadHoldCount = 0;
	Tracking_QuadLocked = 0;
	Tracking_QuadLockCount = 0;
	Tracking_QuadX[0] = 60;
	Tracking_QuadY[0] = 60;
	Tracking_QuadX[1] = 180;
	Tracking_QuadY[1] = 60;
	Tracking_QuadX[2] = 180;
	Tracking_QuadY[2] = 180;
	Tracking_QuadX[3] = 60;
	Tracking_QuadY[3] = 180;

	/* 先只使用 PD 控制，Ki 保持 0，调试稳定后再小幅增加 */
	PID_Config(&Tracking_PidX, 2.0f, 0.0f, 0.3f);
	PID_Config(&Tracking_PidY, 2.0f, 0.0f, 0.3f);

	Stepper_StopBoth();
}

/**
  * 函    数：设置追踪目标坐标
  * 参    数：TargetX 目标 x 坐标
  * 参    数：TargetY 目标 y 坐标
  * 返 回 值：无
  * 说    明：目标坐标会被限制在图像范围内，设置后会停止电机并复位 PID
  */
void Tracking_SetTarget(int16_t TargetX, int16_t TargetY)
{
	Tracking_TargetX = Tracking_ClampPixel(TargetX);
	Tracking_TargetY = Tracking_ClampPixel(TargetY);
	Tracking_StopAndReset();
}

/**
  * 函    数：设置四边形循迹的四个顶点
  * 参    数：X1 第 1 个点 x 坐标
  * 参    数：Y1 第 1 个点 y 坐标
  * 参    数：X2 第 2 个点 x 坐标
  * 参    数：Y2 第 2 个点 y 坐标
  * 参    数：X3 第 3 个点 x 坐标
  * 参    数：Y3 第 3 个点 y 坐标
  * 参    数：X4 第 4 个点 x 坐标
  * 参    数：Y4 第 4 个点 y 坐标
  * 返 回 值：无
  * 说    明：请按实际循迹顺序输入四个点，程序会自动连成 P1->P2->P3->P4->P1；
  *           手动调用后会直接视为四边形已锁定
  */
void Tracking_SetQuadrilateral(int16_t X1, int16_t Y1,
                               int16_t X2, int16_t Y2,
                               int16_t X3, int16_t Y3,
                               int16_t X4, int16_t Y4)
{
	Tracking_UpdateQuadrilateralPoints(Tracking_ClampPixel(X1), Tracking_ClampPixel(Y1),
	                                   Tracking_ClampPixel(X2), Tracking_ClampPixel(Y2),
	                                   Tracking_ClampPixel(X3), Tracking_ClampPixel(Y3),
	                                   Tracking_ClampPixel(X4), Tracking_ClampPixel(Y4));

	Tracking_QuadEdge = 0;
	Tracking_QuadStep = 0;
	Tracking_QuadHoldCount = TRACKING_QUAD_START_HOLD_COUNT;
	Tracking_QuadLocked = 1;
	Tracking_QuadLockCount = TRACKING_QUAD_LOCK_FRAME_COUNT;
	Tracking_UpdateQuadrilateralTarget();
	Tracking_StopAndReset();
}

/**
  * 函    数：重新等待串口矩形锁定
  * 参    数：无
  * 返 回 值：无
  * 说    明：更换矩形框或需要重新识别路径时调用，之后会等待新的有效矩形坐标
  */
void Tracking_ResetQuadrilateralLock(void)
{
	Tracking_QuadLocked = 0;
	Tracking_QuadLockCount = 0;
	Tracking_QuadEdge = 0;
	Tracking_QuadStep = 0;
	Tracking_QuadHoldCount = 0;
	Tracking_LaserValid = 0;
	Tracking_StopAndReset();
}

/**
  * 函    数：设置四边形每条边的插值分段数
  * 参    数：Section 每条边分成多少小段，数值越大循迹越慢
  * 返 回 值：无
  * 说    明：主循环 10ms 调用一次时，200 段大约一圈 8 秒
  */
void Tracking_SetQuadrilateralSection(uint16_t Section)
{
	if (Section == 0)
	{
		Section = 1;
	}

	Tracking_QuadSection = Section;
	Tracking_QuadEdge = 0;
	Tracking_QuadStep = 0;
	Tracking_QuadHoldCount = TRACKING_QUAD_START_HOLD_COUNT;
	Tracking_UpdateQuadrilateralTarget();
	Tracking_StopAndReset();
}

/**
  * 函    数：开启或关闭四边形循迹目标生成
  * 参    数：Enable 1 表示使用四边形移动目标，0 表示使用固定目标
  * 返 回 值：无
  */
void Tracking_EnableQuadrilateral(uint8_t Enable)
{
	if (Enable)
	{
		Tracking_QuadEnableFlag = 1;
		Tracking_QuadEdge = 0;
		Tracking_QuadStep = 0;
		Tracking_QuadHoldCount = TRACKING_QUAD_START_HOLD_COUNT;
		Tracking_UpdateQuadrilateralTarget();
	}
	else
	{
		Tracking_QuadEnableFlag = 0;
	}

	Tracking_StopAndReset();
}

/**
  * 函    数：开启或关闭追踪
  * 参    数：Enable 1 表示开启追踪，0 表示关闭追踪
  * 返 回 值：无
  * 说    明：关闭追踪时立即停止两个电机，开启追踪时清空丢包计数和 PID 状态
  */
void Tracking_Enable(uint8_t Enable)
{
	if (Enable)
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
  * 函    数：追踪任务函数
  * 参    数：无
  * 返 回 值：无
  * 说    明：周期调用，读取摄像头串口数据，计算误差并更新两个轴的电机速度
  */
void Tracking_Task(void)
{
	uint8_t RxLaserX;
	uint8_t RxLaserY;
	uint8_t RxP1X;
	uint8_t RxP1Y;
	uint8_t RxP2X;
	uint8_t RxP2Y;
	uint8_t RxP3X;
	uint8_t RxP3Y;
	uint8_t RxP4X;
	uint8_t RxP4Y;
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
		if (Tracking_NoPacketCount < TRACKING_LOST_COUNT)
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
	RxLaserX = Serial_RxPacket[0];
	RxLaserY = Serial_RxPacket[1];
	RxP1X = Serial_RxPacket[2];
	RxP1Y = Serial_RxPacket[3];
	RxP2X = Serial_RxPacket[4];
	RxP2Y = Serial_RxPacket[5];
	RxP3X = Serial_RxPacket[6];
	RxP3Y = Serial_RxPacket[7];
	RxP4X = Serial_RxPacket[8];
	RxP4Y = Serial_RxPacket[9];

	if (Tracking_IsPointValid(RxLaserX, RxLaserY) == 0)
	{
		Tracking_LaserValid = 0;
		Tracking_NoPacketCount = 0;
		Tracking_StopAndReset();
		return;
	}

	if (Tracking_QuadEnableFlag && Tracking_QuadLocked == 0)
	{
		if (Tracking_IsQuadrilateralValid(RxP1X, RxP1Y, RxP2X, RxP2Y,
		                                  RxP3X, RxP3Y, RxP4X, RxP4Y) == 0)
		{
			Tracking_LaserValid = 0;
			Tracking_NoPacketCount = 0;
			Tracking_QuadLockCount = 0;
			Tracking_StopAndReset();
			return;
		}
	}

	Tracking_LaserX = RxLaserX;
	Tracking_LaserY = RxLaserY;
	Tracking_LaserValid = 1;
	Tracking_NoPacketCount = 0;

	if (Tracking_QuadEnableFlag)
	{
		if (Tracking_QuadLocked == 0)
		{
			/* 锁定前连续接收几帧有效矩形，锁定后忽略后续矩形抖动 */
			Tracking_UpdateQuadrilateralPoints(RxP1X, RxP1Y, RxP2X, RxP2Y,
			                                   RxP3X, RxP3Y, RxP4X, RxP4Y);

			if (Tracking_QuadLockCount < TRACKING_QUAD_LOCK_FRAME_COUNT)
			{
				Tracking_QuadLockCount++;
			}

			if (Tracking_QuadLockCount >= TRACKING_QUAD_LOCK_FRAME_COUNT)
			{
				Tracking_QuadLocked = 1;
				Tracking_QuadEdge = 0;
				Tracking_QuadStep = 0;
				Tracking_QuadHoldCount = TRACKING_QUAD_START_HOLD_COUNT;
				PID_Reset(&Tracking_PidX);
				PID_Reset(&Tracking_PidY);
			}
		}

		Tracking_UpdateQuadrilateralTarget();
	}

	/* 图像 x 向右为正，云台 x 向右为正，误差直接使用目标减当前位置 */
	ErrorX = (int16_t)Tracking_TargetX - (int16_t)Tracking_LaserX;

	/* 图像 y 向下为正，云台 y 向上为正，所以竖直轴误差需要反向 */
	ErrorY = (int16_t)Tracking_LaserY - (int16_t)Tracking_TargetY;

	if (Tracking_Abs16(ErrorX) <= TRACKING_DEAD_ZONE_PIXEL)
	{
		SpeedX = 0;
		PID_Reset(&Tracking_PidX);
	}
	else
	{
		SpeedX = Tracking_CalcPidSpeed(&Tracking_PidX, ErrorX);
	}

	if (Tracking_Abs16(ErrorY) <= TRACKING_DEAD_ZONE_PIXEL)
	{
		SpeedY = 0;
		PID_Reset(&Tracking_PidY);
	}
	else
	{
		SpeedY = Tracking_CalcPidSpeed(&Tracking_PidY, ErrorY);
	}

	SpeedX = Tracking_ApplyDirSign(SpeedX, TRACKING_MOTOR_X_DIR_SIGN);
	SpeedY = Tracking_ApplyDirSign(SpeedY, TRACKING_MOTOR_Y_DIR_SIGN);

	Tracking_SetMotorSpeed(SpeedX, SpeedY);
	Tracking_MoveQuadrilateralTarget();
}

/**
  * 函    数：获取最近一次有效激光点坐标
  * 参    数：X 用于保存激光点 x 坐标，可传入 0 表示不读取
  * 参    数：Y 用于保存激光点 y 坐标，可传入 0 表示不读取
  * 返 回 值：1 表示当前有有效激光点，0 表示无有效激光点
  */
uint8_t Tracking_GetLaserPoint(uint8_t *X, uint8_t *Y)
{
	if (Tracking_LaserValid == 0)
	{
		return 0;
	}

	if (X != 0)
	{
		*X = Tracking_LaserX;
	}
	if (Y != 0)
	{
		*Y = Tracking_LaserY;
	}

	return 1;
}

/**
  * 函    数：获取当前目标坐标
  * 参    数：X 用于保存目标 x 坐标，可传入 0 表示不读取
  * 参    数：Y 用于保存目标 y 坐标，可传入 0 表示不读取
  * 返 回 值：无
  */
void Tracking_GetTarget(uint8_t *X, uint8_t *Y)
{
	if (X != 0)
	{
		*X = Tracking_TargetX;
	}
	if (Y != 0)
	{
		*Y = Tracking_TargetY;
	}
}
