#include "Tracking.h"
#include "Serial.h"
#include "Stepper.h"

#define TRACKING_IMAGE_MAX_X          239
#define TRACKING_IMAGE_MAX_Y          239
#define TRACKING_DEFAULT_TARGET_X     120
#define TRACKING_DEFAULT_TARGET_Y     120

#define TRACKING_DEAD_ZONE_PIXEL      3
#define TRACKING_MIN_SPEED            80
#define TRACKING_MAX_SPEED            800
#define TRACKING_LOST_COUNT           20

/* 如果实际电机正方向与云台输入正方向相反，把对应值改为 -1 */
#define TRACKING_MOTOR_X_DIR_SIGN     1
#define TRACKING_MOTOR_Y_DIR_SIGN     1

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float ErrorLast;
	float ErrorLast2;
	float Output;
} Tracking_PIDTypeDef;

static Tracking_PIDTypeDef Tracking_PidX;
static Tracking_PIDTypeDef Tracking_PidY;

static uint8_t Tracking_TargetX;
static uint8_t Tracking_TargetY;
static uint8_t Tracking_LaserX;
static uint8_t Tracking_LaserY;
static uint8_t Tracking_LaserValid;
static uint8_t Tracking_EnableFlag;
static uint16_t Tracking_NoPacketCount;
static int16_t Tracking_LastSpeedX;
static int16_t Tracking_LastSpeedY;

static int16_t Tracking_Abs16(int16_t Value)
{
	if (Value < 0)
	{
		return -Value;
	}
	return Value;
}

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

static uint8_t Tracking_IsPointValid(uint8_t X, uint8_t Y)
{
	if ((X <= TRACKING_IMAGE_MAX_X) && (Y <= TRACKING_IMAGE_MAX_Y))
	{
		return 1;
	}
	return 0;
}

static void Tracking_PIDConfig(Tracking_PIDTypeDef *Pid, float Kp, float Ki, float Kd)
{
	Pid->Kp = Kp;
	Pid->Ki = Ki;
	Pid->Kd = Kd;
	Pid->ErrorLast = 0.0f;
	Pid->ErrorLast2 = 0.0f;
	Pid->Output = 0.0f;
}

static void Tracking_PIDReset(Tracking_PIDTypeDef *Pid)
{
	Pid->ErrorLast = 0.0f;
	Pid->ErrorLast2 = 0.0f;
	Pid->Output = 0.0f;
}

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

static int16_t Tracking_ApplyDirSign(int16_t Speed, int8_t DirSign)
{
	if (DirSign < 0)
	{
		return -Speed;
	}
	return Speed;
}

static int16_t Tracking_PIDUpdate(Tracking_PIDTypeDef *Pid, int16_t Error)
{
	float ErrorNow;
	float Delta;

	ErrorNow = (float)Error;

	/* 增量式 PID：输出值表示电机速度，单位 step/s */
	Delta = Pid->Kp * (ErrorNow - Pid->ErrorLast)
	      + Pid->Ki * ErrorNow
	      + Pid->Kd * (ErrorNow - 2.0f * Pid->ErrorLast + Pid->ErrorLast2);

	Pid->Output += Delta;
	Pid->ErrorLast2 = Pid->ErrorLast;
	Pid->ErrorLast = ErrorNow;

	return Tracking_ApplyMinSpeed(Tracking_LimitSpeed(Pid->Output));
}

static void Tracking_SetMotorSpeed(int16_t SpeedX, int16_t SpeedY)
{
	if (SpeedX != Tracking_LastSpeedX)
	{
		Stepper_SetSpeed(STEPPER_MOTOR_1, SpeedX);
		Tracking_LastSpeedX = SpeedX;
	}

	if (SpeedY != Tracking_LastSpeedY)
	{
		Stepper_SetSpeed(STEPPER_MOTOR_2, SpeedY);
		Tracking_LastSpeedY = SpeedY;
	}
}

static void Tracking_StopAndReset(void)
{
	Tracking_SetMotorSpeed(0, 0);
	Tracking_PIDReset(&Tracking_PidX);
	Tracking_PIDReset(&Tracking_PidY);
}

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

	/* 先只使用 PD 控制，Ki 保持 0，调试稳定后再小幅增加 */
	Tracking_PIDConfig(&Tracking_PidX, 3.0f, 0.0f, 0.5f);
	Tracking_PIDConfig(&Tracking_PidY, 3.0f, 0.0f, 0.5f);

	Stepper_StopBoth();
}

void Tracking_SetTarget(int16_t TargetX, int16_t TargetY)
{
	Tracking_TargetX = Tracking_ClampPixel(TargetX);
	Tracking_TargetY = Tracking_ClampPixel(TargetY);
	Tracking_StopAndReset();
}

void Tracking_Enable(uint8_t Enable)
{
	if (Enable)
	{
		Tracking_EnableFlag = 1;
		Tracking_NoPacketCount = 0;
		Tracking_PIDReset(&Tracking_PidX);
		Tracking_PIDReset(&Tracking_PidY);
	}
	else
	{
		Tracking_EnableFlag = 0;
		Tracking_StopAndReset();
	}
}

void Tracking_Task(void)
{
	uint8_t RxX;
	uint8_t RxY;
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

	/* 数据包前两个数据固定作为激光点像素坐标：第 0 字节是 x，第 1 字节是 y */
	RxX = Serial_RxPacket[0];
	RxY = Serial_RxPacket[1];

	if (Tracking_IsPointValid(RxX, RxY) == 0)
	{
		Tracking_LaserValid = 0;
		Tracking_NoPacketCount = 0;
		Tracking_StopAndReset();
		return;
	}

	Tracking_LaserX = RxX;
	Tracking_LaserY = RxY;
	Tracking_LaserValid = 1;
	Tracking_NoPacketCount = 0;

	/* 图像 x 向右为正，云台 x 向右为正，误差直接使用目标减当前位置 */
	ErrorX = (int16_t)Tracking_TargetX - (int16_t)Tracking_LaserX;

	/* 图像 y 向下为正，云台 y 向上为正，所以竖直轴误差需要反向 */
	ErrorY = (int16_t)Tracking_LaserY - (int16_t)Tracking_TargetY;

	if (Tracking_Abs16(ErrorX) <= TRACKING_DEAD_ZONE_PIXEL)
	{
		SpeedX = 0;
		Tracking_PIDReset(&Tracking_PidX);
	}
	else
	{
		SpeedX = Tracking_PIDUpdate(&Tracking_PidX, ErrorX);
	}

	if (Tracking_Abs16(ErrorY) <= TRACKING_DEAD_ZONE_PIXEL)
	{
		SpeedY = 0;
		Tracking_PIDReset(&Tracking_PidY);
	}
	else
	{
		SpeedY = Tracking_PIDUpdate(&Tracking_PidY, ErrorY);
	}

	SpeedX = Tracking_ApplyDirSign(SpeedX, TRACKING_MOTOR_X_DIR_SIGN);
	SpeedY = Tracking_ApplyDirSign(SpeedY, TRACKING_MOTOR_Y_DIR_SIGN);

	Tracking_SetMotorSpeed(SpeedX, SpeedY);
}

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
