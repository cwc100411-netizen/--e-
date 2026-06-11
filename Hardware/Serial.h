#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

#define SERIAL_RX_PACKET_LENGTH 10

/* 新增：区分摄像头发来的包类型，避免数字目标包和激光坐标包混在一起 */
#define SERIAL_PACKET_TYPE_NONE          0
#define SERIAL_PACKET_TYPE_TRACKING      1
#define SERIAL_PACKET_TYPE_DIGIT_TARGETS 2

extern uint8_t Serial_TxPacket[];
extern volatile uint8_t Serial_RxPacket[];
extern volatile uint8_t Serial_RxLength;
extern volatile uint8_t Serial_RxType;

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

void Serial_SendPacket(void);
void Serial_SendRxPacket(void);
uint8_t Serial_GetRxFlag(void);

#endif
