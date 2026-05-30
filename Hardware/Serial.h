#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

#define SERIAL_RX_PACKET_LENGTH 8

extern uint8_t Serial_TxPacket[];
extern volatile uint8_t Serial_RxPacket[];

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
