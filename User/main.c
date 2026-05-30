#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"

uint8_t KeyNum;			//定义用于接收按键键码的变量

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	Key_Init();			//按键初始化
	Serial_Init();		//串口初始化
	
	/*显示静态字符串*/
	OLED_ShowString(1, 1, "TxPacket");
	OLED_ShowString(3, 1, "RxPacket");
	
	/*设置发送数据包数组的初始值，用于测试*/
	Serial_TxPacket[0] = 0x01;
	Serial_TxPacket[1] = 0x02;
	Serial_TxPacket[2] = 0x03;
	Serial_TxPacket[3] = 0x04;
	
	while (1)
	{
		
//		Serial_TxPacket[0] ++;		//测试数据自增
//		Serial_TxPacket[1] ++;
//		Serial_TxPacket[2] ++;
//		Serial_TxPacket[3] ++;
			
		//Serial_SendPacket();		//串口发送数据包Serial_TxPacket
			
		//OLED_ShowHexNum(2, 1, Serial_TxPacket[0], 2);	//显示发送的数据包
		//OLED_ShowHexNum(2, 4, Serial_TxPacket[1], 2);
		//OLED_ShowHexNum(2, 7, Serial_TxPacket[2], 2);
		//OLED_ShowHexNum(2, 10, Serial_TxPacket[3], 2);
		//Delay_ms(1000);			//延时1秒
		if (Serial_GetRxFlag() == 1)	//如果接收到数据包
		{
			Serial_SendRxPacket();		//将完整接收数据包回传给电脑
			OLED_ShowHexNum(4, 1, Serial_RxPacket[0], 2);	//显示接收的数据包
			OLED_ShowHexNum(4, 3, Serial_RxPacket[1], 2);
			OLED_ShowHexNum(4, 5, Serial_RxPacket[2], 2);
			OLED_ShowHexNum(4, 7, Serial_RxPacket[3], 2);
			OLED_ShowHexNum(4, 9, Serial_RxPacket[4], 2);
			OLED_ShowHexNum(4, 11, Serial_RxPacket[5], 2);
			OLED_ShowHexNum(4, 13, Serial_RxPacket[6], 2);
			OLED_ShowHexNum(4, 15, Serial_RxPacket[7], 2);
		}
	}
}
