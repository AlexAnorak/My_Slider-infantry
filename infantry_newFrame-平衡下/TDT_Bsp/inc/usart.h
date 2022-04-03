/*****************************************************************************
File name: TDT_Bsp\src\usart.h
Description: 串口
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __USART_H__
#define __USART_H__

#include "board.h"

#pragma pack(1)
typedef struct
{
	u8 frameHeader;
	u8 structLenth;
	uint64_t currentTime;
	float CPU_usage;
	float pitch;
	float yaw;
	float roll;
	float icm20602Dps0;
	float icm20602Dps1;
	float icm20602Dps2;
	float mpu6050Dps0;
	float mpu6050Dps1;
	float mpu6050Dps2;
	float icm20602acc0;
	float icm20602acc1;
	float icm20602acc2;
	float mpu6050acc0;
	float mpu6050acc1;
	float mpu6050acc2;
	u8 deforceFlag;
	u16 CRC16Check;
}Record_Send_Struct;
#pragma pack()

extern Record_Send_Struct record_Send_Struct;
void TDT_Communicate_Init(void);
void Communicate_Update(void);  

#endif
