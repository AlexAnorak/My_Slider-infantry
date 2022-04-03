/******************************
File name: TDT_Bsp\src\usart.cpp
Description: 串口
function:
	——————————————————————————————————————————————————————————————————————————
	
	——————————————————————————————————————————————————————————————————————————
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "usart.h"
#include "crc.h"

Record_Send_Struct record_Send_Struct;

void TDT_Communicate_Init(void)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); //PA9  usart1 rx
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10,GPIO_AF_USART3); //PA10  usart1 Tx

	/* -------------- Configure GPIO ---------------------------------------*/
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 |GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		USART_DeInit(USART3);

		USART_InitStructure.USART_BaudRate = 921600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART3, &USART_InitStructure);

		USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

//		USART_ClearFlag(USART3, USART_FLAG_IDLE);
//		USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

		USART_Cmd(USART3, ENABLE);
	}

//	/* -------------- Configure NVIC ---------------------------------------*/
//	{
//			NVIC_InitTypeDef NVIC_InitStructure;
//		
//			NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
//			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//			NVIC_Init(&NVIC_InitStructure);
//	}

//	//DMA2 stream5 ch4  or (DMA2 stream2 ch4)    !!!!!!! P206 of the datasheet
//	/* -------------- Configure RX DMA -----------------------------------------*/
//	{
//			DMA_InitTypeDef DMA_InitStructure;
//		
//			DMA_DeInit(DMA2_Stream5);

//			DMA_InitStructure.DMA_Channel = DMA_Channel_4;
//			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
//			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)CONTROL_rx_buf[0];
//			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//			DMA_InitStructure.DMA_BufferSize = CONTROL_RX_BUF_NUM;
//			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//			DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//			DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//			DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//			DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//			DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//			DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//			DMA_Init(DMA2_Stream5, &DMA_InitStructure);
//		
//			DMA_DoubleBufferModeConfig(DMA2_Stream5, (uint32_t)CONTROL_rx_buf[1], DMA_Memory_0);
//			DMA_DoubleBufferModeCmd(DMA2_Stream5, ENABLE);
//			DMA_Cmd(DMA2_Stream5, DISABLE); //Add a disable
//			DMA_Cmd(DMA2_Stream5, ENABLE);
//	}
	/* -------------- Configure TX DMA -----------------------------------------*/
	{
		DMA_InitTypeDef DMA_InitStructure;
			
		DMA_DeInit(DMA1_Stream3);

		DMA_DeInit(DMA1_Stream3);
		DMA_InitStructure.DMA_Channel= DMA_Channel_4;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA1_Stream3,&DMA_InitStructure);
	}
}


#include "imu_task.h"
#include "schedule.h"
#include "dbus.h"
void Communicate_Update(void)  
{
	DMA_Cmd(DMA1_Stream3,DISABLE); 

	record_Send_Struct.currentTime = getSysTimeUs();
	record_Send_Struct.CPU_usage = scheduleLoop.CPU_usage;
	record_Send_Struct.yaw = boardImu->Angle.yaw;
	record_Send_Struct.pitch = boardImu->Angle.pitch;
	record_Send_Struct.roll = boardImu->Angle.roll;
	record_Send_Struct.icm20602Dps0 = boardImu->validImu[0]->gyro.dps.data[0];
	record_Send_Struct.icm20602Dps1 = boardImu->validImu[0]->gyro.dps.data[1];
	record_Send_Struct.icm20602Dps2 = boardImu->validImu[0]->gyro.dps.data[2];
	record_Send_Struct.mpu6050Dps0 = boardImu->validImu[1]->gyro.dps.data[0];
	record_Send_Struct.mpu6050Dps1 = boardImu->validImu[1]->gyro.dps.data[1];
	record_Send_Struct.mpu6050Dps2 = boardImu->validImu[1]->gyro.dps.data[2];
	record_Send_Struct.icm20602acc0 = boardImu->validImu[0]->acc.accValue.data[0];
	record_Send_Struct.icm20602acc1 = boardImu->validImu[0]->acc.accValue.data[1];
	record_Send_Struct.icm20602acc2 = boardImu->validImu[0]->acc.accValue.data[2];
	record_Send_Struct.mpu6050acc0 = boardImu->validImu[1]->acc.accValue.data[0];
	record_Send_Struct.mpu6050acc1 = boardImu->validImu[1]->acc.accValue.data[1];
	record_Send_Struct.mpu6050acc2 = boardImu->validImu[1]->acc.accValue.data[2];
	record_Send_Struct.deforceFlag = deforceFlag;

	record_Send_Struct.frameHeader = 0xA5;
	record_Send_Struct.structLenth = sizeof(Record_Send_Struct);

	
	Append_CRC16_Check_Sum((u8*)&record_Send_Struct,sizeof(Record_Send_Struct));
	
	while(DMA_GetCmdStatus(DMA1_Stream3) == ENABLE);
	DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);
    //设置传输数据长度
	DMA_MemoryTargetConfig(DMA1_Stream3,(uint32_t)&record_Send_Struct,DMA_Memory_0);
    DMA_SetCurrDataCounter(DMA1_Stream3,sizeof(Record_Send_Struct));  
    //打开DMA,开始发送  
    DMA_Cmd(DMA1_Stream3,ENABLE); 
}  


