#include "vision.h"
#include "string.h"
#include "judgement.h"
#include "crc.h"
#include "can.h"
#include "dbus.h"
#include "chassis_task.h"
#include "state_task.h"

unsigned char SuperPowerRx_buffer[sizeof(RecvStruct_t)+1];
unsigned char SuperPowerTx_buffer[sizeof(SendStruct_t)];
RecvStruct_t RecvStruct; 
SendStruct_t SendStruct;
DMA_InitTypeDef power_Rx_DMA_InitStructure;
DMA_InitTypeDef power_Tx_DMA_InitStructure;
u8 superPowerOffline,superPowerOfflineCheck;

DMA_InitTypeDef vision_Rx_DMA_InitStructure;
DMA_InitTypeDef vision_Tx_DMA_InitStructure;
/**
 * @ingroup TDT_DEVICE
 * @defgroup TDT_DEVICE_VISION 视觉通信
 * @note 如果视觉通信不通过，请检查下列  
 * - 串口连接是否正常
 * - 是否已调用初始化以及发送函数
 * - 其他地方是否有定义USART1_IRQHandler函数
 * - 发送接受结构体是否正确
 * - 是否一字节对齐 (#pragma pack(1))
 * @{
 */
void Vision_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE); //GPIOA，DMA时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);                      //USART1时钟使能

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  //GPIOA9，USART1，TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10，USART1，RX

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_DeInit(USART1);
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_DeInit(DMA2_Stream5);
    vision_Rx_DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    vision_Rx_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
    vision_Rx_DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SuperPowerRx_buffer;
    vision_Rx_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    vision_Rx_DMA_InitStructure.DMA_BufferSize = sizeof(SuperPowerRx_buffer);
    vision_Rx_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    vision_Rx_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    vision_Rx_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    vision_Rx_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    vision_Rx_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    vision_Rx_DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    vision_Rx_DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    vision_Rx_DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    vision_Rx_DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    vision_Rx_DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream5, &vision_Rx_DMA_InitStructure);
    DMA_Cmd(DMA2_Stream5, ENABLE);

    DMA_DeInit(DMA2_Stream7);
    vision_Tx_DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    vision_Tx_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
    vision_Tx_DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&SendStruct;
    vision_Tx_DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    vision_Tx_DMA_InitStructure.DMA_BufferSize = sizeof(SendStruct_t);
    vision_Tx_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    vision_Tx_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    vision_Tx_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    vision_Tx_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    vision_Tx_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    vision_Tx_DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    vision_Tx_DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    vision_Tx_DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    vision_Tx_DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    vision_Tx_DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream7, &vision_Tx_DMA_InitStructure);

}

/**
 * @addtogroup TDT_DEVICE_VISION
 * @brief 视觉数据发送，如果是应答模式( ANSWER_MODE = 1)，则在串口1接收中断发送；否则在陀螺仪计算完之后发送
 * @sa ANSWER_MODE
 */
void vision_Send_Data()
{
    SendStruct.FrameHeader = 0xA5;
    SendStruct.chassis_power = judgement.powerHeatData.chassisPower;
    SendStruct.max_chassis_power = judgement.gameRobotStatus.chassisPowerLimit;
    SendStruct.chassis_power_buffer = judgement.powerHeatData.chassisPowerBuffer;
	SendStruct.PowerPath_Switch=1;//canStateStruct.PowerPath_Switch;
	SendStruct.Check_Mode = canStateStruct.Check_Mode;
	SendStruct.ULTS_Mode = canStateStruct.ULTS_Mode;
	SendStruct.FrameTailer = 0xD2;
    //设置传输数据长度
    DMA_Cmd(DMA2_Stream7, DISABLE);
    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE)
        ;
    DMA_DeInit(DMA2_Stream7);
    DMA_Init(DMA2_Stream7, &vision_Tx_DMA_InitStructure);
    //打开DMA,开始发送
    DMA_Cmd(DMA2_Stream7, ENABLE);
}

//串口中断
void USART1_IRQHandler(void)
{
	uint8_t data;
    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
		data = USART3->SR;
        data = USART3->DR;
        DMA_Cmd(DMA1_Stream1,DISABLE);
		USART_ClearITPendingBit(USART3, USART_IT_IDLE);
        if (SuperPowerRx_buffer[0] == 0xA5&& SuperPowerRx_buffer[sizeof(SuperPowerRx_buffer) - 2] == 0xD2)
        {
			can1Feedback.SuperPowerOfflineCheck = 0;
			can1Feedback.SuperPowerOffline = 0;
			u8 firstRecv = 0;
			if(RecvStruct.capacitance_percentage == 0)
			{
				firstRecv = 1;
			}
			memcpy((u8*)(&RecvStruct), SuperPowerRx_buffer, sizeof(RecvStruct_t));
			if(RecvStruct.capacitance_percentage < 50 && firstRecv == 1)
			{
				RC.chassisRecv.tryUsingBackup = 1;
				chassis.powerParam.usingBackup = 1;
			}
        }
		while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE)
		{
		}
        DMA_DeInit(DMA1_Stream1);
        DMA_Init(DMA1_Stream1, &power_Rx_DMA_InitStructure);
        DMA_SetCurrDataCounter(DMA1_Stream1, sizeof(SuperPowerRx_buffer));
        DMA_Cmd(DMA1_Stream1,ENABLE);
    }
}
