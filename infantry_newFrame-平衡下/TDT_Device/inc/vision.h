#ifndef _VISION_H
#define _VISION_H
#include "board.h"

/**
 * @addtogroup TDT_DEVICE_VISION
 * @{
 */

///是否启用应答模式
#pragma pack(1) //用于结构体的内存对齐， 方便使用联合体调用
typedef struct _SendStruct_t
{
    uint8_t FrameHeader;
    float chassis_power;
    uint8_t max_chassis_power;
    uint16_t chassis_power_buffer;
	uint8_t PowerPath_Switch;	    //是否开启电容
	uint8_t Check_Mode;
	uint8_t ULTS_Mode;
	uint8_t FrameTailer;
}SendStruct_t;

typedef struct _RecvStruct_t 
{
    uint8_t FrameHeader;
    float capacitance_percentage;//电容容量百分比
    float voltage;//电容电压
    float local_Power;//功率模块计算功率
	uint8_t FrameTailer;
}RecvStruct_t;
#pragma pack() //用于结构体的内存对齐， 方便使用联合体调用

extern RecvStruct_t RecvStruct; 
extern SendStruct_t SendStruct;

///串口初始化
void Vision_Init(void);
void vision_Send_Data();

#ifdef __cplusplus
extern "C"
{
#endif

	void USART1_IRQHandler(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif
