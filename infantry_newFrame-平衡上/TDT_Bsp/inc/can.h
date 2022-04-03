/*
 * @Author: your name
 * @Date: 2021-05-09 06:17:01
 * @LastEditTime: 2021-05-10 12:18:02
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Projectd:\TDT\TDT-Infantry\Infantry_II\TDT_Bsp\inc\can.h
 */
/*****************************************************************************
File name: TDT_Bsp\src\can.cpp
Description: can底层
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.16 #合成can1.c和can2.c #修改接收函数，从队列传递改为直接解算
	——————————————————————————————————————————————————————————————————————————
	19.11.12 #首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __CAN1_H__
#define __CAN1_H__

#include "board.h"

///@addtogroup TDT_BSP_CAN
///@{

void canInit(CAN_TypeDef *can_x);
//void TDT_Can_Tx(vec4f* value,CAN_TypeDef * can_x,uint32_t id);
void canTx(float *data, CAN_TypeDef *can_x, uint32_t id);
void canTx(u8 data[8], CAN_TypeDef *can_x, uint32_t id);

///@}

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

	void CAN1_TX_IRQHandler(void);
	void CAN2_TX_IRQHandler(void);
	void CAN1_RX0_IRQHandler(void);
	void CAN2_RX0_IRQHandler(void);
#ifdef __cplusplus
}
#endif /*__cplusplus*/

struct Can1Feedback
{ /*裁判系统数据*/
	float Jgmt_OutSpd;
	u8 jgmtOffline;
	u8 SuperPowerOffline;
	u8 jgmtOfflineCheck;
	u8 SuperPowerOfflineCheck;
	float GYdistance[3];
};

#pragma pack(1)
struct PowerHeatData
{
	uint16_t speedWS:13;
	uint8_t jgmeOffline:1;
	uint16_t bulletSpeed:9;
	uint16_t coolingHeat:16;
	uint16_t shootNum:16;
};
struct JgmtInfoMsg
{
	uint16_t robotLevel:2;
	uint16_t gameProgress:4;
	uint16_t hurtType:3;
	uint16_t enemyColor:2;
	int16_t speedLimit:5;
	int16_t coolingRate:16;
	int16_t coolingLimit:16;
	uint16_t maxHp:16;
};
#pragma pack()

struct Can2Feedback
{
	uint16_t frictionShootNum;	 //摩擦轮统计的发弹数量
	u16 Snail_A_FeedbackSpd_Now; //摩擦轮B实际转速
	u16 Snail_B_FeedbackSpd_Now; //摩擦轮A实际转速
	u8 AS5048_offline;			 //todo 磁编码器离线
	u8 ready_to_fire;			 //摩擦轮允许开火
	u8 frictionOffline;			 //摩擦轮模块离线
	u8 frictionOfflineCheck;	 //摩擦轮模块离线
};

extern PowerHeatData powerHeatData;
extern JgmtInfoMsg jgmtInfoMsg;
extern Can1Feedback can1Feedback;
extern Can2Feedback can2Feedback;
#endif
