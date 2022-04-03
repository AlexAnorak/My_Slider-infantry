#ifndef _VISION_H
#define _VISION_H
#include "board.h"

/**
 * @addtogroup TDT_DEVICE_VISION
 * @{
 */

///是否启用应答模式
#define ANSWER_MODE 0
#pragma pack(1)
///视觉发送结构体（MCU->NUC）.
///以21赛季步兵为例，其他兵种或有更改
struct vision_Send_Struct_t
{
	uint8_t frameHeader;		  ///<0xA5
	float gimbalTimeStamp[3];	  ///<陀螺仪数据
	uint8_t enemyColor : 1;		  ///<0--Red   1--Blue
	uint8_t energyBeatMode : 1;	  ///<0--非打符模式   1--打符模式
	uint8_t baseShootMode : 1;	  ///<同上 基地吊射模式
	uint8_t EnableAutoAim : 1;	  ///<开启自瞄，提醒视觉有跳变时优先选择距离枪口最近的装甲板
	uint8_t SpiningShoot : 1;	  ///<操作手手动判断陀螺
	uint8_t quickSentryShoot : 1; ///<哨兵快速换向
	uint8_t energyBeatCentre : 1; ///<能量机关打中间
	float nominalBulletSpeed;	  ///<标称弹速
	float realBulletSpeed;		  ///<实际弹速
	int8_t visionGunErrorFlag;	  ///<枪口偏置 +- 1 Pitch +-2 Yaw
	/*↓↓↓↓↓↓↓↓↓↓↓custom data start↓↓↓↓↓↓↓↓↓↓↓*/
	/*↑↑↑↑↑↑↑↑↑↑↑ custom data end ↑↑↑↑↑↑↑↑↑↑↑*/
	uint16_t CRC16CheckSum;
};

///视觉接收结构体（NUC->MCU）.
///以21赛季步兵为例，其他兵种或有更改
struct vision_Recv_Struct_t
{
	uint8_t frameHeader; //0xA5
	double recvTime;
	float Yaw;					   ///<单位: 度
	float Pitch;				   ///<单位: 度
	float enemySpeed;
	uint8_t no_Obj : 1;			   ///<是否找到目标
	uint8_t beat : 1;			   ///<是否能击打
	uint8_t unLimitedFireTime : 6; ///<目标与己方枪口几乎相对静止，且之后unLimitedFireTime微秒内都满足
	uint8_t objSpining : 1;		   ///<目标是否在陀螺
	uint8_t isInEnergyBeat : 1;	   ///<当前模式是否为能量机关
	int8_t nowPitchGunError;	   ///<pitch的补偿
	int8_t nowYawGunError;		   ///<yaw轴的补偿
	/*↓↓↓↓↓↓↓↓↓↓↓custom data start↓↓↓↓↓↓↓↓↓↓↓*/
	/*↑↑↑↑↑↑↑↑↑↑↑ custom data end ↑↑↑↑↑↑↑↑↑↑↑*/
	uint16_t CRC16CheckSum;
};
#pragma pack()

struct visionInfo_t
{
	u16 visionCnt;
	u16 visionFPS;
	u8 offlineFlag;
};

extern vision_Recv_Struct_t vision_RecvStruct;
extern vision_Send_Struct_t vision_SendStruct;
extern visionInfo_t visionInfo;

///串口初始化
void Vision_Init(void);

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
