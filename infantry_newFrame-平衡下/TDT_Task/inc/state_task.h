#ifndef __STATE_TASK_H
#define __STATE_TASK_H

#include "board.h"

/***visionDebugType设置***/
#define VDT_ARMOR 0//装甲板
#define VDT_BUFF 1//打符
#define VDT_BASE_SHOOT 2//基地吊射
#define VDT_AUTO_GET_AMMO 3//补给站自动对位
#define VDT_AUTO_GO_AMMO 4//自动进补给站

class StateCtrl
{
public:
	u8 lastDeforceFlag;
	u8 visionDebugType = VDT_ARMOR;
	float autoGetAmmoFromTime;//从什么时候开始自动对位
	float rightClickFromTimeFirst;//从什么时候第一次按下右键
	float rightClickFromTimeNext;//从什么时候第一次按下右键
	vec3f customeSpeed = {600.0f,600.0f,300.0f};
	u8 levelChoose=0;
	u8 chassisPerformanceChoose=0;
	u8 gimbalPerformanceChoose=0;
	u8 chassisPowerPerformance[3][3] = {{45,50,55}, {60,80,100},{60,80,100}};
	u16 chassisBloodPerformance[3][3] = {{200,300,400},{150,200,250},{300,400,500}};
	u8 shootSpeedPerformance[3][3] = {{15,15,15},{15,18,18},{30,30,30}};
	u8 shootCoolRatePerformance[3][3]={{15,25,35},{40,60,80},{15,25,35}};
	u16 shootMaxHeatPerformance[3][3]={{150,280,400},{50,100,150},{75,150,200}};
	bool getAmmoIng = 0;
	
	void changeVisionError();
	int8_t nowVisionError_Pitch,nowVisionErrorNext_Pitch;
	int8_t nowVisionError_Yaw,nowVisionErrorNext_Yaw;
	int8_t addYawFlag,addPitchFlag,addYawFlag_last,addPitchFlag_last;
	uint8_t stateStructFromCan[8];
};
extern StateCtrl stateCtrl;

struct CanStateStruct
{
	uint8_t PowerPath_Switch:1;	    //是否开启电容
	uint8_t Check_Mode:1;//检录模式
	uint8_t ULTS_Mode:1;//特殊模式
	u32 frictionSpdA:6;
	u32 frictionSpdB:6;
	u32 visionOffline:1;
	u32 frictionOffline:1;
	u32 forceOpenloop:1;
	u32 unLimitedFired:1;
	u32 ammoCoverOpen:1;
	u32 blockError:1;
	u32 visionLock:1;
	u32 visionBeat:1;
	u32 gimbalMode:2;
	u32 localHeat:9;
	u32 heatChoose:1;//0为摩擦轮，1为拨弹轮
	u32 sprocketMotorOffline:1;//拨弹轮离线
	u32 VisionMode:2;
};

#pragma pack(1)
struct PowerInfoMsg
{
	float chassis_power;
	uint16_t chassis_power_buffer;
	uint16_t max_chassis_power;
};
struct PowerRecvMsg
{
	uint16_t capacitance_percentage;//电容能量百分比
	uint16_t maxCurrent;//最大电流
};
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
extern PowerInfoMsg powerInfoMsg;
extern PowerHeatData powerHeatData;
extern JgmtInfoMsg jgmtInfoMsg;
extern PowerRecvMsg powerRecvMsg;
extern struct CanStateStruct canStateStruct;

void State_Ctrl();

void State_Ctrl_RC_Info();

#endif