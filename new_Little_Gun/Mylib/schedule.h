#ifndef _SCHEDULE_H
#define _SCHEDULE_H

#include "stm32f10x.h"


typedef struct _schedule {
    uint16_t   cnt_1ms;
    uint16_t   cnt_2ms;
    uint16_t   cnt_5ms;
    uint16_t   cnt_10ms;
    uint16_t   cnt_20ms;
    uint16_t   cnt_50ms;
    uint16_t   cnt_100ms;
    uint64_t   sysTime_ms;
    uint64_t   beatStart_ms;
} _schedule;

typedef struct _state 
{
    int16_t ShootNum;			//发射子弹数量
	float ShootNum_f;			//发射子弹数量（临时）
	int16_t OldShootNum;		//上一次子弹数量
	int16_t Standby_PWM;		//待机（开环）时占空比
	int16_t StartingTime;		//开始时间计数器
	int16_t UnlockTime;			//解锁时间计数器
	u8 INFANTRY_ID; //ID
	u8 paraLoadFlag; //是否加载PID
	u8 last_INFANTRY_ID;

	int16_t SN_now_spd_A;		//当前磁编码器反馈速度
	int16_t SN_last_spd_A;		//上一次磁编码器反馈速度
	int16_t SN_start_spd_A;		//上一次稳定时的磁编码器反馈速度
	int16_t SN_error_spd_A;		//磁编码器反馈速度差

	int16_t SN_now_spd_B;		//当前磁编码器反馈速度
	int16_t SN_last_spd_B;		//上一次磁编码器反馈速度
	int16_t SN_start_spd_B;		//上一次稳定时的磁编码器反馈速度
	int16_t SN_error_spd_B;		//磁编码器反馈速度差

	
	/****强制开环时用到的变量****/
	int16_t ForceOpenloop_setspeed_A;//强制开环设定值
	int16_t ForceOpenloop_setspeed_B;//强制开环设定值
	/****开环时用到的变量****/
	int16_t openloop_setspeed_A;//开环设定值
	int16_t openloop_setspeed_B;//开环设定值
	/****主控离线时用到的变量****/
	int16_t offline_setspeed_A;//主控离线设定值
	int16_t offline_setspeed_B;//主控离线设定值
	/****磁编码器离线开环用到的变量****/
	int16_t AS5048_Offline_setspeed_A;//磁编码器离线时开环的设定值
	int16_t AS5048_Offline_setspeed_B;//磁编码器离线时开环的设定值
	//闭环时用到的变量
	int16_t MaxSetSpd_A;		//闭环最大设定值
	int16_t MaxSetSpd_B;		//闭环最大设定值
	int16_t OpenLoopMaxSetSpd_A;		//开环最大设定值
	int16_t OpenLoopMaxSetSpd_B;		//开环最大设定值
	int32_t offline_check;		//主控离线检查

	/****状态标志位****/
	u8 ReadyToFire;				//允许开火
	u8 ReadyToStart;			//摩擦轮已经准备好
	u8 WaitReadyToUnlock;		//等到摩擦轮转速稳定才解除拨盘锁的标志位
	u8 EnableQuickStart;		//允许快速启动
	u8 ForceOpenLoop;			//强制开环
	u8 OpenLoop;				//开环状态
	u8 StopState;				//停止标志位（设定值为0时置1，切入闭环时置0）
	u8 NewStart;				//开始标志位（切入闭环时置1，进入pid计算清0，用于清空pid）
	u8 OpenLoopMaxSetSpdLimit;	//是否开启开环速度限制（调试用时清零，用于给开环速度限幅）
	
	u8 AS5048_A_Offline;		//磁编码器离线标志位
	u8 AS5048_B_Offline;		//磁编码器离线标志位
} _state;

void Loop_1000Hz(void); //1ms执行一次
void Loop_500Hz(void);	//2ms执行一次
void Loop_200Hz(void);	//5ms执行一次
void Loop_100Hz(void);	//10ms执行一次
void Loop_50Hz(void);	  //20ms执行一次
void Loop_20Hz(void);	  //50ms执行一次
void Loop_10Hz(void);	  //100ms执行一次

void Loop(void);

extern struct _schedule schedule;
extern struct _state state;

#endif
