#include "schedule.h"
#include "dbus.h"
#include "imu_task.h"
#include "led_task.h"
#include "judgement.h"
#include "ammo_cover_task.h"
#include "chassis_task.h"
#include "fast_selfCheck.h"
#include "fire_task.h"
#include "gimbal_task.h"
#include "autoIntoStation.h"
#include "state_task.h"
#include "motor.h"
#include "usart.h"
#include "vision.h"

///1ms执行一次
void TDT_Loop_1000Hz(void) 
{
	//遥控器计算函数
	RC.run_1000Hz();
#if USE_JUDGEMENT
	//裁判系统信息获取函数，步兵的裁判系统有专门模块读取
	ringQueue();
#endif
}

///2ms执行一次
void TDT_Loop_500Hz(void) 
{
	//陀螺仪任务执行函数
	Imu_Task();
	//云台任务执行函数
	gimbal.run();
	//底盘任务函数
	chassis.run();
	//电机发送函数
	Motor::sendCanMsg();
	//状态控制任务
	State_Ctrl();
	//车钥匙数据发送
	Communicate_Update();
}

///5ms执行一次
void TDT_Loop_200Hz(void) 
{
	//自动进补给站函数
	intoStation.run();
	//弹仓盖任务函数
	ammoCover.run();
	//快速自检任务函数，用于检测各个电机是否离线
	fastSelfCheck.run();
	//开火控制任务函数
	fireCtrl.run();
	
	void sendCanStateMsg();
	sendCanStateMsg();

}

///10ms执行一次
void TDT_Loop_100Hz(void) 
{
}

///20ms执行一次
void TDT_Loop_50Hz(void)
{
}

///50ms执行一次
void TDT_Loop_20Hz(void) 
{
	//LED任务运行函数
	Led_Task();
}

///100ms执行一次
void TDT_Loop_10Hz(void) 
{
}

///500ms执行一次
void TDT_Loop_2Hz(void) 
{
}

///1000ms执行一次
void TDT_Loop_1Hz(void) 
{
}

///轮询执行函数，并且通过获取每个周期函数的运行时间计算占用率
///@note 占用率计算方法：∑(周期函数运行的时间(us)*周期函数运行的频率(Hz))/1e6f
void TDT_Loop(struct _Schedule *robotSchedule)
{
	uint64_t startTimeStamp = getSysTimeUs();
	TDT_Loop_1000Hz();
	robotSchedule->runTime_1ms = getSysTimeUs() - startTimeStamp;
	
	if (robotSchedule->cnt_2ms >= 2)
	{
		robotSchedule->cnt_2ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_500Hz();
		robotSchedule->runTime_2ms = getSysTimeUs() - startTimeStamp;
	}
	if (robotSchedule->cnt_5ms >= 5)
	{
		robotSchedule->cnt_5ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_200Hz();
		robotSchedule->runTime_5ms = getSysTimeUs() - startTimeStamp;
	}
	if (robotSchedule->cnt_10ms >= 10)
	{
		robotSchedule->cnt_10ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_100Hz();
		robotSchedule->runTime_10ms = getSysTimeUs() - startTimeStamp;
	}
	if (robotSchedule->cnt_20ms >= 20)
	{
		robotSchedule->cnt_20ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_50Hz();
		robotSchedule->runTime_20ms = getSysTimeUs() - startTimeStamp;
	}
	if (robotSchedule->cnt_50ms >= 50)
	{
		robotSchedule->cnt_50ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_20Hz();
		robotSchedule->runTime_50ms = getSysTimeUs() - startTimeStamp;
	}
	if (robotSchedule->cnt_100ms >= 100)
	{
		robotSchedule->cnt_100ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_10Hz();
		robotSchedule->runTime_100ms = getSysTimeUs() - startTimeStamp;
	}
	if (robotSchedule->cnt_500ms >= 500)
	{
		robotSchedule->cnt_500ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_2Hz();
		robotSchedule->runTime_500ms = getSysTimeUs() - startTimeStamp;
	}
	if (robotSchedule->cnt_1000ms >= 1000)
	{
		robotSchedule->cnt_1000ms = 0;
		uint64_t startTimeStamp = getSysTimeUs();
		TDT_Loop_1Hz();
		robotSchedule->runTime_1000ms = getSysTimeUs() - startTimeStamp;
	}
	
	robotSchedule->CPU_usage = (robotSchedule->runTime_1ms*1000 + 
								robotSchedule->runTime_2ms*500 + 
								robotSchedule->runTime_5ms*200 + 
								robotSchedule->runTime_10ms*100 + 
								robotSchedule->runTime_20ms*50 + 
								robotSchedule->runTime_50ms*20 + 
								robotSchedule->runTime_100ms*10 + 
								robotSchedule->runTime_500ms*2 + 
								robotSchedule->runTime_1000ms)/1e6f;
}
