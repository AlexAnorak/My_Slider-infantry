#ifndef _SCHEDULE_H
#define _SCHEDULE_H

#include "board.h"

///@addtogroup TDT_SCHEDULE_API
///@{
typedef struct _Schedule
{
	///运行频率计数
	///@{
	uint16_t cnt_2ms;
	uint16_t cnt_5ms;
	uint16_t cnt_10ms;
	uint16_t cnt_20ms;
	uint16_t cnt_50ms;
	uint16_t cnt_100ms;
	uint16_t cnt_500ms;
	uint16_t cnt_1000ms;
	
	///@}
	///CPU占用率(相对)
	float CPU_usage;
	///运行时间统计, 单位微秒
	///@{
	uint32_t runTime_1ms;
	uint32_t runTime_2ms;
	uint32_t runTime_5ms;
	uint32_t runTime_10ms;
	uint32_t runTime_20ms;
	uint32_t runTime_50ms;
	uint32_t runTime_100ms;
	uint32_t runTime_500ms;
	uint32_t runTime_1000ms;
	///@}
} Schedule;

void TDT_Loop_1000Hz(void); ///<1ms执行一次
void TDT_Loop_500Hz(void);	///<2ms执行一次
void TDT_Loop_200Hz(void);	///<5ms执行一次
void TDT_Loop_100Hz(void);	///<10ms执行一次
void TDT_Loop_50Hz(void);	///<20ms执行一次
void TDT_Loop_20Hz(void);	///<50ms执行一次
void TDT_Loop_10Hz(void);	///<100ms执行一次
void TDT_Loop_2Hz(void);	///<500ms执行一次
void TDT_Loop_1Hz(void);	///<1000ms执行一次

#ifdef __cplusplus
extern "C"
{
#endif
	void TDT_Loop(struct _Schedule *robotSchedule);
#ifdef __cplusplus
}
#endif

extern Schedule scheduleLoop;

///@}
#endif
