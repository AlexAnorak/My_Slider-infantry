/*****************************************************************************
File name: TDT_Task\src\cylce.h
Description: 用于检测两次调用之间的时间
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.16 改写Get_Cycle_T函数，省略初始化后，第一次调用返回值为零，修改入参
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次记录
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef _CYCLE_H_
#define _CYCLE_H_

#include "board.h"
/**
 * @addtogroup TDT_SCHEDULE_API
 * @brief 提供了获取准确时间的API
 */
class Cycle{
private:
	uint64_t cycleTime[3];
public:
	Cycle()
	{
		cycleTime[0]=0;
		cycleTime[1]=0;
		cycleTime[2]=0;
		getCycleT();	//设定开始时间
	}
	float getCycleT();
};



#endif
