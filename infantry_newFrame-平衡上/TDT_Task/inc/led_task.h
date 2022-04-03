/*****************************************************************************
File name: TDT_Task\src\led_task.h
Description: 呼吸灯控制任务
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __LED_TASK_H__
#define __LED_TASK_H__

#include "board.h"
#include "led.h"
extern Led boardLed;
extern Led laser;

void ledInit();
void Led_Task();
#endif
