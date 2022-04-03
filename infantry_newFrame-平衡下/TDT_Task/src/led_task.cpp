/******************************
File name: TDT_Task\src\led_task.cpp
Description: 呼吸灯控制任务
function:
	——————————————————————————————————————————————————————————————————————————
	void Led_Task(void *pvParameters)
	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "led_task.h"
#include "iwdg.h"
#include "vision.h"

//定义板载lED灯对象
Led boardLed = Led(RCC_AHB1Periph_GPIOB , GPIOB , GPIO_Pin_14);
#if defined USE_MAIN_CTRL_2019	
Led	laser = Led(RCC_AHB1Periph_GPIOB , GPIOB, GPIO_Pin_9);
#elif defined USE_MAIN_CTRL_2021_PJ||defined USE_MAIN_CTRL_2021_B	\
	||defined USE_MAIN_CTRL_2021_A||defined USE_MAIN_CTRL_2020
Led laser = Led(RCC_AHB1Periph_GPIOC , GPIOC, GPIO_Pin_8);
#endif

extern u8 deforceFlag;

void ledInit()
{
	/*LED初始化*/
	boardLed.init();
	
	laser.init();
	//设置激光的高低极性
	laser.setLHNagation(0);
	laser.show(0);
}

/**
  * @brief LED任务函数
  * @note 负责LED的控制和喂狗
  * @warning 该函数为重写完成
  */
void Led_Task()
{
	//喂狗
	iwdgFeed();
	//视觉瞄上目标或开启打符模式，关闭激光，防止激光照到装甲板影响视觉

	laser.setError(1,LedES_BlinkFast);
	
	//LED状态展示
	boardLed.stateShow(50);
	laser.stateShow(50);


}






