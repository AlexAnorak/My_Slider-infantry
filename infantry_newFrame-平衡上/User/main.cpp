/******************************
  _________               _____       _________ 
 /___  ___/              / ___ \     /___  ___/  
    / /                 / /   \ \       / /     
   / /    _________    / /    / /      / /      
  / /    /___  ___/   / /    / /      / /       
 / /                 / /____/ /      / /        
/_/                 /________/      /_/     


【首次使用请先阅读Readme.md!!】
File name: User/main.cpp
Description: 完成初始化和开始任务的创建，以及打开调度器
function:
	——————————————————————————————————————————————————————————————————————————
	int main(void); //主入口函数
	——————————————————————————————————————————————————————————————————————————
Frame Version: #1.3.3.0113alpha
History: 
	——————————————————————————————————————————————————————————————————————————
	You can get some infomation from "readme.md"
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/

#include "board.h"
#include "cycle.h"
#include "my_task.h"
#include "timer.h"

/**
 * @mainpage TDT_Frame
 * @section mainpage T-DT General Modules
 * 
 * @ref TDT_Frame "点击此处查看电控通用程序框架组成"
 * @image html T-DT.jpg width=100%
 * 
 * 包括LED、电机控制、PID算法、陀螺仪解算算法、遥控器处理等多种算法，方便队员跳过底层代码构建，快速进行上层代码开发
 * @section howToRead 如何阅读此文档
 * @ref TDT_Frame "点击此处查看电控通用程序框架组成"
 * @subsection howToRead_Modules Modules
 * -# 先阅读Detailed Description以及Detailed Description上面的成员
 * -# 如果该模块存在类重要结构体的话，可点击类的链接跳入，阅读类的Reference说明
 * -# 如果该类有继承的话（参考 Inheritance diagram for ...），可参考子类的使用方法
 * @subsection howToRead_Modules_eg Modules example
 * -# @ref TDT_Motor "电机处理"
 *    下滚到Detailed Description
 * -# @ref Motor "电机类"
 */

/**
 * @defgroup TDT_Frame T-DT General Modules
 * T-DT 通用控制模组
 * 包括LED、电机控制、PID算法、陀螺仪解算算法、遥控器处理等多种算法，方便队员跳过底层代码构建，快速进行上层代码开发
 */

/**
 * @defgroup TDT_ALG 算法
 * @ingroup TDT_Frame
 * T-DT 通用算法
 * 包括PID算法、陀螺仪解算算法、麦轮等多种算法
 */

/**
 * @defgroup TDT_DEVICE 设备
 * @ingroup TDT_Frame
 * T-DT 通用设备
 * 包括LED、电机控制、遥控器处理等多种算法
 */

/**
 * @defgroup TDT_BSP 板载外设支持
 * @ingroup TDT_Frame
 * T-DT 板载外设支持
 * 包括CAN、SPI、软硬件IIC等板载外设支持
 */

/*通用任务的宏定义在 my_task.h 下*/
int main(void)
{
	/*中断分组*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    /*滴答定时器配置*/
    sysTickInit();
	/*初始化TIM2，便于统计CPU占用率*/
	ConfigureTimeForRunTimeStats();
	boardALLInit();
	while(1){}
}

