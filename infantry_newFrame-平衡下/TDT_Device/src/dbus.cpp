/******************************
File name: TDT_Device\src\dbus.cpp
Description: 遥控器接收机
function:
	——————————————————————————————————————————————————————————————————————————
	void Dbus_Config(void)
	——————————————————————————————————————————————————————————————————————————
	void USART2_IRQHandler(void)
	——————————————————————————————————————————————————————————————————————————
	static void Handle_data(volatile const uint8_t *sbus_buf, struct _RC *rc_ctrl)
	——————————————————————————————————————————————————————————————————————————
	int Get_Keypress(uint16_t Key)
	——————————————————————————————————————————————————————————————————————————
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.18 肖银河-增加切换为原始数据处理的变量
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次记录
	——————————————————————————————————————————————————————————————————————————
****************************  */
#define __DBUS_DRIVER_GLOBALS
#include "dbus.h"
#include "board.h"
#include "string.h"
#include "task_virtual.h"
#include "KeyProcess.h"
#include "state_task.h"
#include "fast_selfCheck.h"

/**
 * @ingroup TDT_DEVICE
 * @defgroup DBUS 遥控器解算
 * @brief 该类提供了遥控器的数据解算，以及常用的重要标志位
 */
_RC RC;
uint8_t deforceFlag = 0;
uint8_t dbusOnlineFlag = 0;

void _RC::init()
{
	deforceFlag = 1;
	chassisRecv.deforceFlag = 1;
}

///@sa VirtualTask
u8 _RC::run_1000Hz()
{
	if(updated)
	{
		updated = 0;
		dbusOnlineFlag = 1;
		memcpy(&lastChassisRecv, &chassisRecv, sizeof(chassisRecv));
		memcpy(&chassisRecv, rcDataFromCan, sizeof(chassisRecv));
		deforceFlag = chassisRecv.deforceFlag;
		if (deforceFlag && chassisRecv.resetFlag)
		{
			__disable_irq();	//关闭所有中断
			NVIC_SystemReset(); //复位
			while (1)
			{
			} //仅等待复位
		}
		return true;
	}
	if (timeIntervalFrom(updateTime) > 50000) //500ms
	{
		deforceFlag = 1;
		memset(&chassisRecv, 0, sizeof(chassisRecv));
		chassisRecv.deforceFlag = 1;
		/*离线检测*/
		if (dbusOnlineFlag == 1)//后面置一代表相当于离线跳变
		{
			//清除旧数据
			chassisRecv.deforceFlag = 1;
			deforceFlag = 1;
		}
		dbusOnlineFlag = 0;
	}
	return false;
}

