//parameter.cpp
/******************************
File name: TDT_Alg\src\parameter.cpp
Description: switch infantry
function:
	——————————————————————————————————————————————————————————————————————————
	void TDT_ParametersInitSet()
	——————————————————————————————————————————————————————————————————————————
	void TDT_Get_PIDparameters(pid* pidStruct, u8 pidIndex)
	——————————————————————————————————————————————————————————————————————————
	void Struct_Init()
	——————————————————————————————————————————————————————————————————————————
Author: 祖传
Version:
Date: 21.5.31
History: 
****************************  */
#include "parameter.h"
#include "flash_var.h"
#include "led_task.h"

u8 STANDARD_ID = 0;			  //参数加载类型NONE-0；SHIP_CAR-1；TWONETWONE_CAR-2；Gamma-3；Delta-4
u8 UseDef = 0;				  //使用默认数据，重写Flash[0:使用代码默认但不重写主Flash；1:使用代码默认且重写主Flash；]
#define CreatMark NONE	  //为主控打上标记
#define EmergencyForceID NIRVANA //强制指定主控ID，强制指定直接跳过flash，不重写

void TDT_ParametersInitSet()
{
#if !EmergencyForceID
#if CreatMark
	u8 creatmark = CreatMark;
	IFlash.link(creatmark, 1);
	IFlash.save();
	boardLed.setError(1, LedES_BlinkSlow);
	laser.setError(1, LedES_BlinkSlow);
	while (1)
	{
		boardLed.stateShow(25);
		laser.stateShow(25);
		delayMs(25);
	}
#else
	IFlash.link(STANDARD_ID, 1);
	IFlash.read();
	if (STANDARD_ID == 0 || STANDARD_ID == 0xFF)
	{
		boardLed.setError(1, 1);
		laser.setError(1, 1);
		while (1)
		{
			boardLed.stateShow(25);
			laser.stateShow(25);
			delayMs(25);
		}
	}
#endif
#else
	STANDARD_ID = EmergencyForceID;
#endif
}
