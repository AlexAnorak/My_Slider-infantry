#include "fast_selfCheck.h"
#include "dbus.h"
#include "motor.h"
#include "led_task.h"
#include "vision.h"
#include "can.h"

FastSelfCheck fastSelfCheck;

void getCanMotorLostId(u8 isCan2)
{
	for (int i = 0; i < 8; i++)
	{
		if (motorList[isCan2][i].motorPoint == 0)
			continue;
		if (motorList[isCan2][i].motorPoint->getEnableMotor() == 0)
			continue;
		if (motorList[isCan2][i].motorPoint->canInfo.lostFlag)
		{
			laser.setError(0, i + 1);
			return;
		}
	}
	laser.setError(0, LedES_BlinkFast);
}

void getOtherOfflineModule()
{
	laser.setError(0, LedES_BlinkFast);
	if (can1Feedback.SuperPowerOffline)
	{
		laser.setError(0, 2);
		return;
	}
	if (can1Feedback.jgmtOffline)
	{
		laser.setError(0, 3);
		return;
	}
	//	if(can2Feedback.AS5048_offline)
	//	{
	//		laser.setError(0,5);
	//		return;
	//	}
}

extern u8 deforceFlag;
void FastSelfCheck::run()
{
	if (!RC.chassisRecv.selfCheck)
		return;

	if (RC.chassisRecv.rcSwitch1 != RC.lastChassisRecv.rcSwitch1)
	{
		lastKeySwitchTime = getSysTimeUs();
		laser.setError(0, LedES_ConstantDark);
	}
	if (timeIntervalFrom(lastKeySwitchTime) < 500000000)
	{
		return;
	}
	if (!deforceFlag)
	{
		laser.setError(0, LedES_BlinkSlow);
		return;
	}
	switch (RC.chassisRecv.rcSwitch1)
	{
	case RCS::Up:
		getCanMotorLostId(0);
		break;
	case RCS::Mid:
		getCanMotorLostId(1);
		break;
	case RCS::Down:
		getOtherOfflineModule();
		break;
	default:
		break;
	}
}