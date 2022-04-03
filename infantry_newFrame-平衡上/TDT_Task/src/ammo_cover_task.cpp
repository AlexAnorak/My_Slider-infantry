#include "ammo_cover_task.h"
#include "motor.h"
#include "chassis_task.h"
#include "dbus.h"
PidParam ammoCoverParamInner = {3};
PidParam ammoCoverParamOuter = {0.15};
AmmoCover ammoCover;
#define AMMO_COVER_OFFSET_SPD_MAX 2000
#define AMMO_COVER_OFFSET_CURRENT_MAX 2000

u16 AMMO_COVER_ERROR_THESHOLD = 5;
u8 OFFSET_TIME_MAX = 100;

u8 AmmoCover::calOffset()
{
	if (offsetTimes < OFFSET_TIME_MAX)
	{
		ammoCoverParamInner.resultMax = AMMO_COVER_OFFSET_CURRENT_MAX;
		ammoCoverParamOuter.resultMax = AMMO_COVER_OFFSET_SPD_MAX;
		ammoCoverMotor->ctrlSpeed(AMMO_COVER_OFFSET_SPD_MAX);
		if (ammoCoverMotor->canInfo.lostFlag != 1 && 
			ABS(ammoCoverMotor->canInfo.totalEncoder - ammoCoverMotor->canInfo.lastTotalEncoder) < AMMO_COVER_ERROR_THESHOLD)
		{
			coverOffset = ammoCoverMotor->canInfo.totalEncoder;
			offsetTimes++;
		}
		else
		{
			offsetTimes = 0;
		}
		return false;
	}
	offsetTimes = 0;
	return true;
}

void AmmoCover::init()
{
	ammoCover.LoadParam();
	ammoCover.ammoCoverMotor->pidOuter.setPlanNum(1);
	ammoCover.ammoCoverMotor->pidInner.setPlanNum(1);

	coverRota = -100000;
	coverOffsetWrong = 1;
	coverOffseting = 1;
	coverOpen = 0;
	ammoCoverMotor->pidInner.paramPtr = &ammoCoverParamInner;
	ammoCoverMotor->pidOuter.paramPtr = &ammoCoverParamOuter;
}

void AmmoCover::run()
{
	if (chassis.customFlag == 0 && chassis.judgeIfMoving() && ammoCover.coverOpen)
	{
		ammoCover.coverOpen = 0;
	}

	if (deforceFlag)
	{
		ammoCoverMotor->pidOuter.Clear();
		ammoCoverMotor->pidInner.Clear();
		offsetTimes = 0;
		coverOffsetStart();
		return;
	}

	if (coverOffseting || coverOffsetWrong)
	{
		if (calOffset())
		{
			coverOffseting = 0;
			coverOffsetWrong = 0;
			coverOpen = 0;
		}
		return;
	}

	if (coverOpen)
	{
		ammoCoverMotor->ctrlPosition(coverOffset + coverRota);
	}
	else
	{
		ammoCoverMotor->ctrlPosition(coverOffset);
	}
}

#include "parameter.h"

void AmmoCover::LoadParam()
{
	switch(STANDARD_ID)
	{
		case NIRVANA:
			ammoCoverMotor = new Motor(M2006, CAN1, 0x201);
		break;
		case TWONETWONE_CAR:
			ammoCoverMotor = new Motor(M2006, CAN2, 0x201);
		break;
		case AURORA:
			ammoCoverMotor = new Motor(M2006, CAN2, 0x201);
			coverRota = -99000;
		break;
		case GALAXY:
			ammoCoverMotor = new Motor(M2006, CAN2, 0x201);
			coverRota = -99000;
		break;
	}
}