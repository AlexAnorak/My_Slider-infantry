#include "fire_task.h"
#include "dbus.h"
#include "can.h"
#include "vision.h"
#include "state_task.h"

#define FRICTION_CLOSELOOP_INDEX 0
#define FRICTION_OPENLOOP_INDEX 1

#define FRICTION_0mps_INDEX 0
#define FRICTION_15mps_INDEX 1
#define FRICTION_18mps_INDEX 2
#define FRICTION_30mps_INDEX 3

#define FRICTION_A_INDEX 0
#define FRICTION_B_INDEX 1

#define FIRE_INTERVAL_FAST_INDEX 0
#define FIRE_INTERVAL_MID_INDEX 1
#define FIRE_INTERVAL_SLOW_INDEX 2
#define FIRE_INTERVAL_BUFF_INDEX 3

int sprocketErrorText = 0;
int sprocketIntegralErrorTest = 0;
int sprocketIntegralTime = 0;
int sprocketIntegralTimeMax = 5;
int sprocketIntegralErrorOverTimes = 0;

FireCtrl fireCtrl;
Motor *FeedSprocketMotor;
PidParam FeedSprocketPidInner, FeedSprocketPidOuter;

float fireIntervalWatch;

void FireCtrl::fireIntervalSwitch()
{
	if (can1Feedback.MaxHeat == -1 || unLimitedFired == 1)
	{
		fireInterval = fireIntervalSet[FIRE_INTERVAL_FAST_INDEX];
		return;
	}

	switch (fireMode)
	{
	case SILENCE:
		fireInterval = FLT_MAX;
		break;
	case CLEAN_AMMO:
	case FAST_SHOOT:
		fireInterval = fireIntervalSet[FIRE_INTERVAL_FAST_INDEX];
		finalHeatAddition = finalHeatAdditionSet[FIRE_INTERVAL_FAST_INDEX];
		break;
	case MID_SHOOT:
		fireInterval = fireIntervalSet[FIRE_INTERVAL_MID_INDEX];
		finalHeatAddition = finalHeatAdditionSet[FIRE_INTERVAL_MID_INDEX];
		break;
	case SLOW_SHOOT:
		fireInterval = fireIntervalSet[FIRE_INTERVAL_SLOW_INDEX];
		finalHeatAddition = finalHeatAdditionSet[FIRE_INTERVAL_SLOW_INDEX];
		break;
	case BUFF_SHOOT:
		fireInterval = fireIntervalSet[FIRE_INTERVAL_BUFF_INDEX];
		break;
	default:
		break;
	}

	if (jgmtCoolDown + jgmtHeatMax > 200 && fireMode == MID_SHOOT) //爆发优先2级、弹速、冷却优先3级
	{
		fireInterval = 0.06;
		finalHeatAddition = finalHeatAdditionSet[FIRE_INTERVAL_FAST_INDEX];
	}
}

void FireCtrl::frictionSpdSwitch()
{
	if (!startFrictionWheel)
	{
		for (int i = 0; i < FRICTION_COUNT; i++)
		{
			frictionSpdSet[i] = 0;
		}
		return;
	}
	u8 tmpSpeedIndex = getSpeedIndex();

	for (int i = 0; i < FRICTION_COUNT; i++)
	{
		frictionSpdSet[i] = frictionSpdArray[forceOpenLoop][tmpSpeedIndex][i];
	}
}

u8 FireCtrl::getSpeedIndex()
{
	switch (jgmtBulletSpeedMax)
	{
	case 15:
		vision_SendStruct.nominalBulletSpeed = 15;
		return FRICTION_15mps_INDEX;
	case 18:
		vision_SendStruct.nominalBulletSpeed = 18;
		return FRICTION_18mps_INDEX;
	case 30:
		vision_SendStruct.nominalBulletSpeed = 30;
		return FRICTION_30mps_INDEX;
	default:
		vision_SendStruct.nominalBulletSpeed = 15;
		return FRICTION_15mps_INDEX;
	}
	return FRICTION_15mps_INDEX;
}

void FireCtrl::fireIntervalLimit()
{
	if (unLimitedFired)
	{
		fireIntervalExtLimit = 1;
		return;
	}

	float heatLast = jgmtHeatMax - finalHeat; //剩余可消耗热量
	fireIntervalExtLimit = heatLast - 10.5;

	fireIntervalExtLimit = LIMIT((int)fireIntervalExtLimit, 0.00000001, 1);
	fireInterval /= fireIntervalExtLimit;
}

void FireCtrl::heatCalc()
{
	jgmtHeatCalc();

	frictionHeatCalc(); //摩擦轮不准

	sprocketHeatCalc();

	switch (heatChoose)
	{
	case 0:
		finalHeat = frictionHeat + finalHeatAddition;
		return;
	default:
	case 1:
		finalHeat = sprocketHeat;
		return;
	}
}

void FireCtrl::jgmtHeatCalc()
{
	if (!can1Feedback.jgmtOffline)
		jgmtHeat = can1Feedback.Jgmt_Heat;
	else
		jgmtHeat = jgmtHeatMax;
}

void FireCtrl::frictionHeatCalc()
{
	if (thisShootNum.frictionShootNum != lastShootNum.frictionShootNum)
	{
		frictionHeat += 10 * (thisShootNum.frictionShootNum - lastShootNum.frictionShootNum);
	}
	lastShootNum.frictionShootNum = thisShootNum.frictionShootNum;

	frictionHeat -= fireCtrl.jgmtCoolDown * frictionHeatTimer.getCycleT();
	frictionHeat = MAX(frictionHeat, 0);
}

void FireCtrl::sprocketHeatCalc()
{
	if (thisShootNum.sprocketShootNum != lastShootNum.sprocketShootNum)
	{
		sprocketHeat += 10 * (thisShootNum.sprocketShootNum - lastShootNum.sprocketShootNum);
	}
	lastShootNum.sprocketShootNum = thisShootNum.sprocketShootNum;

	sprocketHeat -= fireCtrl.jgmtCoolDown * sprocketHeatTimer.getCycleT();
	sprocketHeat = LIMIT(sprocketHeat, 0, jgmtHeatMax);
}

void FireCtrl::frictionSpdLimit()
{
	if (jgmtBulletSpeed == lastJgmtBulletSpeed)
		return;
	if (lastJgmtBulletSpeed == 0)
	{
		lastJgmtBulletSpeed = jgmtBulletSpeed;
		return;
	}

	u8 speedIndex = getSpeedIndex();
	if (jgmtBulletSpeed > jgmtBulletSpeedMax)
	{
		for (int i = 0; i < FRICTION_COUNT; i++)
		{
			frictionSpdArray[forceOpenLoop][speedIndex][i] -= (jgmtBulletSpeed - jgmtBulletSpeedMax + 1) * (speedIndex + 1) * 15; //速度越高，每次减少的量越多
		}
	}

	lastJgmtBulletSpeed = jgmtBulletSpeed;
}

void FireCtrl::init()
{
	loadParam();

	//has set plan num(default is 1)
	//has set fbValuePtr (inner default is canInfo.speed, outer default is totalEncoder)
	FeedSprocketMotor->pidInner.paramPtr = &FeedSprocketPidInner;
	FeedSprocketMotor->pidOuter.paramPtr = &FeedSprocketPidOuter;
}

void FireCtrl::run()
{
	if (can2Feedback.frictionOfflineCheck == 0)
	{
		can2Feedback.frictionOffline = 1;
	}
	can2Feedback.frictionOfflineCheck = 0;

	//		if(can2Feedback.AS5048_offline)
	//			fireCtrl.forceOpenLoop = 1;

	//		if(++heatMultTimer > 200)
	//		{
	//			heatMult();
	//			heatMultTimer = 0;
	//		}

	fireCtrl.getFireLimit();

	fireCtrl.initFrictionShootNum();

	fireCtrl.heatCalc(); //无论是否脱力，都计算热量

	fireCtrl.valueFb = FeedSprocketMotor->canInfo.totalEncoder;

	fireCtrl.jgmtBulletSpeed = can1Feedback.Jgmt_OutSpd;

	u8 sprocket_friction_ready = 1;
	if (deforceFlag)
	{
		if (!lastDeforceFlag)
		{
			deforceStartTime = getSysTimeUs() / 1e6f; //记录开始脱力的时间
		}
		if (timeIntervalFrom_f(deforceStartTime) > 0.5f) //200ms后自动发0
			for (int i = 0; i < FRICTION_COUNT; i++)
				frictionSpdSet[i] = 0;

		lastDeforceFlag = deforceFlag;
		valueSet = valueFb;
		ammoNowDifferTimes = ammoDifferTimes;
		goto fire_beforeReturn;
	}
	lastDeforceFlag = deforceFlag;

	if (sprocketIntegralTime >= sprocketIntegralTimeMax - 1)
	{
		if (ABS(sprocketIntegralErrorTest) > 40000)
		{
			sprocketIntegralErrorOverTimes++;
		}
		else
		{
			sprocketIntegralErrorOverTimes = 0;
		}
	}

	if (ABS(valueSet - valueFb) > ABS(singleAmmoRota) * 2) // || sprocketIntegralErrorOverTimes > 10)//堵转
	{
		shootErrorFlag++;
		shootErrorCheckTime = 0;
		goto fire_beforeReturn;
	}

	if (!FeedSprocketMotor->canInfo.lostFlag)
	{
		if (lastSprocketOfflineFlag)
			sprocketOfflineTimer = getSysTimeUs() / 1e6f;	 //记录开始脱力的时间
		if (timeIntervalFrom_f(sprocketOfflineTimer) < 0.3f) //200ms后自动发0
			sprocket_friction_ready = 0;
	}
	lastSprocketOfflineFlag = FeedSprocketMotor->canInfo.lostFlag;

	if (!can2Feedback.frictionOffline && !unLimitedFired)
	{
		if (lastFrictionOfflineFlag)
			frictionOfflineTimer = getSysTimeUs() / 1e6f;	 //记录开始脱力的时间
		if (timeIntervalFrom_f(frictionOfflineTimer) < 1.8f) //200ms后自动发0
			sprocket_friction_ready = 0;
	}
	lastFrictionOfflineFlag = can2Feedback.frictionOffline;

	if (!startFrictionWheel)
	{
		if (lastStartFrictionWheel)
		{
			stopFrictionWheelTime = getSysTimeUs() / 1e6f; //记录开始脱力的时间
		}
		if (timeIntervalFrom_f(stopFrictionWheelTime) > 0.3f) //200ms后自动发0
			for (int i = 0; i < FRICTION_COUNT; i++)
				frictionSpdSet[i] = 0;

		lastStartFrictionWheel = startFrictionWheel;
		ammoNowDifferTimes = ammoDifferTimes;
		goto fire_beforeReturn;
	}
	if (!lastStartFrictionWheel && startFrictionWheel)
	{
		startFrictionWheelTime = getSysTimeUs() / 1e6f;
	}
	lastStartFrictionWheel = startFrictionWheel;

	notFireTimer += notFireTimerCycle.getCycleT();

	if (ammoNowDifferTimes < ammoDifferTimes)
	{
		ammoNowDifferTimes++;
		if (ammoNowDifferTimes == ammoDifferTimes)
			valueSet += ammoRotaDir * (singleAmmoRota % ammoDifferTimes); //最后一次补上最后的余数
		else
			valueSet += ammoRotaDir * (singleAmmoRota / ammoDifferTimes);
	}

	fireIntervalSwitch();
	frictionSpdSwitch();

	fireIntervalLimit();
	frictionSpdLimit();

	if (!sprocket_friction_ready)
	{
		valueSet = valueFb;
		ammoNowDifferTimes = ammoDifferTimes;
		goto fire_beforeReturn;
	}

	if (!fireRequirement()) //开火前置条件不满足
		goto fire_beforeReturn;

	if (!fireSource()) //没人喊开火
		goto fire_beforeReturn;

	notFireTimer = 0;

	ammoNowDifferTimes = 0; //开火，重新计算微分次数

	thisShootNum.sprocketShootNum++; //拨弹轮认为打出弹了，拨弹轮进行热量限制

	lastFireTime = getSysTimeUs() / 1e6f;

	fire_beforeReturn:

	if (can1Feedback.Jgmt_Heat > can1Feedback.MaxHeat || fireCtrl.forceOpenLoop)
	{
		fireCtrl.heatChoose = 1; //超热量自动切拨弹轮计算的热量
	}

	fireCtrl.shootErrorProtect();

	fireCtrl.frictionSend();
}

u8 heatMultTimer = 0;
float jgmtHeatNow = 0;
float jgmtHeatLast = 0;
float localHeatNow = 0;
float localHeatLast = 0;
float deltaLocalHeat = 0;
void heatMult()
{
	jgmtHeatNow = can1Feedback.Jgmt_Heat;

	localHeatNow = fireCtrl.sprocketHeat;
	deltaLocalHeat = MIN(0, localHeatNow - localHeatLast);

	if (jgmtHeatLast == jgmtHeatNow)
	{
		localHeatLast = localHeatNow;
		return;
	}

	fireCtrl.sprocketHeat = deltaLocalHeat + jgmtHeatNow;
	localHeatNow = fireCtrl.sprocketHeat;
	jgmtHeatLast = jgmtHeatNow;
	localHeatLast = localHeatNow;
}

void FireCtrl::getFireLimit()
{
	if (!can1Feedback.jgmtOffline)
	{
		fireCtrl.jgmtCoolDown = can1Feedback.CoolRate;
		fireCtrl.jgmtBulletSpeedMax = can1Feedback.shooterId1_17mmSpeedLimit;
		fireCtrl.jgmtHeatMax = can1Feedback.MaxHeat;
	}
	else
	{
		fireCtrl.jgmtCoolDown = stateCtrl.shootCoolRatePerformance[stateCtrl.gimbalPerformanceChoose][stateCtrl.levelChoose];
		fireCtrl.jgmtBulletSpeedMax = stateCtrl.shootSpeedPerformance[stateCtrl.gimbalPerformanceChoose][stateCtrl.levelChoose];
		fireCtrl.jgmtHeatMax = stateCtrl.shootMaxHeatPerformance[stateCtrl.gimbalPerformanceChoose][stateCtrl.levelChoose];
	}
}

void FireCtrl::initFrictionShootNum()
{
	if (fireCtrl.thisShootNum.frictionShootNum == 0 && fireCtrl.lastShootNum.frictionShootNum == 0)
	{
		fireCtrl.lastShootNum.frictionShootNum = can2Feedback.frictionShootNum;
	}
	fireCtrl.thisShootNum.frictionShootNum = can2Feedback.frictionShootNum;
}

void FireCtrl::shootErrorProtect()
{
	sprocketErrorText = fireCtrl.valueSet - fireCtrl.valueFb;
	sprocketIntegralErrorTest += sprocketErrorText;
	sprocketIntegralTime++;
	if (sprocketIntegralTime >= sprocketIntegralTimeMax)
	{
		sprocketIntegralTime = 0;
		sprocketIntegralErrorTest = 0;
	}
	if (!deforceFlag)
	{
		if (fireCtrl.shootErrorFlag && fireCtrl.shootErrorCheckTime < fireCtrl.shootErrorCheckTimeMax)
		{
			fireCtrl.shootErrorCheckTime++;
			FeedSprocketMotor->ctrlSpeed(-1.0f * fireCtrl.ammoRotaDir / fireCtrl.ammoNumPerRound * fireCtrl.shootErrorRetSpd);
			fireCtrl.valueSet = fireCtrl.valueFb;
		}
		else
		{
			fireCtrl.shootErrorFlag = 0;
			FeedSprocketMotor->ctrlPosition(fireCtrl.valueSet);
		}
	}
}

u8 FireCtrl::fireRequirement()
{
	if (FeedSprocketMotor->canInfo.lostFlag) //电机丢失
		return false;

	if (shootErrorFlag) //拨弹轮堵转
		return false;

	if (can2Feedback.frictionOffline && !unLimitedFired) //摩擦轮离线
		return false;

	if (!startFrictionWheel || timeIntervalFrom_f(startFrictionWheelTime) < 2.0f)
		return false;

	if (!fireCtrl.forceOpenLoop && !unLimitedFired)
	{
		for (int i = 0; i < FRICTION_COUNT; i++)
			if (frictionSpdSet[i] == 0) //未开启
				return false;
		//todo
		//		if (!can2Feedback.ready_to_fire) //未准备好
		//			return false;
	}

	if (fireCtrl.forceOpenLoop && !unLimitedFired)
		for (int i = 0; i < FRICTION_COUNT; i++)
			if (frictionSpdSet[i] == 0) //未开启
				return false;

	if (fireInterval > notFireTimer) //冷却时间不够长
		return false;

	return true;
}
#include "state_task.h"

u8 FireCtrl::fireSource()
{
	switch (fireMode)
	{
	case SILENCE:
		return false;
	case CLEAN_AMMO:
		return true;
	case BUFF_SHOOT:
		if (!visionInfo.offlineFlag && !vision_RecvStruct.no_Obj && vision_RecvStruct.beat)
			return true;
		break;
	case FAST_SHOOT:
	case MID_SHOOT:
	case SLOW_SHOOT:
		if (operatorAskFire && !visionAskFire) //相当于只按左键
			return true;
		if (visionAskFire)
		{
			if (RC.Key.SW1 == RCS::SWPos::Up && (stateCtrl.visionDebugType == VDT_ARMOR || stateCtrl.visionDebugType == VDT_BUFF))
			{
				if (!visionInfo.offlineFlag && !vision_RecvStruct.no_Obj && vision_RecvStruct.beat)
				{
					return true;
				}
				return false;
			}
			if (vision_RecvStruct.objSpining && !visionInfo.offlineFlag && !vision_RecvStruct.no_Obj && vision_RecvStruct.beat)
				return true;
			if (operatorAskFire)
				return true;
		}
		break;
	}
	return false;
}

void FireCtrl::frictionSend()
{
	u8 TxData[8];
	TxData[0] = (u8)(frictionSpdSet[0] >> 8);
	TxData[1] = (u8)(frictionSpdSet[0]);
	TxData[2] = (u8)(frictionSpdSet[1] >> 8);
	TxData[3] = (u8)(frictionSpdSet[1]);
	TxData[7] = forceOpenLoop;

	canTx(TxData, CAN2, 0x100);
}

#include "parameter.h"

void FireCtrl::loadParam()
{

	finalHeatAdditionSet[0] = 22;
	finalHeatAdditionSet[1] = 15;
	finalHeatAdditionSet[2] = 15;
	switch (STANDARD_ID)
	{
	case NIRVANA:
		FeedSprocketMotor = new Motor(M2006, CAN2, 0x202);
		FeedSprocketPidInner.kp = 6;
		FeedSprocketPidInner.ki = 0;
		FeedSprocketPidInner.kd = 0;
		FeedSprocketPidInner.resultMax = Motor::getMotorCurrentLimit(M2006);
		FeedSprocketPidInner.integralErrorMax = 100;

		FeedSprocketPidOuter.kp = 0.2;
		FeedSprocketPidOuter.ki = 0;
		FeedSprocketPidOuter.kd = 0;
		FeedSprocketPidOuter.resultMax = Motor::getMotorSpeedLimit(M2006);
		FeedSprocketPidOuter.integralErrorMax = 100;

		fireIntervalExtLimit = 1;
		forceOpenLoop = 0;
		can2Feedback.frictionOffline = 1;
		visionFireAuthority = 1;

		ammoNumPerRound = 8;
		ammoRotaDir = 1;
		singleAmmoRota = 8192 * 36 / ammoNumPerRound;

		fireMode = MID_SHOOT;

		fireInterval = fireIntervalSet[FIRE_INTERVAL_MID_INDEX];

		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_A_INDEX] = 1580;
		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_B_INDEX] = 1580;

		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_A_INDEX] = 1750;
		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_B_INDEX] = 1750;

		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_A_INDEX] = 3200;
		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_B_INDEX] = 3200;

		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_A_INDEX] = 4450;
		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_B_INDEX] = 4450;

		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_A_INDEX] = 4530;
		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_B_INDEX] = 4520;

		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_A_INDEX] = 5200;
		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_B_INDEX] = 5186;
		break;

	case TWONETWONE_CAR:
		FeedSprocketMotor = new Motor(M2006, CAN2, 0x202); //2121
		FeedSprocketPidInner.kp = 6;
		FeedSprocketPidInner.ki = 0;
		FeedSprocketPidInner.kd = 0;
		FeedSprocketPidInner.resultMax = Motor::getMotorCurrentLimit(M2006);
		FeedSprocketPidInner.integralErrorMax = 100;

		FeedSprocketPidOuter.kp = 0.2;
		FeedSprocketPidOuter.ki = 0;
		FeedSprocketPidOuter.kd = 0;
		FeedSprocketPidOuter.resultMax = Motor::getMotorSpeedLimit(M2006);
		FeedSprocketPidOuter.integralErrorMax = 100;

		fireIntervalExtLimit = 1;
		forceOpenLoop = 0;
		can2Feedback.frictionOffline = 1;
		visionFireAuthority = 1;

		ammoNumPerRound = 6;
		ammoRotaDir = -1;
		singleAmmoRota = 8192 * 36 / ammoNumPerRound;

		fireMode = MID_SHOOT;

		fireInterval = fireIntervalSet[FIRE_INTERVAL_MID_INDEX];

		fireCtrl.frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_A_INDEX] = 1550;
		fireCtrl.frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_B_INDEX] = 1550;

		fireCtrl.frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_A_INDEX] = 1800;
		fireCtrl.frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_B_INDEX] = 1800;

		fireCtrl.frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_A_INDEX] = 2700;
		fireCtrl.frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_B_INDEX] = 2700;

		fireCtrl.frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_A_INDEX] = 5060;
		fireCtrl.frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_B_INDEX] = 5060;

		fireCtrl.frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_A_INDEX] = 5156;
		fireCtrl.frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_B_INDEX] = 5156;

		fireCtrl.frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_A_INDEX] = 6200;
		fireCtrl.frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_B_INDEX] = 6200;
		break;

	case AURORA:
		FeedSprocketMotor = new Motor(M2006, CAN2, 0x202);
		FeedSprocketPidInner.kp = 6;
		FeedSprocketPidInner.ki = 0;
		FeedSprocketPidInner.kd = 0;
		FeedSprocketPidInner.resultMax = Motor::getMotorCurrentLimit(M2006);
		FeedSprocketPidInner.integralErrorMax = 100;

		FeedSprocketPidOuter.kp = 0.2;
		FeedSprocketPidOuter.ki = 0;
		FeedSprocketPidOuter.kd = 0;
		FeedSprocketPidOuter.resultMax = Motor::getMotorSpeedLimit(M2006);
		FeedSprocketPidOuter.integralErrorMax = 100;

		fireIntervalExtLimit = 1;
		forceOpenLoop = 0;
		can2Feedback.frictionOffline = 1;
		visionFireAuthority = 1;

		ammoNumPerRound = 8;
		ammoRotaDir = 1;
		singleAmmoRota = 8192 * 36 / ammoNumPerRound;

		fireMode = MID_SHOOT;

		fireInterval = fireIntervalSet[FIRE_INTERVAL_MID_INDEX];

		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_A_INDEX] = 1640;
		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_B_INDEX] = 1640;

		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_A_INDEX] = 1800;
		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_B_INDEX] = 1800;

		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_A_INDEX] = 2850;
		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_B_INDEX] = 2850;

		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_A_INDEX] = 5000;
		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_B_INDEX] = 5000;

		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_A_INDEX] = 5131;
		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_B_INDEX] = 5131;

		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_A_INDEX] = 5700;
		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_B_INDEX] = 5700;
		break;

	case GALAXY:
		FeedSprocketMotor = new Motor(M2006, CAN2, 0x202);
		FeedSprocketPidInner.kp = 6;
		FeedSprocketPidInner.ki = 0;
		FeedSprocketPidInner.kd = 0;
		FeedSprocketPidInner.resultMax = Motor::getMotorCurrentLimit(M2006);
		FeedSprocketPidInner.integralErrorMax = 100;

		FeedSprocketPidOuter.kp = 0.2;
		FeedSprocketPidOuter.ki = 0;
		FeedSprocketPidOuter.kd = 0;
		FeedSprocketPidOuter.resultMax = Motor::getMotorSpeedLimit(M2006);
		FeedSprocketPidOuter.integralErrorMax = 100;

		fireIntervalExtLimit = 1;
		forceOpenLoop = 0;
		can2Feedback.frictionOffline = 1;
		visionFireAuthority = 1;

		ammoNumPerRound = 8;
		ammoRotaDir = 1;
		singleAmmoRota = 8192 * 36 / ammoNumPerRound;

		fireMode = MID_SHOOT;

		fireInterval = fireIntervalSet[FIRE_INTERVAL_MID_INDEX];

		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_A_INDEX] = 1600;
		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_B_INDEX] = 1600;

		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_A_INDEX] = 1790;
		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_B_INDEX] = 1790;

		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_A_INDEX] = 3100;
		frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_B_INDEX] = 3100;

		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_A_INDEX] = 4450;
		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_B_INDEX] = 4440;

		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_A_INDEX] = 4530;
		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_B_INDEX] = 4520;

		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_A_INDEX] = 5200;
		frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_B_INDEX] = 5186;
		break;
	}
}