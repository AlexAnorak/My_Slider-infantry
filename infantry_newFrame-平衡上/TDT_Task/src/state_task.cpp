#include "state_task.h"
#include "ammo_cover_task.h"
#include "gimbal_task.h"
#include "fire_task.h"
#include "chassis_task.h"
#include "vision.h"
#include "dbus.h"
#include "can.h"
#include "imu_task.h"
#include "autoIntoStation.h"
#include "parameter.h"

extern Can2Feedback can2Feedback;
StateCtrl stateCtrl;
CanStateStruct canStateStruct;
ChassisSend chassisSend;
PowerInfoMsg powerInfoMsg;
PowerRecvMsg powerRecvMsg;

void sendChassisSend()
{
	canTx((u8*)&chassisSend, CAN1, 0x150);
}

void sendCanStateMsg()
{
	canTx((u8*)&canStateStruct, CAN1, 0x170);
}

void StateCtrl::autoGetAmmo()
{
	/*判断是否需要提前退出自动对位*/
	if (ABS(chassis.remoteSpeed.data[0]) > 0 || ABS(chassis.remoteSpeed.data[0]) > 0 || ABS(chassis.keyboardSpeed.data[0]) > 0 || ABS(chassis.keyboardSpeed.data[0]) > 0 || chassis.rotateFlag || chassis.swingFlag)
	{
		autoGetAmmoFromTime = 0;
		ammoCover.setCoverOpen(0);
		gimbal.desireMode = gimbal.NormalMode;
		stateCtrl.getAmmoIng = 0;
		chassis.customFlag = 0;
		chassis.ifChassisFollow = 1; //切回底盘正常模式
	}
	//todo config timer interval
	if (autoGetAmmoFromTime == 0)
	{
		ammoCover.setCoverOpen(1);
		autoGetAmmoFromTime = getSysTimeUs() / 1e6f;
	}
	if (timeIntervalFrom_f(autoGetAmmoFromTime) < 1.2)
	{
		chassis.customSpeedIn.data[0] = 0;
		chassis.customSpeedIn.data[1] = -customeSpeed.data[1]; //左
		return;
	}
	if (timeIntervalFrom_f(autoGetAmmoFromTime) < 2.4)
	{
		chassis.customSpeedIn.data[0] = customeSpeed.data[0]; //前
		chassis.customSpeedIn.data[1] = 0;
		return;
	}
	if (timeIntervalFrom_f(autoGetAmmoFromTime) < 3.3)
	{
		chassis.customSpeedIn.data[0] = -customeSpeed.data[0] * 0.65; //0.5f在1921车认为大
		chassis.customSpeedIn.data[1] = customeSpeed.data[1] * 0.65;
		return;
	}
	/*走完一遍退出*/
	autoGetAmmoFromTime = 0;

	gimbal.desireMode = gimbal.NormalMode;
	stateCtrl.getAmmoIng = 0;
	chassis.customFlag = 0;
	chassis.ifChassisFollow = 1; //切回底盘正常模式
}

void StateCtrl::changeVisionError()
{
	nowVisionError_Pitch = vision_RecvStruct.nowPitchGunError;
	nowVisionError_Yaw = vision_RecvStruct.nowYawGunError;
	if (addYawFlag != addYawFlag_last)
	{
		nowVisionErrorNext_Yaw = nowVisionError_Yaw + LIMIT(addYawFlag, -1, 1);
	}

	if (nowVisionError_Yaw == nowVisionErrorNext_Yaw)
	{
		addYawFlag = 0;
		vision_SendStruct.visionGunErrorFlag = addYawFlag * 1;
	}

	if (addPitchFlag != addPitchFlag_last)
	{
		nowVisionErrorNext_Pitch = nowVisionError_Pitch + LIMIT(addPitchFlag, -1, 1);
	}

	if (nowVisionError_Pitch == nowVisionErrorNext_Pitch)
	{
		addPitchFlag = 0;
		vision_SendStruct.visionGunErrorFlag = addPitchFlag * 2;
	}
	addYawFlag_last = addYawFlag;
	addPitchFlag_last = addPitchFlag;
	if (addYawFlag)
		vision_SendStruct.visionGunErrorFlag = addYawFlag * 1;
	else if (addPitchFlag)
		vision_SendStruct.visionGunErrorFlag = addPitchFlag * 2;
	else
		vision_SendStruct.visionGunErrorFlag = 0;
}

void enterBuff()
{
//	if (STANDARD_ID == GALAXY) //银河不需要此案件
		chassis.lockChassis = 1;
	gimbal.desireMode = gimbal.buffMode;
	fireCtrl.fireMode = fireCtrl.BUFF_SHOOT;
	vision_SendStruct.energyBeatMode = 1;
	visionSendYaw = &boardImu->Angle.yaw;
	visionSendPitch = &boardImu->Angle.pitch;
}

void quitBuff()
{
//	if (STANDARD_ID == GALAXY) //银河不需要此案件
		chassis.lockChassis = 0;
	gimbal.desireMode = gimbal.NormalMode;
	vision_SendStruct.energyBeatMode = 0;
	fireCtrl.fireMode = fireCtrl.MID_SHOOT;
	visionSendYaw = &boardImu->Angle.yaw;
	visionSendPitch = &boardImu->Angle.pitch;
}

void enterAutoGoAmmo()
{
	intoStation.allreadyCnt = 0;
	intoStation.haveInThere = 0;
	intoStation.intoStationFalg = 1;
	chassis.customFlag = 1; //启用自定义模式
}

void quitAutoGoAmmo()
{
	intoStation.intoStationFalg = 0;
	chassis.customFlag = 0;
}
#include "fast_selfCheck.h"

void State_Ctrl()
{
	chassisSend.deforceFlag = deforceFlag;
	chassisSend.keyCtrl = RC.Key.CTRL;
	chassisSend.keyShift = RC.Key.SHIFT;
	chassisSend.resetFlag = false;
	chassisSend.selfCheck = fastSelfCheck.selfCheckingFlag;
	chassisSend.rcSwitch1 = RC.Key.SW1;
	chassisSend.rcSwitch2 = RC.Key.SW2;
	chassisSend.zeroYaw = 0;

	stateCtrl.lastDeforceFlag = deforceFlag;
	
	canStateStruct.PowerPath_Switch = chassis.powerParam.PowerPath_Switch;
	canStateStruct.Check_Mode = chassis.powerParam.checkMode;
	canStateStruct.ULTS_Mode = chassis.powerParam.ULTSMode;
	canStateStruct.forceOpenloop = fireCtrl.forceOpenLoop;
	canStateStruct.frictionOffline = can2Feedback.frictionOffline;
	canStateStruct.frictionSpdA = can2Feedback.Snail_A_FeedbackSpd_Now / 100;
	canStateStruct.frictionSpdB = can2Feedback.Snail_B_FeedbackSpd_Now / 100;
	canStateStruct.unLimitedFired = fireCtrl.unLimitedFired;
	canStateStruct.visionLock = !vision_RecvStruct.no_Obj;
	canStateStruct.visionBeat = vision_RecvStruct.beat;
	canStateStruct.localHeat = fireCtrl.finalHeat;
	canStateStruct.visionOffline = visionInfo.offlineFlag;
	canStateStruct.ammoCoverOpen = ammoCover.coverOpen;
	canStateStruct.blockError = (fireCtrl.shootErrorFlag && fireCtrl.shootErrorCheckTime < fireCtrl.shootErrorCheckTimeMax);
	canStateStruct.heatChoose = fireCtrl.heatChoose;
	canStateStruct.sprocketMotorOffline = FeedSprocketMotor->canInfo.lostFlag;
	if (gimbal.desireMode > gimbal.baseShootMode)
	{
		canStateStruct.gimbalMode = gimbal.desireMode - 1;
	}
	else
	{
		canStateStruct.gimbalMode = gimbal.desireMode;
	}

	if (gimbal.desireMode == gimbal.buffMode) //打符
	{
		canStateStruct.VisionMode = 1;
	}
	else if (vision_SendStruct.SpiningShoot) //小陀螺
	{
		canStateStruct.VisionMode = 2;
	}
	else if (vision_SendStruct.quickSentryShoot) //哨兵快速换向
	{
		canStateStruct.VisionMode = 3;
	}
	else
		canStateStruct.VisionMode = 0;

	sendChassisSend();
}

void State_Ctrl_RC_Info()
{
	can1Feedback.jgmtOfflineCheck++;
	if (can1Feedback.jgmtOfflineCheck > 10)
	{
		can1Feedback.jgmtOfflineCheck = 10;
		can1Feedback.jgmtOffline = 1;
	}
	else
	{
		can1Feedback.jgmtOffline = 0;
	}

	can1Feedback.SuperPowerOfflineCheck++;
	if (can1Feedback.SuperPowerOfflineCheck > 10)
	{
		can1Feedback.SuperPowerOfflineCheck = 10;
		can1Feedback.SuperPowerOffline = 1;
	}
	else
	{
		can1Feedback.SuperPowerOffline = 0;
	}

	if (!deforceFlag && stateCtrl.lastDeforceFlag)
	{
		ammoCover.setCoverOpen(0);
		gimbal.desireMode = gimbal.NormalMode;
		fireCtrl.fireMode = fireCtrl.MID_SHOOT;
		fireCtrl.operatorAskFire = 0;
		fireCtrl.visionAskFire = 0;
	}

	stateCtrl.changeVisionError();
	u8 tmpOperatorAskFire = 0; //临时变量记录，相当于下方条件只要有一个置一
	u8 tmpVisionAskFire = 0;   //临时变量记录，相当于下方条件只要有一个置一
	u8 tmpEnableAutoAim = 0;
	u8 tmpStartFrictionWheel = 0;

	if (stateCtrl.getAmmoIng)
	{
		stateCtrl.autoGetAmmo(); //运行结束后自动退出，云台为补弹模式
	}

	if (RC.Key.SW1 == RCS::SWPos::Up)
	{

		switch (stateCtrl.visionDebugType)
		{
		case VDT_ARMOR:
			gimbal.desireMode = gimbal.armorMode;
			tmpEnableAutoAim = 1;
			tmpOperatorAskFire = 1;
			tmpVisionAskFire = 1;
			break;
		case VDT_BUFF:
			enterBuff();
			tmpEnableAutoAim = 1;
			tmpOperatorAskFire = 1;
			tmpVisionAskFire = 1;
			break;
		case VDT_BASE_SHOOT:
			gimbal.desireMode = gimbal.baseShootMode;
			vision_SendStruct.baseShootMode = 1;
			tmpEnableAutoAim = 1;
			tmpOperatorAskFire = 1;
			tmpVisionAskFire = 1;
			break;
			break;
		case VDT_AUTO_GET_AMMO:
			if (RC.LastKey.SW1 != RCS::SWPos::Up && stateCtrl.getAmmoIng != 1) //对位期间不可再次启动对位
			{
				stateCtrl.getAmmoIng = 1;
				gimbal.desireMode = gimbal.autoGetAmmoMode;
				chassis.customFlag = 1;		 //启用自定义模式
				chassis.ifChassisFollow = 0; //取消底盘跟随
				chassis.ifTransform = 1;	 //前进方向为云台指定方向（斜着进补给站，操作手可以调整方向）
			}
		case VDT_AUTO_GO_AMMO:
			if (RC.LastKey.SW1 != RCS::SWPos::Up && intoStation.intoStationFalg != 1) //对位期间不可再次启动对位
			{
				enterAutoGoAmmo();
			}
			break;
		default:
			break;
		}
	}
	else if (RC.LastKey.SW1 == RCS::SWPos::Up) //从上跳变为其他
	{
		quitAutoGoAmmo();
		chassis.ifChassisFollow = 1;
		quitBuff();
		chassis.lockChassis = 0;
		gimbal.desireMode = gimbal.NormalMode;
		tmpEnableAutoAim = 0;
		vision_SendStruct.energyBeatMode = 0;
		vision_SendStruct.baseShootMode = 0;
		stateCtrl.autoGetAmmoFromTime = 0;
	}

	if (RC.Key.SW1 == RCS::SWPos::Down) //清弹不自动开摩擦轮
	{
		fireCtrl.fireMode = fireCtrl.CLEAN_AMMO;
	}
	else if (RC.LastKey.SW1 == RCS::SWPos::Down)
	{
		fireCtrl.fireMode = fireCtrl.MID_SHOOT;
	}

	if (RC.Key.SW2 == RCS::SWPos::Up)
	{
		tmpStartFrictionWheel = 1;
	}

	if (RC.Key.SW1 == RCS::SWPos::Mid)
	{
		if (RC.Key.Right_jump)
		{
			gimbal.desireMode = gimbal.armorMode;
			tmpEnableAutoAim = 1;
			tmpVisionAskFire = 1;
			if (!RC.LastKey.Right_jump)
			{
				stateCtrl.rightClickFromTimeFirst = stateCtrl.rightClickFromTimeNext;
				stateCtrl.rightClickFromTimeNext = getSysTimeUs() / 1e6f;
				if (timeIntervalFrom_f(stateCtrl.rightClickFromTimeFirst) < 0.25f)
				{
					vision_SendStruct.SpiningShoot = 1;
				}
			}
		}
		else if (RC.LastKey.Right_jump)
		{
			vision_SendStruct.SpiningShoot = 0;
			gimbal.desireMode = gimbal.NormalMode;
		}
	}

	if (RC.Key.left_jump)
	{
		tmpOperatorAskFire = 1;
	}

	if (((int)RC.Key.CH[0]) < -600 && ((int)RC.Key.CH[1]) > 600 && ((int)RC.Key.CH[2]) > 600 && ((int)RC.Key.CH[3]) > 600)
	{
		//▼ 检录模式
		chassis.powerParam.checkMode = 1; //重启清零
	}

	fireCtrl.operatorAskFire = tmpOperatorAskFire;
	fireCtrl.visionAskFire = tmpVisionAskFire;
	fireCtrl.startFrictionWheel = tmpStartFrictionWheel;
	vision_SendStruct.EnableAutoAim = tmpEnableAutoAim;
}

#include "KeyProcess.h"
KeyProcess ammoCoverToggleOpen(
	KEY_B, [](uint32_t *) {
		ammoCover.toggleCoverOpen();
	},
	NULL, NULL, 0, 1); //组合键

KeyProcess autoGoAmmo(
	KEY_B | KEY_SHIFT | KEY_CTRL, [](uint32_t *) {
		enterAutoGoAmmo();
	},
	NULL, NULL);

KeyProcess buffToggle(
	KEY_G, [](uint32_t *) {
		if (gimbal.desireMode == gimbal.buffMode)
		{
			quitBuff();
		}
		else
		{
			enterBuff();
		}
	},
	NULL, NULL, 1, 1); //单键，遇到ctrl和shift不处理

static float trunCarInterval = 1.0f;
static float lastTurnCarTime = 0;
KeyProcess trunCar(
	KEY_Q | KEY_CTRL, [](uint32_t *) {
		if (gimbal.desireMode == gimbal.NormalMode &&
			timeIntervalFrom_f(lastTurnCarTime) > trunCarInterval)
		{
			gimbal.gimbalPositionSet[0] += 180.0f;
			lastTurnCarTime = getSysTimeUs() / 1e6f;
		}
	},
	NULL, NULL, 1, 1); //单键，遇到ctrl和shift也处理

KeyProcess flySlope(
	KEY_V, [](uint32_t *) { chassis.flySloping = 1; chassisSend.tryUsingBackup = 1; },
	[](uint32_t *) {
		chassis.flySloping = 0;
	},
	NULL, 1, 1); //单键，遇到ctrl和shift也处理

KeyProcess tiltWalkToggle(
	KEY_G | KEY_CTRL, [](uint32_t *) {
		if (STANDARD_ID == GALAXY) //银河不需要此案件
			return;
		if (gimbal.yawDiffAngleForChassis == 0)
		{
			gimbal.yawDiffAngleForChassis = 45;
		}
		else
		{
			gimbal.yawDiffAngleForChassis = 0;
		}
	});

KeyProcess unLimitedFire(
	KEY_C, NULL, [](uint32_t *) {
		fireCtrl.fireMode = fireCtrl.MID_SHOOT;
		fireCtrl.unLimitedFired = 0; },
	[](uint32_t *) {
		fireCtrl.fireMode = fireCtrl.FAST_SHOOT;
		fireCtrl.unLimitedFired = 1;
	},
	1, 1); //单键，遇到ctrl和shift不处理

KeyProcess PathSwitch(
	KEY_SHIFT | KEY_C, [](uint32_t *) {
		chassis.powerParam.PowerPath_Switch = !chassis.powerParam.PowerPath_Switch;
	},
	NULL, NULL, 1, 1);

//KeyProcess cancelUnLimitedFire(
//	KEY_CTRL | KEY_C, [](uint32_t *) {
//		fireCtrl.fireMode = fireCtrl.MID_SHOOT;
//		fireCtrl.unLimitedFired = 0;
//	},
//	NULL, NULL); //组合键

KeyProcess fireFreqSwitch(
	KEY_F, NULL, [](uint32_t *) {
		fireCtrl.fireMode = fireCtrl.MID_SHOOT;
		fireCtrl.unLimitedFired = 0; },
	[](uint32_t *) {
		fireCtrl.fireMode = fireCtrl.FAST_SHOOT;
		fireCtrl.unLimitedFired = 0;
	},
	0, 0); //单键，遇到ctrl和shift不处理

KeyProcess forceOpenLoopSwitch(
	KEY_CTRL | KEY_C, [](uint32_t *) {
		fireCtrl.forceOpenLoop = !fireCtrl.forceOpenLoop;
	},
	NULL, NULL, 1, 1); //单键，遇到ctrl和shift不处理

KeyProcess cancelFollow(
	KEY_Z, NULL, [](uint32_t *) {
		chassis.ifTransform = 1;
		chassis.ifChassisFollow = 1; },
	[](uint32_t *) {
		chassis.ifTransform = 0;
		chassis.ifChassisFollow = 0;
	},
	1, 1); //单键，遇到ctrl和shift不处理

KeyProcess sentryShootToggle(
	KEY_R, NULL,
	[](uint32_t *) {
		vision_SendStruct.quickSentryShoot = 0;
	},
	[](uint32_t *) {
		vision_SendStruct.quickSentryShoot = 1;
	},
	1, 1); //单键，遇到ctrl和shift不处理

KeyProcess releaseBackupPower(
	KEY_X,
	[](uint32_t *) {
		chassisSend.tryUsingBackup = 1;
	},
	[](uint32_t *) {
		chassisSend.tryUsingBackup=0;
	},
	NULL, 1, 0); //单键，遇到ctrl和shift不处理

KeyProcess ULTSModeToggle(
	KEY_CTRL | KEY_X, [](uint32_t *) {
		if (chassis.powerParam.ULTSMode == 1)
			chassis.powerParam.ULTSMode = 0;
		else if (chassis.powerParam.ULTSMode == 0)
			chassis.powerParam.ULTSMode = 1;
	},
	NULL, NULL); //组合键
KeyProcess changeFlexRotate(
	KEY_CTRL | KEY_E, [](uint32_t *) {
		if (chassis.flexRotate == 1)
			chassis.flexRotate = 0;
		else if (chassis.flexRotate == 0)
			chassis.flexRotate = 1;
	},
	NULL, NULL); //组合键
KeyProcess rotate(
	KEY_E, NULL,
	[](uint32_t *) {
		chassis.rotateFlag = 0;
	},
	[](uint32_t *) { chassis.rotateFlag = 1; }); //单键，遇到ctrl和shift也处理

KeyProcess swing(
	KEY_Q, NULL, [](uint32_t *) { chassis.swingFlag = 0; },
	[](uint32_t *) { chassis.swingFlag = 1; }); //单键，遇到ctrl和shift也处理

KeyProcess reset(
	KEY_G | KEY_SHIFT | KEY_CTRL, [](uint32_t *) {
		__set_FAULTMASK(1); //关闭所有中断
		NVIC_SystemReset(); //复位
		while (1)
		{
		} //仅等待复位
	},
	NULL, NULL);

KeyProcess exitSpecialMode_W(
	KEY_W, [](uint32_t *) {
		if (RC.Key.SHIFT && RC.Key.G)
			return;
		if (gimbal.desireMode == gimbal.buffMode)
		{
			quitBuff();
		}
		quitAutoGoAmmo();
	},
	NULL, NULL, 0, 0);

KeyProcess exitSpecialMode_D(
	KEY_D, [](uint32_t *) {
		if (RC.Key.SHIFT && RC.Key.G)
			return;
		if (gimbal.desireMode == gimbal.buffMode)
		{
			quitBuff();
		}
		quitAutoGoAmmo();
	},
	NULL, NULL, 0, 0);

KeyProcess exitSpecialMode_A(
	KEY_A, [](uint32_t *) {
		if (RC.Key.SHIFT && RC.Key.G)
			return;
		if (gimbal.desireMode == gimbal.buffMode)
		{
			quitBuff();
		}
		quitAutoGoAmmo();
	},
	NULL, NULL, 0, 0);

KeyProcess exitSpecialMode_S(
	KEY_S, [](uint32_t *) {
		if (RC.Key.SHIFT && RC.Key.G)
			return;
		if (gimbal.desireMode == gimbal.buffMode)
		{
			quitBuff();
		}
		quitAutoGoAmmo();
	},
	NULL, NULL, 0, 0);

KeyProcess changeHeatChoose(
	KEY_C | KEY_CTRL | KEY_SHIFT, [](uint32_t *) {
		fireCtrl.heatChoose = !fireCtrl.heatChoose;
	},
	NULL, NULL, 0, 0);

KeyProcess changeChassisPerformaceTo2(
	KEY_F | KEY_CTRL, [](uint32_t *) {
		if (can1Feedback.jgmtOffline)
			stateCtrl.chassisPerformanceChoose = 1;
	},
	NULL, NULL, 1, 1);

KeyProcess changeChassisPerformaceTo1(
	KEY_V | KEY_CTRL, [](uint32_t *) {
		if (can1Feedback.jgmtOffline)
			stateCtrl.chassisPerformanceChoose = 0;
	},
	NULL, NULL, 1, 1);

KeyProcess changeGimbalPerformaceTo3(
	KEY_R | KEY_SHIFT, [](uint32_t *) {
		if (can1Feedback.jgmtOffline)
			stateCtrl.gimbalPerformanceChoose = 2;
	},
	NULL, NULL, 1, 1);

KeyProcess changeGimbalPerformaceTo2(
	KEY_F | KEY_SHIFT, [](uint32_t *) {
		if (can1Feedback.jgmtOffline)
			stateCtrl.gimbalPerformanceChoose = 1;
	},
	NULL, NULL, 1, 1);

KeyProcess changeGimbalPerformaceTo1(
	KEY_V | KEY_SHIFT, [](uint32_t *) {
		if (can1Feedback.jgmtOffline)
			stateCtrl.gimbalPerformanceChoose = 0;
	},
	NULL, NULL, 1, 1);

KeyProcess changeLevelTo3(
	KEY_R | KEY_CTRL | KEY_SHIFT, [](uint32_t *) {
		if (can1Feedback.jgmtOffline)
			stateCtrl.levelChoose = 2;
	},
	NULL, NULL, 1, 1);

KeyProcess changeLevelTo2(
	KEY_F | KEY_CTRL | KEY_SHIFT, [](uint32_t *) {
		if (can1Feedback.jgmtOffline)
			stateCtrl.levelChoose = 1;
	},
	NULL, NULL, 1, 1);

KeyProcess changeLevelTo1(
	KEY_V | KEY_CTRL | KEY_SHIFT, [](uint32_t *) {
		if (can1Feedback.jgmtOffline)
			stateCtrl.levelChoose = 0;
	},
	NULL, NULL, 1, 1);

KeyProcess energyBeatCentre(
	KEY_G | KEY_SHIFT, [](uint32_t *) { vision_SendStruct.energyBeatCentre = 1; },
	[](uint32_t *) {
		vision_SendStruct.energyBeatCentre = 0;
	},
	NULL, 1, 1);

KeyProcess changeUseImu(
	KEY_Z | KEY_CTRL | KEY_SHIFT, [](uint32_t *) {
		boardImu->forceImuUsed = 1;
		boardImu->forceAppoint = !boardImu->forceAppoint;
	});

KeyProcess visionOffsetUp(
	KEY_G | KEY_SHIFT | KEY_W, [](uint32_t *) {
		stateCtrl.addPitchFlag = 1;
	},
	NULL, NULL, 1, 1);
KeyProcess visionOffsetDown(
	KEY_G | KEY_SHIFT | KEY_S, [](uint32_t *) {
		stateCtrl.addPitchFlag = -1;
	},
	NULL, NULL, 1, 1);
KeyProcess visionOffsetLeft(
	KEY_G | KEY_SHIFT | KEY_A, [](uint32_t *) {
		stateCtrl.addYawFlag = 1;
	},
	NULL, NULL, 1, 1);
KeyProcess visionOffsetRight(
	KEY_G | KEY_SHIFT | KEY_D, [](uint32_t *) {
		stateCtrl.addYawFlag = -1;
	},
	NULL, NULL, 1, 1);
