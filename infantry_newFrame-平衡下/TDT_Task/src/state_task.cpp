#include "state_task.h"
#include "ammo_cover_task.h"
#include "gimbal_task.h"
#include "fire_task.h"
#include "chassis_task.h"
#include "vision.h"
#include "dbus.h"
#include "can.h"
#include "imu_task.h"
#include "parameter.h"
#include "judgement.h"
StateCtrl stateCtrl;
CanStateStruct canStateStruct;
PowerInfoMsg powerInfoMsg;
PowerHeatData powerHeatData;
JgmtInfoMsg jgmtInfoMsg;
PowerRecvMsg powerRecvMsg;
void State_Ctrl()
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
	memcpy(&canStateStruct, stateCtrl.stateStructFromCan, 8);
	powerInfoMsg.chassis_power = judgement.powerHeatData.chassisPower;
	powerInfoMsg.chassis_power_buffer = judgement.powerHeatData.chassisPowerBuffer;
	powerInfoMsg.max_chassis_power = judgement.gameRobotStatus.chassisPowerLimit;
	canTx((u8*)&powerInfoMsg,CAN1,0x110);
	
	powerHeatData.speedWS = chassis.speedFb;
	powerHeatData.jgmeOffline = judgement.jgmtOffline;
	powerHeatData.bulletSpeed = uint16_t(judgement.shootData.bulletSpeed*10);
	powerHeatData.coolingHeat = judgement.powerHeatData.shooterId1_17mmCoolingHeat;
	powerHeatData.shootNum = judgement.shootNum[0];
	canTx((u8*)&powerInfoMsg,CAN1,0x160);
	
	jgmtInfoMsg.robotLevel = judgement.gameRobotStatus.robotLevel;
	jgmtInfoMsg.gameProgress = judgement.gameStatus.gameProgress;
	jgmtInfoMsg.hurtType = judgement.robotHurt.hurtType;
	if(judgement.gameRobotStatus.robotId == 0)
		jgmtInfoMsg.enemyColor = 0;
	else if(judgement.gameRobotStatus.robotId <=9)
		jgmtInfoMsg.enemyColor = 1;
	else
		jgmtInfoMsg.enemyColor = 2;
	jgmtInfoMsg.speedLimit = LIMIT(judgement.gameRobotStatus.shooterId1_17mmSpeedLimit,15,30);
	jgmtInfoMsg.coolingRate = judgement.gameRobotStatus.shooterId1_17mmCoolingRate;
	jgmtInfoMsg.coolingLimit = judgement.gameRobotStatus.shooterId1_17mmCoolingLimit;
	jgmtInfoMsg.maxHp = judgement.gameRobotStatus.maxHp;
	canTx((u8*)&powerInfoMsg,CAN1,0x140);
}

void State_Ctrl_RC_Info()
{
}

