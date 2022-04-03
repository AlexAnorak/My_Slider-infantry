#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "board.h"
#include "motor.h"
#include "pid.h"
#define GIMBAL_MOTOR_COUNT 2
/*********PID参数索引相关*********/
#define PID_GIMBAL_PARAM_COUNT 4

struct Gimbal
{
public:
	Motor *gimbalMotor;

	PidParam gimbalParamInner[GIMBAL_MOTOR_COUNT][PID_GIMBAL_PARAM_COUNT];
	PidParam gimbalParamOuter[GIMBAL_MOTOR_COUNT][PID_GIMBAL_PARAM_COUNT];
    float yawProcessNiose_Q , yawMeasureNoise_R;
    float pitchProcessNiose_Q , pitchMeasureNoise_R;
	float yawFbLPF_smpFreq=1000, yawFbLPF_coFreq=42;
	float pitchFbLPF_smpFreq=1000, pitchFbLPF_coFreq=42;

	enum
	{
		NormalMode = 0,
		armorMode,
		buffMode,
		baseShootMode,
		autoGetAmmoMode,
	}desireMode, //期望的模式
	gimbalMode, lastGimbalMode;//实际当前模式(期望模式的条件不满足)
	u8 pidUseParamIndex;
	float speedFb[GIMBAL_MOTOR_COUNT];
	float gimbalSensitivity[GIMBAL_MOTOR_COUNT][3];
	void operatorCtrl(u8 operatorSensitivity);
	float gimbalPositionSet[GIMBAL_MOTOR_COUNT],gimbalPositionFb[GIMBAL_MOTOR_COUNT], gimbalPositionErr[GIMBAL_MOTOR_COUNT];
	u8 lastFIND_ENEMY_OBJ;
	float pitchUpAngle, pitchDownAngle;
    float yawZeroAngle, pitchZeroAngle;

	void init();
	void run();
	void loadGimbalParam();
	void valueSetLimit();
	void imu_FbSet();
	void mech_FbSet(u8 speedIsImu);
	float yawAngleForChassis;
	float yawDiffAngleForChassis;
	
	void valueSwitch(){memcpy(gimbalPositionSet, gimbalPositionFb, sizeof(gimbalPositionFb));};

	void transformChassisAngle();
};

extern Gimbal gimbal;

#endif
