#include "gimbal_task.h"
#include "motor.h"
#include "vision.h"
#include "imu_task.h"
#include "filter.h"
#include "dbus.h"

#define PID_OPERATOR_INDEX 0
#define PID_ARMOR_INDEX 1
#define PID_BUFF_INDEX 2
#define PID_BASE_SHOOT_INDEX 3

Kf visionYawAngle, visionPitchAngle;
Lpf2p fbFilter_Yaw, fbFilter_Pitch;

u8 kfSwitch = 1;

Gimbal gimbal;

/******电机即pid参数索引********/
#define YAW 0
#define PITCH 1

/*************操作手灵敏度相关***************/
#define OPERATOR_NORMAL 0	 //正常
#define OPERATOR_FINE_TUNE 1 //微调
#define OPERATOR_SILENCE 2	 //静默

float PitchStick_Sensitivity = -0.0006f; //灵敏度
float PitchMouse_Sensitivity = -0.003f;

float YawStick_Sensitivity = -0.0008f; //灵敏度
float YawMouse_Sensitivity = -0.0032f;

void Gimbal::init()
{
	loadGimbalParam();
	
	gimbalSensitivity[YAW][OPERATOR_NORMAL] = 1;
	gimbalSensitivity[PITCH][OPERATOR_NORMAL] = 1;

	gimbalSensitivity[YAW][OPERATOR_FINE_TUNE] = 0.5;
	gimbalSensitivity[PITCH][OPERATOR_FINE_TUNE] = 0.5;

	gimbalSensitivity[YAW][OPERATOR_SILENCE] = 0;
	gimbalSensitivity[PITCH][OPERATOR_SILENCE] = 0;

	gimbalMotor[YAW].setZeroValue(yawZeroAngle);
	gimbalMotor[PITCH].setZeroValue(pitchZeroAngle);

	for (int i = 0; i < GIMBAL_MOTOR_COUNT; i++)
	{
		gimbalMotor[i].pidInner.setPlanNum(PID_GIMBAL_PARAM_COUNT);
		gimbalMotor[i].pidInner.paramPtr = gimbalParamInner[i];
		gimbalMotor[i].pidOuter.setPlanNum(PID_GIMBAL_PARAM_COUNT);
		gimbalMotor[i].pidOuter.paramPtr = gimbalParamOuter[i];
	}

	for (int i = 0; i < PID_GIMBAL_PARAM_COUNT; i++)
	{
		gimbalMotor[YAW].pidOuter.fbValuePtr[i] = &gimbalPositionFb[YAW];
		gimbalMotor[PITCH].pidOuter.fbValuePtr[i] = &gimbalPositionFb[PITCH];
		gimbalMotor[YAW].pidInner.fbValuePtr[i] = &speedFb[YAW];
		gimbalMotor[PITCH].pidInner.fbValuePtr[i] = &speedFb[PITCH];
	}

	imu_FbSet();
	valueSwitch();
}

void Gimbal::transformChassisAngle()
{
	if (!deforceFlag)
		for (int i = 0; i < GIMBAL_MOTOR_COUNT; i++)
		{
			gimbal.gimbalMotor[i].ctrlPosition(gimbal.gimbalPositionSet[i], gimbal.pidUseParamIndex);
			gimbal.gimbalPositionErr[i] = gimbal.gimbalPositionSet[i] - gimbal.gimbalPositionFb[i];
		}

	float yawAngleForChassis = gimbal.gimbalMotor[YAW].canInfo.encoderCalibration * 360 / 8192.0f;

	if (gimbal.gimbalMotor[YAW].canInfo.lostFlag)
	{
		yawAngleForChassis = 0;
	}

	if (yawAngleForChassis > 180)
	{
		yawAngleForChassis -= 360;
	}
	else if (yawAngleForChassis < -180)
	{
		yawAngleForChassis += 360;
	}

	gimbal.yawAngleForChassis = yawAngleForChassis;
}

#define FIND_ENEMY_OBJ (!vision_RecvStruct.no_Obj && !visionInfo.offlineFlag)
void Gimbal::run()
{
	imu_FbSet();
	if (deforceFlag)
	{
		valueSwitch();
		transformChassisAngle();
		return;
	}
	
	

	u8 operatorSensitivity = OPERATOR_NORMAL;
	switch (desireMode)
	{
	case buffMode:
		operatorSensitivity = OPERATOR_SILENCE;
		pidUseParamIndex = PID_BUFF_INDEX;
		gimbalMode = buffMode;
		if (FIND_ENEMY_OBJ && vision_RecvStruct.isInEnergyBeat)
		{
			if (kfSwitch)
			{
				gimbalPositionSet[YAW] = visionYawAngle.KalmanFilter(vision_RecvStruct.Yaw, yawProcessNiose_Q, yawMeasureNoise_R);
				gimbalPositionSet[PITCH] = visionPitchAngle.KalmanFilter(vision_RecvStruct.Pitch, pitchProcessNiose_Q, pitchMeasureNoise_R);
			}
			else
			{
				gimbalPositionSet[YAW] = vision_RecvStruct.Yaw;
				gimbalPositionSet[PITCH] = vision_RecvStruct.Pitch;
			}
		}
		break;
	case baseShootMode:

		gimbalMode = baseShootMode;
		pidUseParamIndex = PID_BASE_SHOOT_INDEX;
		operatorSensitivity = OPERATOR_FINE_TUNE;
		//todo 使用机械角 使用视觉
		break;
	case armorMode:
		if (FIND_ENEMY_OBJ && !vision_RecvStruct.isInEnergyBeat)
		{
			gimbalMode = armorMode;
			if (kfSwitch)
			{
				gimbalPositionSet[YAW] = visionYawAngle.KalmanFilter(vision_RecvStruct.Yaw, yawProcessNiose_Q, yawMeasureNoise_R);
				gimbalPositionSet[PITCH] = visionPitchAngle.KalmanFilter(vision_RecvStruct.Pitch, pitchProcessNiose_Q, pitchMeasureNoise_R);
			}
			else
			{
				gimbalPositionSet[YAW] = vision_RecvStruct.Yaw;
				gimbalPositionSet[PITCH] = vision_RecvStruct.Pitch;
			}
			operatorSensitivity = OPERATOR_SILENCE;
			pidUseParamIndex = PID_ARMOR_INDEX;
		}
		else
		{
			pidUseParamIndex = PID_OPERATOR_INDEX;
			operatorSensitivity = OPERATOR_NORMAL;
			gimbalMode = NormalMode;
		}
		break;
	case autoGetAmmoMode:
		gimbalMode = autoGetAmmoMode;
		pidUseParamIndex = PID_OPERATOR_INDEX;
		operatorSensitivity = OPERATOR_SILENCE;
		break;
	case NormalMode:
		gimbalMode = NormalMode;
		pidUseParamIndex = PID_OPERATOR_INDEX;
		operatorSensitivity = OPERATOR_NORMAL;
		break;
	default:
		pidUseParamIndex = PID_OPERATOR_INDEX;
		operatorSensitivity = OPERATOR_NORMAL;
		break;
	}

	if ((gimbalMode != lastGimbalMode)) //note 如果在目标丢失，\
		gimbalMode会从装甲板切换成普通模式，设定值等于反馈值
	{
		valueSwitch();
	}

	if(boardImu->forceAppoint && boardImu->currentImuUsed != boardImu->lastCurrentImuUsed)
	{
		valueSwitch();
		for(int i = 0; i < 2; i++)
		{
			gimbalMotor[i].pidOuter.Clear();
			gimbalMotor[i].pidInner.Clear();
		}
	}
	
	lastFIND_ENEMY_OBJ = FIND_ENEMY_OBJ;
	lastGimbalMode = gimbalMode;

	//操作手控制
	operatorCtrl(operatorSensitivity);

	valueSetLimit();

	transformChassisAngle();
}

void Gimbal::valueSetLimit()
{
	//todo 利用机械角限幅
	gimbalPositionSet[PITCH] = LIMIT(gimbalPositionSet[PITCH], pitchUpAngle, pitchDownAngle);
}

void Gimbal::operatorCtrl(u8 operatorSensitivity)
{
	float angleDelta[GIMBAL_MOTOR_COUNT];
	angleDelta[PITCH] = RC.Key.CH[1] * PitchStick_Sensitivity + RC.Key.CH[7] * PitchMouse_Sensitivity;

	angleDelta[YAW] = RC.Key.CH[0] * YawStick_Sensitivity + RC.Key.CH[6] * YawMouse_Sensitivity;

	for (int i = 0; i < GIMBAL_MOTOR_COUNT; i++)
	{
		gimbalPositionSet[i] += angleDelta[i] * gimbalSensitivity[i][operatorSensitivity];
	}
}



void Gimbal::imu_FbSet()
{
	gimbalPositionFb[YAW] = *visionSendYaw;		 //boardImu->AngleFuseWithTrustAngle.yaw;
	gimbalPositionFb[PITCH] = *visionSendPitch; //-boardImu->Angle.roll;
	speedFb[YAW] = boardImu->gyro.dps.data[2];
	speedFb[PITCH] = boardImu->gyro.dps.data[1];
}

void Gimbal::mech_FbSet(u8 speedIsImu)
{
	gimbalPositionFb[YAW] = gimbalMotor[YAW].canInfo.totalAngle_f;	   //boardImu->Angle.yaw;
	gimbalPositionFb[PITCH] = gimbalMotor[PITCH].canInfo.totalAngle_f; //boardImu->Angle.pitch;
	if (!speedIsImu)
	{
		speedFb[YAW] = gimbalMotor[YAW].canInfo.dps;	 //boardImu->gyro.dps.data[2];
		speedFb[PITCH] = gimbalMotor[PITCH].canInfo.dps; //boardImu->gyro.dps.data[1];
	}
	else
	{
		speedFb[YAW] = boardImu->gyro.dps.data[2];
		speedFb[PITCH] = boardImu->gyro.dps.data[1];
	}
}


#include "parameter.h"

void Gimbal::loadGimbalParam()
{
	switch (STANDARD_ID)
	{
		case NIRVANA:
		gimbalMotor= new Motor[GIMBAL_MOTOR_COUNT]{
			Motor(GM6020, CAN1, 0x208),
			Motor(GM6020, CAN1, 0x206)};

		pitchUpAngle = -35;
		pitchDownAngle = 23;
		yawZeroAngle = 3725; //Y轴零点
		pitchZeroAngle = 1292; //P轴零点
		yawProcessNiose_Q = 0.05; //Y过程噪声
		yawMeasureNoise_R = 1; //Y测量噪声
		pitchProcessNiose_Q = 0.1; //P过程噪声
		pitchMeasureNoise_R = 0; //P测量噪声
		kfSwitch = 1; //使用卡尔曼滤波


		gimbalParamInner[YAW][PID_OPERATOR_INDEX].kp = 100;
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].ki = 100;
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].kd = 0;
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].integralErrorMax = 1;

		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].kp = 20;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].ki = 0;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].integralErrorMax = 100;

		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].kp = 200;
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].ki =-100;
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].kd = 0;
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].integralErrorMax = 0.5;

		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].kp = 30;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].ki = 0;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].integralErrorMax = 0.2;

		gimbalParamInner[YAW][PID_ARMOR_INDEX].kp = 120;//400;
		gimbalParamInner[YAW][PID_ARMOR_INDEX].ki = 2000;
		gimbalParamInner[YAW][PID_ARMOR_INDEX].kd = 0;//0.1;
		gimbalParamInner[YAW][PID_ARMOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_ARMOR_INDEX].integralErrorMax = 1;
							  
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].kp = 20;//15;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].ki = 0;//0;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].integralErrorMax = 0.5;

		gimbalParamInner[PITCH][PID_ARMOR_INDEX].kp = 150;//-300;
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].ki = 1000;//1000;
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].kd = 0;
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].integralErrorMax = 1;
								
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].kp = 20;//8;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].ki = 0;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].integralErrorMax = 100;

		gimbalParamInner[YAW][PID_BUFF_INDEX].kp = 200;
		gimbalParamInner[YAW][PID_BUFF_INDEX].ki = 200;
		gimbalParamInner[YAW][PID_BUFF_INDEX].kd = 0;
		gimbalParamInner[YAW][PID_BUFF_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_BUFF_INDEX].integralErrorMax = 10;

		gimbalParamOuter[YAW][PID_BUFF_INDEX].kp = 50;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].ki = 150;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].integralErrorMax = 100;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].integralMethod = 1;		
		gimbalParamOuter[YAW][PID_BUFF_INDEX].integralThreshold = 1;		

		gimbalParamInner[PITCH][PID_BUFF_INDEX].kp = 300;
		gimbalParamInner[PITCH][PID_BUFF_INDEX].ki = 100;
		gimbalParamInner[PITCH][PID_BUFF_INDEX].kd = 0;
		gimbalParamInner[PITCH][PID_BUFF_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_BUFF_INDEX].integralErrorMax = 10;

		gimbalParamOuter[PITCH][PID_BUFF_INDEX].kp = 30;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].ki = 100;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].integralErrorMax = 100;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].integralMethod= 1;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].integralThreshold = 1;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].kpAmpForIntegralSep = 1;

		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].kp = 300;
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].kp = 10;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].kp = 200;
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].kp = 12;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;
		break;
		
		
		
		
		
		
		
		
		
		
		case TWONETWONE_CAR:
		gimbalMotor = new Motor[GIMBAL_MOTOR_COUNT]{
			  Motor(GM6020, CAN1, 0x206),
			  Motor(GM6020, CAN2, 0x205)}; //2121
		
		pitchUpAngle = -42; //P轴上限
		pitchDownAngle = 18; //P轴下限
		yawZeroAngle = 4375; //Y轴零点
		pitchZeroAngle = 6828; //P轴零点
		yawProcessNiose_Q = 0.05; //Y过程噪声,todo 近距离buff时0.01
		yawMeasureNoise_R = 1; //Y测量噪声,todo 近距离buff时5
		pitchProcessNiose_Q = 0.1; //P过程噪声,todo 近距离buff时0.01
		pitchMeasureNoise_R = 0; //P测量噪声,todo 近距离buff时5
		kfSwitch = 1; //使用卡尔曼滤波

		gimbalParamInner[YAW][PID_OPERATOR_INDEX].kp = 120;//400;
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].ki = 2000;
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].kd = 0;//0.1;
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].integralErrorMax = 1;

		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].kp = 15;//15;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].ki = 0;//0;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].integralErrorMax = 0.5;

		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].kp = -150;//-300;
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].ki = -1000;//1000;
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].kd = 0;
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].integralErrorMax = 1;

		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].kp = 20;//8;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].ki = 0;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].integralErrorMax = 100;
		
		gimbalParamInner[YAW][PID_ARMOR_INDEX].kp = 120;//400;
		gimbalParamInner[YAW][PID_ARMOR_INDEX].ki = 2000;
		gimbalParamInner[YAW][PID_ARMOR_INDEX].kd = 0;//0.1;
		gimbalParamInner[YAW][PID_ARMOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_ARMOR_INDEX].integralErrorMax = 1;
								  
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].kp = 20;//15;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].ki = 0;//0;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].integralErrorMax = 0.5;

		gimbalParamInner[PITCH][PID_ARMOR_INDEX].kp = -150;//-300;
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].ki = -1000;//1000;
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].kd = 0;
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].integralErrorMax = 1;
									
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].kp = 20;//8;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].ki = 0;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].integralErrorMax = 100;

		gimbalParamInner[YAW][PID_BUFF_INDEX].kp = 120;//400;
		gimbalParamInner[YAW][PID_BUFF_INDEX].ki = 2000;
		gimbalParamInner[YAW][PID_BUFF_INDEX].kd = 0;//0.1;
		gimbalParamInner[YAW][PID_BUFF_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_BUFF_INDEX].integralErrorMax = 1;

		gimbalParamOuter[YAW][PID_BUFF_INDEX].kp = 20;//15;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].ki = 0;//0;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].integralErrorMax = 0.5;

		gimbalParamInner[PITCH][PID_BUFF_INDEX].kp = -150;//-300;
		gimbalParamInner[PITCH][PID_BUFF_INDEX].ki = -1000;//1000;
		gimbalParamInner[PITCH][PID_BUFF_INDEX].kd = 0;
		gimbalParamInner[PITCH][PID_BUFF_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_BUFF_INDEX].integralErrorMax = 1;
									
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].kp = 20;//8;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].ki = 0;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].integralErrorMax = 100;

		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].kp = 300;
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].kp = 10;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].kp = -200;
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].kp = 12;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;
		break;
			
		
		
		
		
		
		
		
		case AURORA:
		gimbalMotor = new Motor[GIMBAL_MOTOR_COUNT]{
			  Motor(GM6020, CAN1, 0x206),
			  Motor(GM6020, CAN1, 0x205)}; //2121
		
		pitchUpAngle = -42; //P轴上限
		pitchDownAngle = 18; //P轴下限
		yawZeroAngle = 4695; //Y轴零点
		pitchZeroAngle = 2696; //P轴零点
		yawProcessNiose_Q = 0.05; //Y过程噪声
		yawMeasureNoise_R = 1; //Y测量噪声
		pitchProcessNiose_Q = 0.1; //P过程噪声
		pitchMeasureNoise_R = 0; //P测量噪声
		kfSwitch = 1; //使用卡尔曼滤波

		gimbalParamInner[YAW][PID_OPERATOR_INDEX].kp = 120;//400;
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].ki = 2000;
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].kd = 0;//0.1;
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].integralErrorMax = 1;

		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].kp = 20;//15;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].ki = 0;//0;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].integralErrorMax = 0.5;

		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].kp = -150;//-300;
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].ki = -1000;//1000;
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].kd = 0;
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].integralErrorMax = 1;

		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].kp = 20;//8;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].ki = 0;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].integralErrorMax = 100;

		gimbalParamInner[YAW][PID_ARMOR_INDEX].kp = 120; //400
		gimbalParamInner[YAW][PID_ARMOR_INDEX].ki = 200;
		gimbalParamInner[YAW][PID_ARMOR_INDEX].kd = 0;
		gimbalParamInner[YAW][PID_ARMOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_ARMOR_INDEX].integralErrorMax = 5; //1

		gimbalParamOuter[YAW][PID_ARMOR_INDEX].kp = 40; //7
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].ki = 100;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].integralErrorMax = 100;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].kpAmpForIntegralSep = 0;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].integralThreshold = 3;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].integralMethod = 1;

		gimbalParamInner[PITCH][PID_ARMOR_INDEX].kp = -120;
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].ki = 0;
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].kd = 0; //2.5
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].integralErrorMax = 1;

		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].kp = 20; //14
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].ki =200; //60
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].integralErrorMax = 100;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].kpAmpForIntegralSep = 2;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].integralThreshold = 3;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].integralMethod = 1;

		gimbalParamInner[YAW][PID_BUFF_INDEX].kp = 120;//400;
		gimbalParamInner[YAW][PID_BUFF_INDEX].ki = 800;
		gimbalParamInner[YAW][PID_BUFF_INDEX].kd = 0;//0.1;
		gimbalParamInner[YAW][PID_BUFF_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_BUFF_INDEX].integralErrorMax = 1;

		gimbalParamOuter[YAW][PID_BUFF_INDEX].kp = 40;//15;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].ki = 100;//0;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].integralErrorMax = 100;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].kpAmpForIntegralSep = 1;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].integralThreshold = 3;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].integralMethod = 1;


		gimbalParamInner[PITCH][PID_BUFF_INDEX].kp = -120;//-300;
		gimbalParamInner[PITCH][PID_BUFF_INDEX].ki = -100;//1000;
		gimbalParamInner[PITCH][PID_BUFF_INDEX].kd = 0;
		gimbalParamInner[PITCH][PID_BUFF_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_BUFF_INDEX].integralErrorMax = 10;
									
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].kp = 30;//8;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].ki = 200;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].integralErrorMax = 100;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].kpAmpForIntegralSep = 1;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].integralThreshold = 3;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].integralMethod = 1;

		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].kp = 300;
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].kp = 10;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].kp = -200;
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].kp = 12;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;
		break;









		case GALAXY:
		gimbalMotor = new Motor[GIMBAL_MOTOR_COUNT]{
			  Motor(GM6020, CAN1, 0x206),
			  Motor(GM6020, CAN1, 0x205)}; //2121
		
		pitchUpAngle = -44; //P轴上限
		pitchDownAngle = 23; //P轴下限
		yawZeroAngle = 2007; //Y轴零点
		pitchZeroAngle = 2369; //P轴零点
		yawProcessNiose_Q = 0.05; //Y过程噪声
		yawMeasureNoise_R = 1; //Y测量噪声
		pitchProcessNiose_Q = 0.1; //P过程噪声
		pitchMeasureNoise_R = 0; //P测量噪声
		kfSwitch = 1; //使用卡尔曼滤波

		gimbalParamInner[YAW][PID_OPERATOR_INDEX].kp = 100;//400;
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].ki = 0;
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].kd = 0;//0.1;
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_OPERATOR_INDEX].integralErrorMax = 1;

		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].kp = 20;//15;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].ki = 0;//0;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_OPERATOR_INDEX].integralErrorMax = 0.5;

		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].kp = -200;//-300;
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].ki = -100;//1000;
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].kd = 0;
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_OPERATOR_INDEX].integralErrorMax = 10;

		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].kp = 30;//8;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].ki = 0;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].integralErrorMax = 100;

		gimbalParamInner[YAW][PID_ARMOR_INDEX].kp = 120; //400
		gimbalParamInner[YAW][PID_ARMOR_INDEX].ki = 2000;
		gimbalParamInner[YAW][PID_ARMOR_INDEX].kd = 0;
		gimbalParamInner[YAW][PID_ARMOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_ARMOR_INDEX].integralErrorMax = 0.5; //1

		gimbalParamOuter[YAW][PID_ARMOR_INDEX].kp = 40; //7
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].ki = 100;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].integralErrorMax = 100;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].kpAmpForIntegralSep = 1;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].integralThreshold = 5;
		gimbalParamOuter[YAW][PID_ARMOR_INDEX].integralMethod = 1;

		gimbalParamInner[PITCH][PID_ARMOR_INDEX].kp = -120;
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].ki = -1000;
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].kd = 0; //2.5
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_ARMOR_INDEX].integralErrorMax = 1;

		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].kp = 20; //14
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].ki =150; //60
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].integralErrorMax = 100;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].kpAmpForIntegralSep = 2;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].integralThreshold = 2;
		gimbalParamOuter[PITCH][PID_ARMOR_INDEX].integralMethod = 1;

		gimbalParamInner[YAW][PID_BUFF_INDEX].kp = 150;//400;
		gimbalParamInner[YAW][PID_BUFF_INDEX].ki = 200;
		gimbalParamInner[YAW][PID_BUFF_INDEX].kd = 0;//0.1;
		gimbalParamInner[YAW][PID_BUFF_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_BUFF_INDEX].integralErrorMax = 10;

		gimbalParamOuter[YAW][PID_BUFF_INDEX].kp = 60;//15;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].ki = 300;//0;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].integralErrorMax = 100;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].kpAmpForIntegralSep = 0;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].integralThreshold = 3;
		gimbalParamOuter[YAW][PID_BUFF_INDEX].integralMethod = 1;


		gimbalParamInner[PITCH][PID_BUFF_INDEX].kp = -120;//-300;
		gimbalParamInner[PITCH][PID_BUFF_INDEX].ki = -200;//1000;
		gimbalParamInner[PITCH][PID_BUFF_INDEX].kd = 0;
		gimbalParamInner[PITCH][PID_BUFF_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_BUFF_INDEX].integralErrorMax = 10;
									
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].kp = 25;//8;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].ki = 300;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].integralErrorMax = 100;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].kpAmpForIntegralSep = 1;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].integralThreshold = 3;
		gimbalParamOuter[PITCH][PID_BUFF_INDEX].integralMethod = 1;

		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].kp = 300;
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].kp = 10;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].kp = -200;
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
		gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].kp = 12;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].ki = 0;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].kd = 0;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
		gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;
		break;
	}
	
	fbFilter_Yaw.SetCutoffFreq(yawFbLPF_smpFreq, yawFbLPF_coFreq);
	fbFilter_Pitch.SetCutoffFreq(pitchFbLPF_smpFreq, pitchFbLPF_coFreq);	
}
