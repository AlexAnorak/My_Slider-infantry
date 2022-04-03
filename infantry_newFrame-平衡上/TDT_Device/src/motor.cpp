/******************************
File name: TDT_Device\src\can_task.cpp
Description: 电机数据的处理
Class:
	——————————————————————————————————————————————————————————————————————————
	Motor
	——————————————————————————————————————————————————————————————————————————
Author: 郑竣元
Version: 1.3.1.191119_alpha
Date: 19.10.15
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.24 肖银河-改写数据合帧的方法，修复数据合帧时的BUG
	——————————————————————————————————————————————————————————————————————————
	19.11.19 肖银河-总体框架更改，将三大主体部分重新分配完成-核心功能已测试
	——————————————————————————————————————————————————————————————————————————
	19.11.15 肖银河-将方法的具体实现放到cpp文件
	——————————————————————————————————————————————————————————————————————————
	19.11.12 肖银河-将C_Motor分为Pid feedBack Motor三部分，并完善代码规范
	——————————————————————————————————————————————————————————————————————————
	19.10.15 郑竣元-首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************/
#include "motor.h"
#include "can.h"
#include "dbus.h"

/**
 * @ingroup TDT_DEVICE
 * @defgroup TDT_Motor 电机处理
 * @brief 该类提供了大疆电机的数据处理、CAN接受与发送以及PID计算算法
 * @todo - 将Motor继续抽象成虚拟电机，大疆CAN电机继承虚拟电机，其余各种电机继承Motor类，这样能更方便适配其他电机
 * - 将各种保护写成虚函数，方便自定义的保护
 * @par 单方案电机示例:
 * @code {.cpp}
 * Motor djiMotor(GM6020, CAN1, 0x205);
 * PidParam djiMotorPidInner = {10, 0, 0, 0, Motor::getMotorCurrentLimit(GM6020)}; //输出值获取方法1
 * PidParam djiMotorPidOuter = {10, 0, 0, 0};
 * void init()
 * {
 * 	// ...
 * 	djiMotorPidOuter.resultMax = djiMotor.getMotorCurrentLimit();
 * 	djiMotor.pidInner.paramPtr = &djiMotorPidInner; //设定参数指针
 * 	djiMotor.pidOuter.paramPtr = &djiMotorPidOuter; //设定参数指针
 * 	djiMotor.motorInit();
 * 	// ...
 * }
 * 
 * float motorPositionSet;
 * void loop()
 * {
 * 	// ...
 * 	djiMotor.ctrlPosition(motorPositionSet);//仅需调用controlPosition，自动调用controlSpeed以及controlCurrent并自动发送
 * 	// ...
 * }
 * @endcode
 * @par 多方案电机示例:
 * @code {.cpp}
 * #include "imu_task.h"
 * #define MOTOR_PLAN 3
 * Motor djiMotor(GM6020, CAN1, 0x205);
 * //设置参数方法1：直接赋值
 * PidParam djiMotorPidInner[MOTOR_PLAN] = {{10, 0, 0, 0, Motor::getMotorCurrentLimit(GM6020)},
 * 										 {10, 0, 0, 0, Motor::getMotorCurrentLimit(GM6020)},
 * 										 {10, 0, 0, 0, Motor::getMotorCurrentLimit(GM6020)}};
 * //设置参数方法2：在运行时赋值
 * PidParam djiMotorPidOuter[MOTOR_PLAN] = {0};
 * float customFbValue;
 * void init()
 * {
 * 	// ...
 * 
 * 	djiMotorPidOuter[0].kp = 10;
 * 	djiMotorPidOuter[0].resultMax = djiMotor.getMotorSpeedLimit();
 * 	//...
 * 	djiMotorPidOuter[2].resultMax = djiMotor.getMotorSpeedLimit();
 * 	djiMotor.pidInner.paramPtr = djiMotorPidInner; //设定参数指针
 * 	djiMotor.pidOuter.paramPtr = djiMotorPidOuter; //设定参数指针
 * 
 * 	djiMotor.pidInner.fbValuePtr[0] = &djiMotor.canInfo.dps;//默认反馈值，只有一个方案或者该方案的反馈值一样可以省略
 * 	djiMotor.pidInner.fbValuePtr[1] = &boardImu->gyro.radps.data[2];
 * 	djiMotor.pidInner.fbValuePtr[1] *= -1;//反馈值方向反了，为防止正反馈，可通过反馈值指针的*=操作符设置正反转
 * 	djiMotor.pidInner.fbValuePtr[1] *= RAD_TO_ANGLE; //还可以放大，使得方案号1和方案号2的反馈值的单位相同
 * 
 * 	djiMotor.pidInner.fbValuePtr[2] = &customFbValue;
 * 
 * 	djiMotor.motorInit();
 * 	// ...
 * }
 * 
 * float motorPositionSet;
 * void loop()
 * {
 * 	// ...
 * 	djiMotor.ctrlPosition(motorPositionSet,2);//使用第三套方案的反馈值以及pid参数
 * 	// ...
 * }
 * @endcode
 * @{
 */

///电机电流输出限制表
///@note 顺序对应枚举变量MotorType里的电机类型排列-M2006,M3508,M3510,GM3510,GM6020
uint16_t motorCurrentLimitList[5] = {10000, 16384, 32760, 29000, 30000};
//电机速度输出限制表-取决于各类电机的最高转速
///@note 顺序对应枚举变量MotorType里的电机类型排列-M2006,M3508,M3510,GM3510,GM6020
uint16_t motorSpeedLimitList[5] = {9500, 9158, 9600, 1200, 320};
//电机绕组最高允许温度列表
///@note 顺序对应枚举变量MotorType里的电机类型排列-M2006,M3508,M3510,GM3510,GM6020
uint16_t motorMaxTempList[5] = {0, 125, 0, 100, 125};

extern uint8_t deforceFlag;
/** @} */

///大疆电机对象列表
MotorList motorList[2][8];
///电机类大疆电机标志位初始化
u8 Motor::DJI_MotorFlag[2][8] = {0};
///大疆电机CAN发送缓冲区
float Motor::canBuff[2][8] = {0};

/**
 * @class Motor
 * @copydoc TDT_Motor
 */

/**
 * @param  motorType        电机类型: 用以区别不同类型的电机
 * @param  _Canx             CAN口
 * @param  _Std_ID           CAN标准标识符
 * @sa motorInit
 */
Motor::Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t _Std_ID) : pidInner(Pid(1)), pidOuter(Pid(1))
{
	//电机基本信息填充
	motorInfo.type = motorType;
	motorInfo.std_ID = _Std_ID;
	motorInfo.can_x = _Canx == CAN1 ? Can_1 : Can_2;
	canInfo.offsetEncoder = 0;

	//电机额外信息填充
	otherInfo.currentLimit = motorCurrentLimitList[motorType]; //输出限幅（内环输出限幅）
	otherInfo.speedLimit = motorSpeedLimitList[motorType];	   //速度限幅（外环输出限幅）
	otherInfo.tempLimit = motorMaxTempList[motorType];		   //绕组最高允许温度
	otherInfo.offSetLimit = otherInfo.currentLimit / 3;		   //默认位置校零时以三分之一输出开始
	otherInfo.criticalTemp = 0.7f * otherInfo.tempLimit;	   //过热保护临界温度
	otherInfo.maxOverTemp = 0.8f * otherInfo.tempLimit;		   //过热保护截止温度，高于此温度电机输出为0

	//对于有最高绕组温度的电机开启过热保护
	if (motorMaxTempList[motorType] != 0)
	{
		this->enableFlag.overTempProtect = 1; //过热保护使能
	}

	//pid默认反馈值填充
	pidInner.fbValuePtr[0] = &canInfo.speed;
	pidOuter.fbValuePtr[0] = &canInfo.totalEncoder;

	//进行初始化
	motorInit();
	canInfo.lostFlag = 1;
}

/**
 * @details 该函数
 * @note 该函数在构造时会自动调用，因此正常情况（使用大疆电机）不需要手动调用
 * @warning 该函数未重写
 */
void Motor::motorInit(void)
{
	//检测重复定义
	if (isDJIMotor(motorInfo.can_x == Can_1 ? CAN1 : CAN2, motorInfo.std_ID)) //是大疆电机
	{
		otherInfo.isDjiMotorFlag = 1;
		if (DJI_MotorFlag[motorInfo.can_x][motorInfo.std_ID - 0x201] == 0 || (DJI_MotorFlag[motorInfo.can_x][motorInfo.std_ID - 0x201] == 1 && motorList[motorInfo.can_x][motorInfo.std_ID - 0x201].motorPoint == this)) //此前该id未定义
		{
			canBeEnable = 1; //能够被初始化
		}
		else
		{
			canBeEnable = 0; //无法被使能
		}
	}
	else
	{
		otherInfo.isDjiMotorFlag = 0;
		canBeEnable = 1;
	}

	//电机功能使能位
	enableFlag.canSendMsg = canBeEnable; //发送使能
	enableMotor = canBeEnable;

	if (otherInfo.isDjiMotorFlag) //大疆电机
	{
		DJI_MotorFlag[motorInfo.can_x][motorInfo.std_ID - 0x201] = canBeEnable;
		//更新电机列表
		motorList[(u8)motorInfo.can_x][motorInfo.std_ID - 0x201].motorPoint = this;
		motorList[(u8)motorInfo.can_x][motorInfo.std_ID - 0x201].enableFlag = canBeEnable;
		motorList[(u8)motorInfo.can_x][motorInfo.std_ID - 0x201].sendEnableFlag = canBeEnable;
	}
}

/**
 * @details 对输入的电流进行限幅，包括温度限幅、功率限幅、最大值限幅；并将电流值填入canBuff的缓存区中（可选）
 * @param  current          电流值
 * @param  sendFlag         是否将限幅后的输出值填入canBuff对应的缓存区中
 * @return float 经过温度限幅、功率限幅、最大值限幅的电流值
 */
float Motor::ctrlCurrent(float current, u8 sendFlag)
{
	//输出限幅
	if (this->otherInfo.isDjiMotorFlag)
	{
		current = LIMIT(current, -motorCurrentLimitList[motorInfo.type], motorCurrentLimitList[motorInfo.type]);
	}
	//过热保护
	if (enableFlag.overTempProtect == 1)
	{
		this->overHeatProtect(canInfo.temperature);
		current *= otherInfo.overHeatKp;
	}
	// 功率输出限幅系数
	if (enableFlag.powerLimit == 1)
	{
		current *= *otherInfo.powerOutKp;
	}

	//如果can发送使能且电机在线
	if (sendFlag && enableFlag.canSendMsg)
	{
		//非大疆电机调用自定义发送函数
		if (this->otherInfo.isDjiMotorFlag == 0)
		{
			//参数检查
			if (this->otherMotorFunction != 0)
			{
				otherMotorFunction(current, this);
			}
		}
		else if (otherInfo.isDjiMotorFlag && canInfo.lostFlag || deforceFlag) //电机未丢失并且未脱力
		{
			/*结果发送到缓存区*/
			motorPowerOut(0); //电机丢失或脱力 发0
			return 0;
		}
		else
		{
			motorPowerOut(current); //
		}
	}
	return current;
}

/**
 * @details 将设定的速度值通过pid计算后，将pid输出值传入ctrlCurrent
 * @param  speed            速度设定值
 * @param  planIndex        方案号
 * @param  sendFlag         是否自动调用发送函数（传入sendFlag中）
 * @return float 经过内环pid控制，以及温度限幅、功率限幅、最大值限幅的 \b 电流值
 */
float Motor::ctrlSpeed(float speed, int8_t planIndex, u8 sendFlag)
{
	/*电流输出*/
	//DJI电机丢失发0
	if ((otherInfo.isDjiMotorFlag && canInfo.lostFlag))
	{
		pidInner.Clear();
		return ctrlCurrent(0, sendFlag);
	}
	/*PID计算，默认输出到对象的result变量*/
	pidInner.Calculate(speed, planIndex);
	return ctrlCurrent(pidInner.result, sendFlag);
}

/**
 * @details 将设定的位置通过pid计算后，将pid输出值传入ctrlSpeed
 * @param  position         位置设定值
 * @param  planIndex        方案号
 * @param  sendFlag         是否自动调用发送函数
 * @return float 经过双环pid控制，以及温度限幅、功率限幅、最大值限幅的电流值
 */
float Motor::ctrlPosition(double position, int8_t planIndex, u8 sendFlag)
{
	/*内环*/
	//todo PositiveFeedback
	//DJI电机丢失发0
	if ((otherInfo.isDjiMotorFlag && canInfo.lostFlag))
	{
		pidOuter.Clear();
		return ctrlSpeed(0, 0, sendFlag);
	}

	/*外环*/
	if (++otherInfo.pidOuterCnt >= 2)
	{
		otherInfo.pidOuterCnt = 0;
		/*PID计算，默认输出到对象的result变量*/
		pidOuter.Calculate(position, planIndex);
	}
	return ctrlSpeed(pidOuter.result, planIndex, sendFlag);
}

/**
 * @param  canResult        电流值
 */
void Motor::motorPowerOut(float canResult)
{
	//根据can口和id填充缓冲区并清空计数器和标志位
	canBuff[motorInfo.can_x][motorInfo.std_ID - 0x201] = canResult;				//填充缓冲区
	motorList[motorInfo.can_x][motorInfo.std_ID - 0x201].updateOfflineCnt = 0;	//清空计数器
	motorList[motorInfo.can_x][motorInfo.std_ID - 0x201].updateOfflineFlag = 0; //清空标志位
}

/**
  * @param[in] outFun 结果输出,电机编号
  * @note 电机编号用于提供多个电机共用一个发送函数时区分不同电机用
  * @warning 未对此函数进行测试
  */
void Motor::setOutFunction(OutFunction outFun)
{
	otherMotorFunction = outFun;
}

/**
 * @param  _CANx            CAN口
 * @param  _Std_ID          can_id
 * @return u8 是否为大疆电机
 */
u8 Motor::isDJIMotor(CAN_TypeDef *_CANx, uint32_t _Std_ID)
{
	if (_CANx != CAN1 && _CANx != CAN2)
	{
		return 0;
	}
	else
	{
		if (_Std_ID < 0x201 || _Std_ID > 0x208)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
}

///交换两个变量的值，仅限于整形相同变量类型
#define Change_Value(a, b) \
	a ^= b;                \
	b ^= a, a ^= b;

/**
 * @param  reSetSpeed       每次调用此函数，外环设定值的增量
 * @param  maxErr           外环最大偏差
 * @param  outLimit         输出限幅，若默认则为电流输出限幅的1/3
 * @return u8 返回是否已经堵转
 * @warning 只对方案0适用
 * @warning 务必判断返回的堵转标志位，堵转立刻退出
 */
u8 Motor::ctrlMotorOffset(float reSetSpeed, float maxErr, float outLimit)
{
	if (outLimit != 0)
	{
		//更新输出限幅
		this->otherInfo.offSetLimit = outLimit;
	}

	/*校零开始*/
	if (otherInfo.posOffSetFlag == 0) //获取初始角
	{
		//更改PID输出限幅为校零模式
		pidOuter.paramPtr[0].resultMax = otherInfo.offSetLimit;
		//获取外环信息
		otherInfo.offSetPos = *pidOuter.fbValuePtr[0];
		otherInfo.posOffSetFlag = 1;
	}
	else if (otherInfo.posOffSetFlag == 1) //开始反转
	{
		otherInfo.offSetPos += reSetSpeed;
	}

	/*校零结束*/
	if (pidOuter.Is_LockTurn(maxErr) == 1) //当堵转时
	{
		int16_t offsetEncoderBuf = canInfo.offsetEncoder;
		memset(&canInfo, 0, sizeof(CanInfo));
		canInfo.offsetEncoder = offsetEncoderBuf;

		pidOuter.Clear();
		pidOuter.Clear();
		otherInfo.posOffSetFlag = 0;
		//恢复PID输出限幅为正常模式
		pidOuter.paramPtr[0].resultMax = otherInfo.currentLimit;
		return 1;
	}
	else
	{
		ctrlPosition(otherInfo.offSetPos);
	}
	return 0;
}

/**
 * @param  temp             当前温度
 */
void Motor::overHeatProtect(int16_t temp)
{
	otherInfo.overHeatKp = (float)(1.0f - ((temp - otherInfo.criticalTemp) / (otherInfo.maxOverTemp - otherInfo.criticalTemp)));
	otherInfo.overHeatKp = LIMIT(otherInfo.overHeatKp, 0, 1);
}

/********************/

/**
 * @details 将零点值更改
 * @warning 该操作可能会导致圈数多1或少1，建议在初始化时使用。下版本可能会修复此问题
 * @todo 修复圈数可能多1或少1的问题（同步更改lastEncoderCalibration）
 * @param  offset           电机机械零点值
 */
void Motor::setZeroValue(uint16_t offset)
{
	canInfo.offsetEncoder = offset;

	canInfo.encoderCalibration = canInfo.encoder - canInfo.offsetEncoder;
	if (canInfo.encoderCalibration > 4096)
	{
		canInfo.encoderCalibration -= 8192;
	}
	else if (canInfo.encoderCalibration < -4096)
	{
		canInfo.encoderCalibration += 8192;
	}
	canInfo.encoderCalibration = canInfo.encoder - canInfo.offsetEncoder;
	if (canInfo.encoderCalibration > 4096)
	{
		canInfo.encoderCalibration -= 8192;
	}
	else if (canInfo.encoderCalibration < -4096)
	{
		canInfo.encoderCalibration += 8192;
	}
}

/**
 * @details 调用后会在控制电流时将该指针的值乘到电流中
 * @warning - 不建议传入局部变量的地址。如果确实需要，请保证直接或间接调用ctrlCurrent的过程在该局部变量的生命周期中，否则请使用该函数更新地址
 * - 该函数不对传入的地址进行有效检查
 * @param  Kp               功率限幅指针
 */
void Motor::setPowerOutLimit(float *Kp)
{
	enableFlag.powerLimit = 1;
	otherInfo.powerOutKp = Kp;
}

/**
 * @param  state            是否使能该电机
 */
void Motor::setMotorState(FunctionalState state)
{
	if (state == ENABLE)
	{
		motorInit();
	}
	else
	{
		enableMotor = 0;
		motorList[(u8)motorInfo.can_x][motorInfo.std_ID - 0x201].enableFlag = 0;
	}
}

/**
 * @return MotorType 电机类型
 */
MotorType Motor::getType(void)
{
	return motorInfo.type;
}
/**
 * @return Can_x can口
 */
Can_x Motor::getCan_x(void)
{
	return motorInfo.can_x;
}
/**
 * @return uint32_t 电机can_id
 */
uint32_t Motor::getStd_Id(void)
{
	return motorInfo.std_ID;
}

/**
 * @return uint8_t 获取电机是否使能
 */
uint8_t Motor::getEnableMotor(void)
{
	return enableMotor;
}

/**
 * @return uint16_t 该对象对应电机的电流最大值
 */
uint16_t Motor::getMotorCurrentLimit(void)
{
	return motorCurrentLimitList[motorInfo.type];
}
/**
 * @param  motorType        指定电机类型
 * @return uint16_t 指定电机类型的电流最大值
 */
uint16_t Motor::getMotorCurrentLimit(MotorType motorType)
{
	return motorCurrentLimitList[motorType];
}

/**
 * @return uint16_t 该对象对应电机的速度最大值
 */
uint16_t Motor::getMotorSpeedLimit(void)
{
	return motorSpeedLimitList[motorInfo.type];
}
/**
 * @param  motorType        指定电机类型
 * @return uint16_t 指定电机类型的速度最大值
 */
uint16_t Motor::getMotorSpeedLimit(MotorType motorType)
{
	return motorSpeedLimitList[motorType];
}

/**
 * @return uint16_t 该对象对应电机的温度最大值
 */
uint16_t Motor::getMotorMaxTemp(void)
{
	return motorMaxTempList[motorInfo.type];
}
/**
 * @param  motorType        指定电机类型
 * @return uint16_t 指定电机类型的温度最大值
 */
uint16_t Motor::getMotorMaxTemp(MotorType motorType)
{
	return motorMaxTempList[motorType];
}

/**
 * @brief 定时器回调函数，can发送
 * @param  xTimer           哪个定时器
 */
void Motor::sendCanMsg()
{
	//can1 or can2
	for (int can_i = 0; can_i < 2; can_i++)
	{
		//0~3 OR 4~7
		for (int idIndex = 0; idIndex < 2; idIndex++)
		{
			bool containMotor = false;
			//遍历4个电机
			for (int idOffset = 0; idOffset < 4; idOffset++)
			{
				//未定义
				if(motorList[can_i][idIndex * 4 + idOffset].motorPoint  == 0)
					continue;
				if(motorList[can_i][idIndex * 4 + idOffset].motorPoint->enableMotor == 0)
					continue;

				containMotor = true;
				/*电机离线检测部分*/
				if(motorList[can_i][idIndex * 4 + idOffset].motorPoint->canInfo.lostCnt > 5)//两百个周期都没有接收到
				{
					motorList[can_i][idIndex * 4 + idOffset].motorPoint->canInfo.lostFlag = 1;//离线
					canBuff[can_i][idIndex * 4 + idOffset] = 0; //自动脱力该电机
					continue;
				}
				motorList[can_i][idIndex * 4 + idOffset].motorPoint->canInfo.lostCnt++;

				//超过100ms没有刷新计数器，认为没有调用控制/发送函数，自动脱力该电机
				if (motorList[can_i][idIndex * 4 + idOffset].updateOfflineCnt > 100) //100ms
				{
					motorList[can_i][idIndex * 4 + idOffset].updateOfflineFlag = 1;
					canBuff[can_i][idIndex * 4 + idOffset] = 0; //自动脱力该电机
					continue;
				}
				if(deforceFlag)
				{
					canBuff[can_i][idIndex * 4 + idOffset] = 0; //自动脱力该电机
				}
				else
				{
					motorList[can_i][idIndex * 4 + idOffset].updateOfflineCnt++;
				}

			}
			if (!containMotor) //不包含电机，跳过
				continue;

			canTx(&canBuff[can_i][idIndex * 4], can_i == 0 ? CAN1 : CAN2, idIndex == 0 ? 0x200 : 0x1ff);
		}
	}
}
