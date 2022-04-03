/*****************************************************************************
File name: TDT_Device\inc\motor.h
Author: 郑俊元
Version: 1.3.1.191119_alpha
Date: 19.10.15
History: 
	——————————————————————————————————————————————————————————————————————————
	参考Readme.md
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "board.h"
#include "pid.h"
#include "can_calculate.h"
#include <stdint.h>

/**
 * @addtogroup TDT_Motor
 * @{
 */

///电机can口枚举定义，用于motorList指定索引
enum Can_x
{
	Can_1 = 0, ///<CAN1
	Can_2 = 1  ///<CAN2
};

///电机类型-带G的为云台电机，用于CanInfo根据电机类型进行解析
enum MotorType
{
	M2006,
	M3508,
	M3510,
	GM3510,
	GM6020
};

class Motor
{
public:
	///Pid内环
	Pid pidInner;
	///Pid外环
	Pid pidOuter;

	///Can信息集合
	CanInfo canInfo;

	///创建电机对象，并进行初始化（能进行数据接收）
	Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t _Std_ID);

	///对电机进行初始化,包括CAN使能
	void motorInit(void);

	/*▲ 电机控制方法*/

	///控制电流
	float ctrlCurrent(float current, u8 sendFlag = 1);

	///控制速度
	float ctrlSpeed(float speed, int8_t planIndex = 0, u8 sendFlag = 1);

	///控制位置
	float ctrlPosition(double position, int8_t planIndex = 0, u8 sendFlag = 1);

	///电机零点矫正
	u8 ctrlMotorOffset(float reSetSpeed, float maxErr, float outLimit = 0);

	/*▲ 电机设置方法*/

	///设置电机机械零点值
	void setZeroValue(uint16_t offset);

	///设置功率限幅指针
	void setPowerOutLimit(float *Kp);

	///非大疆电机输出函数
	typedef void (*OutFunction)(float, Motor *);

	///非大疆电机的发送函数设置[结果输出]
	void setOutFunction(OutFunction);

	///设置是否使能该电机
	void setMotorState(FunctionalState state);

	/*▲ 私有变量访问接口*/

	///获取电机类型
	MotorType getType(void);

	///获取can口
	Can_x getCan_x(void);

	///获取电机can_id
	uint32_t getStd_Id(void);

	///获取获取电机是否使能
	uint8_t getEnableMotor(void);

	///获取该对象对应电机的电流最大值
	uint16_t getMotorCurrentLimit(void);

	///获取指定电机类型的电流最大值
	static uint16_t getMotorCurrentLimit(MotorType motorType);

	///获取该对象对应电机的速度最大值
	uint16_t getMotorSpeedLimit(void);

	///获取指定电机类型的速度最大值
	static uint16_t getMotorSpeedLimit(MotorType motorType);

	///获取该对象对应电机的温度最大值
	uint16_t getMotorMaxTemp(void);

	///获取指定电机类型的温度最大值
	static uint16_t getMotorMaxTemp(MotorType motorType);

	///定时器回调函数，can发送
	static void sendCanMsg();

private:
	///电机常规参数
	///结构体-电机基本信息-初始化时填充，只允许构造器赋值
	struct _motorInfo
	{
		MotorType type;	 ///<电机类型
		Can_x can_x;	 ///<挂载CAN总线[CAN_1 / CAN_2]
		uint32_t std_ID; ///<电机CAN总线反馈StdID
	} motorInfo;
	///电机额外辅助数据
	///结构体-电机基本信息-初始化时填充，只允许构造器赋值
	struct
	{
		u8 pidOuterCnt;	   ///<外环计次-用于外环
		u8 posOffSetFlag;  ///<位置校零标志位
		double offSetPos;  ///<位置校零设定值变量
		float overHeatKp;  ///<电机过热保护系数，作用于内环输出
		float *powerOutKp; ///<功率控制系数，作用于内环输出 , 功率输出限幅系数
		//以下变量初始化函数中进行赋默认值
		float currentLimit; ///<电流限制
		float speedLimit;	///<速度限制
		float offSetLimit;	///<位置校零输出限幅系数
		float tempLimit;	///<温度限制
		float criticalTemp; ///<临界温度
		float maxOverTemp;	///<最大超出温度
		u8 isDjiMotorFlag;	///<否是DJI电机的标志位
	} otherInfo;
	///功能使能位
	struct
	{
		u8 overTempProtect; ///<过热保护
		u8 canSendMsg;		///<数据输出
		u8 speedToMech;		///<机械角与速度积分合成
		u8 powerLimit;		///<使用功率系数powerKp
	} enableFlag;

	///电机电流发送值
	void motorPowerOut(float canResult);

	///对温度进行限幅计算
	void overHeatProtect(int16_t temp);

	///是否为大疆电机
	u8 isDJIMotor(CAN_TypeDef *_CANx, uint32_t _Std_ID);
	static u8 DJI_MotorFlag[2][8]; ///<用来检查std_id是否重复
	u8 canBeEnable;				   ///<为1才可初始化电机	为1条件：DJI电机: std_id：0x201-0x208，std_id不重复。非DJI电机: std_id不在0x201-0x20B
	u8 enableMotor;				   ///<电机使能标志位

	///非大疆电机的发送函数指针[结果输出,电机对象地址(用于提供多个电机共用一个发送函数时区分不同电机用)]
	void (*otherMotorFunction)(float, Motor *);

	static float canBuff[2][8]; ///<电机CAN信息缓冲区
};

///结构体：电机列表，用于通过CAN口和ID检索（轮询）所有电机
typedef struct
{
	Motor *motorPoint;	  ///<电机对象指针
	u8 enableFlag;		  ///<电机使能标志位
	u8 sendEnableFlag;	  ///<电机发送使能标志位
	u16 updateOfflineCnt; ///<电机更新离线计数器
	u8 updateOfflineFlag; ///<电机更新离线标志位
} MotorList;

///电机对象标记
extern MotorList motorList[2][8];

/** @} */

///最小线性二乘法拟合线性方程
class Linear
{
private:
	uint16_t getValueCnt;
	double l[4];

public:
	float kA;
	float kB;
	Linear()
	{
		getValueCnt = 1;
		memset(l, 0, sizeof(l));
	}
	void clear(void)
	{
		getValueCnt = 1;
		memset(l, 0, sizeof(l));
	}
	void linearRegression(float valueX, float valueY, float valueNum, float *_aResult = 0, float *_bResult = 0, u8 resetFlag = 0)
	{
		if (resetFlag == 1)
		{
			Linear();
		}
		if (++getValueCnt < valueNum)
		{
			l[0] += valueX;
			l[1] += valueY;
			l[2] += valueX * valueY;
			l[3] += valueX * valueX;
		}
		else
		{
			kA = (valueNum * l[2] - l[0] * l[1]) / (valueNum * l[3] - l[0] * l[0]);
			kB = l[1] / valueNum - kA * l[0] / valueNum;
			if (_aResult != 0 && _bResult != 0)
			{
				*_aResult = kA;
				*_bResult = kB;
			}
		}
	}
};

#endif
