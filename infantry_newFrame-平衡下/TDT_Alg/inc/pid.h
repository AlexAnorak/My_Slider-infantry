/*****************************************************************************
File name: TDT_Alg\src\pid.h
Description: PID算法
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.19 增加微分时间的处理
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef _PID_H
#define _PID_H

#include <stdint.h>
#include "stm32f4xx.h"
#include "my_math.h"
#include "cycle.h"
#include "cycle.h"

/**
 * @addtogroup TDT_ALG_PID
 * @{
 */

typedef struct _FbValuePtr
{
	void *fbValuePtr; ///<存放反馈值的指针
	///数据类型
	enum
	{
		type_u8 = 0,
		type_u16,
		type_u32,
		type_u64,
		type_i8,
		type_i16,
		type_i32,
		type_i64,
		type_float,
		type_double
	} dataType;

	double directOrAmp; ///<缩放因子

public:
	///@brief 运算符重载 根据=后面的类型执行对应的函数
	///@param fbValuePtr 反馈值的指针
	///@{
	void operator=(int8_t *fbValuePtr);
	void operator=(int16_t *fbValuePtr);
	void operator=(int32_t *fbValuePtr);
	void operator=(int64_t *fbValuePtr);
	void operator=(uint8_t *fbValuePtr);
	void operator=(uint16_t *fbValuePtr);
	void operator=(uint32_t *fbValuePtr);
	void operator=(uint64_t *fbValuePtr);
	void operator=(float *fbValuePtr);
	void operator=(double *fbValuePtr);
	///@}

	///正负号或者对fbValue的值进行放大缩小
	void operator*=(double directOrAmp);

	///重载操作符：返回反馈值
	double operator*();
	///重载操作符：判断fbValuePtr是否为空
	bool operator!();

	///重载构造器：设定fbValuePtr并赋予dataType初值
	///@param fbValuePtr 反馈值的指针
	///@{
	_FbValuePtr();
	_FbValuePtr(int8_t *fbValuePtr);
	_FbValuePtr(int16_t *fbValuePtr);
	_FbValuePtr(int32_t *fbValuePtr);
	_FbValuePtr(int64_t *fbValuePtr);
	_FbValuePtr(uint8_t *fbValuePtr);
	_FbValuePtr(uint16_t *fbValuePtr);
	_FbValuePtr(uint32_t *fbValuePtr);
	_FbValuePtr(uint64_t *fbValuePtr);
	_FbValuePtr(float *fbValuePtr);
	_FbValuePtr(double *fbValuePtr);
	///@}
} FbValuePtr;

///Pid参数存储结构体
typedef struct _PidParam
{
	float kp;				   ///<比例系数
	float ki;				   ///<积分系数
	float kd;				   ///<微分系数
	float integralErrorMax;	   ///<积分限幅
	float resultMax;		   ///<最终结果限幅
	u8 positiveFBFlag;		   ///<正反馈标志位，为1时输出会*(-1)1
	float integralThreshold;   ///<积分分离阈值
	float kpAmpForIntegralSep; ///<误差大于积分分离阈值时的放大系数
	float kiAmpForIntegralSep; ///<误差大于积分分离阈值时的放大系数
	u8 integralMethod;		   ///<积分方法【正常=0，积分分离=1，变积分=2, 变积分2 =3】
	u8 differentialMethod;	   ///<微分算法【正常=0，微分先行=1，加低通滤波】
	///以下参数是更加高级也更加少用的参数
	///@warning 部分参数及算法未测试
	struct
	{
		float VIntegralIndex; ///<变积分系数
		float DLPFIndex;	  ///<微分项低通滤波系数
		float S_Buff;		  ///<变积分增益
		float MAXlimit;		  ///<变积分上极限
		float MINlimit;		  ///<变积分下极限
		float alpha;		  ///<不完全微分系数
	} advancedPara;
} PidParam;

///Pid计算类
struct Pid
{
public:
	/*计算数据*/
	double setValue;
	double fbValue;
	float lastError;	  ///<上次偏差——lastError = [lastError = error]
	float integralError;  ///<偏差积分 += [该次偏差 * 时间]——integralError += [error * T]
	float deltaError;	  ///<偏差增量 = [该次偏差 - 上次偏差] / 时间——  deltaError	 = [error - lastError]
	float lastDeltaError; ///<上次偏差增量 = [上次偏差 - 上上次偏差] / 时间—— deltaError = [error - lastError]

public:
	float error;  ///<该次偏差 = [设定值 - 反馈值]——error = [setValue - feedbackValue]
	float result; ///<限幅最终输出后结果

	PidParam *paramPtr;
	FbValuePtr *fbValuePtr;

	//构造器
	Pid();
	Pid(int planNum); //构造器

	/*装载PID参数【pid参数数组首地址或pid指针】*/
	int getPlanNum();
	void setPlanNum(int planNum);

	//检查PID参数
	bool LoadCheck(int planIndex = 0);
	//清空相关变量
	void Clear();
	//PID算法实现
	float Calculate(double _setValue, double _fbValue, int planIndex = 0, float *_result = 0, float _T = 0);
	float Calculate(double _setValue, int planIndex = 0, float *result = 0, float T_ = 0);

	//获取反馈值
	double getFbValue(int planIndex = 0);

	//电机堵转检测【最大误差】
	u8 Is_LockTurn(float maxErr);

private:
	/*获取周期时间*/
	Cycle pidCycle;

	PidParam *nowParamPtr; //当前方案号的pid参数
	int nowPlanIndex;	   //当前方案号
	int planNum;		   //方案数量
	int lastPlanIndex;	   //上一次运算的方案号
};

/**
 * @}
 */

#endif
