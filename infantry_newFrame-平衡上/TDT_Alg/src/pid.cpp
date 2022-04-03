/******************************
File name: TDT_Alg\src\pid.cpp
Description: PID算法
function:
	——————————————————————————————————————————————————————————————————————————
	
	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.20 肖银河-梯形积分PID控制器：提升积分精度
	——————————————————————————————————————————————————————————————————————————
	19.11.19 肖银河-增加微分时间的处理
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "pid.h"

/*笔记：
√1、	经典PID 
√2、	积分分离PID
√3、	抗积分饱和：如果已经饱和，只累计反向的
√4、	梯形积分PID控制器：提升积分精度，将矩形积分改为梯形积分可以提高运算精度
√5、	变积分PID控制器：根据系统的偏差大小改变积分速度
6、	不完全微分PID控制器：那就是容易引进高频干扰，在偏差扰动突变时尤其显出微分项的不足。低通滤波方式来解决这一问题
7、 微分先行PID
*/

/**
 * @ingroup TDT_ALG
 * @defgroup TDT_ALG_PID PID类
 * @details 由于许多套pid使用时产生了内存浪费的问题，使用 PidParam 的指针。同时，为方便pid直接取得反馈值，使用了 FbValuePtr 作为随取的指针。因此PID类的难点也在于如何使用这两个指针
 * @sa PidParam 
 * @sa FbValuePtr 
 * 
 * - 初始化
 * @code {.cpp}
 * Pid pidInner(x);//设置x个方案
 * @endcode
 * 或
 * @code {.cpp}
 * Pid pidInner;
 * void init()
 * {
 * 	pidInner.setPlanNum(x);//设置x个方案
 * 	//...
 * }
 * @endcode
 * - 参数加载
 * Pid类会通过setPlanNum管理fbValuePtr，但不会管理paramPtr；因此：  
 * 对于paramPtr，需要外部定义PidParam结构体（数组）并将其首地址填入paramPtr中；  
 * 对于fbValuePtr，仅需对fbValuePtr的每个元素都填入即可
 * 	- 加载1个
 * @code {.cpp}
 * PidParam pidParam;
 * void init()
 * {
 * 	//...//设置pid参数
 * 	pidInner.setPlanNum(1);//设置1个方案
 * 	pinInner.paramPtr = &pidParam;//加载
 * 	pidInner.fbValuePtr[0] = &imuData;//第0套方案采用陀螺仪数据
 * }
 * @endcode
 * 	- 加载多个
 * @code {.cpp}
 * PidParam pidParam[x];
 * void init()
 * {
 * 	//...//设置pid参数
 * 	pidInner.setPlanNum(x);//设置x个方案
 * 	pinInner.paramPtr = pidParam;//加载
 *  for(int i = 0; i < x; i++)
 * 	{
 * 		pidInner.fbValuePtr[i] = &imuData;//第0~x-1套方案采用陀螺仪数据
 * 	}
 * }
 * @endcode
 * 或
 * @code {.cpp}
 * PidParam *pidParam = nullptr;
 * void init()
 * {
 * 	pidParam = new PidParam[4];
 * 	//...//设置pid参数
 * 	pidInner.setPlanNum(4);//设置x个方案
 * 	pidInner.paramPtr = pidParam;//加载
 * 	for(int i = 0; i < 4; i++)
 * 	{
 * 		pidInner.fbValuePtr[i] = &imuData;//第0~3套方案采用陀螺仪数据陀螺仪数据
 * 	}
 * }
 * @endcode
 * - 使用
 * @code {.cpp}
 * void run()
 * {
 * 	float result = 0;
 * 	result = pidInner.Calculate(setValue, 0);//仅有一个方案，故为方案0；使用 fbValuePtr 自动获取反馈值；不需要传出result指针
 * }
 * @endcode
 * @warning 如果反馈值是整形，又需要传入参数，必须 @b 强制转换成浮点 ，或者至少 @b 反馈值和方案号这两个参数都填上  
 * 注意下方各个参数的区别
 * @code {.cpp}
 *	pidInner.Calculate(setValue, (float)int_fbValue);//反馈值为int_fbValue；方案号使用默认（0）
 *	pidInner.Calculate(setValue, int_fbValue);//使用int_fbValue方案号
 *	pidInner.Calculate(setValue, 0.0);//反馈值为(double)0.0；方案号使用默认（0）
 *	pidInner.Calculate(setValue, 0, 1);//使用外传入的反馈值参数，使用方案号1
 * @endcode
 */

void _FbValuePtr::operator=(uint8_t *fbValuePtr)
{
	dataType = type_u8;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(uint16_t *fbValuePtr)
{
	dataType = type_u16;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(uint32_t *fbValuePtr)
{
	dataType = type_u32;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(uint64_t *fbValuePtr)
{
	dataType = type_u64;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(int8_t *fbValuePtr)
{
	dataType = type_i8;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(int16_t *fbValuePtr)
{
	dataType = type_i16;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(int32_t *fbValuePtr)
{
	dataType = type_i32;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(int64_t *fbValuePtr)
{
	dataType = type_i64;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(float *fbValuePtr)
{
	dataType = type_float;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(double *fbValuePtr)
{
	dataType = type_double;
	this->fbValuePtr = fbValuePtr;
}

//正负号或者对fbValue的值进行放大缩小
void _FbValuePtr::operator*=(double directOrAmp)
{
	this->directOrAmp *= directOrAmp;
}

//运算符重载 根据dataType确定fbValuePtr对应的函数
double _FbValuePtr::operator*()
{
	if (!(*this))
	{
		return 0;
	}
	switch (dataType)
	{
	case type_u8:
		return directOrAmp * ((*((uint8_t *)fbValuePtr)));
	case type_u16:
		return directOrAmp * (*((uint16_t *)fbValuePtr));
	case type_u32:
		return directOrAmp * (*((uint32_t *)fbValuePtr));
	case type_u64:
		return directOrAmp * (*((uint64_t *)fbValuePtr));
	case type_i8:
		return directOrAmp * (*((int8_t *)fbValuePtr));
	case type_i16:
		return directOrAmp * (*((int16_t *)fbValuePtr));
	case type_i32:
		return directOrAmp * (*((int32_t *)fbValuePtr));
	case type_i64:
		return directOrAmp * (*((int64_t *)fbValuePtr));
	case type_float:
		return directOrAmp * (*((float *)fbValuePtr));
	case type_double:
		return directOrAmp * (*((double *)fbValuePtr));
	}
}

//操作符重载 判断指针是否为空
bool _FbValuePtr::operator!()
{
	return fbValuePtr == 0;
}

///@todo 使用模板类重写
_FbValuePtr::_FbValuePtr() : fbValuePtr(0), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(int8_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_u8), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(int16_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_u16), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(int32_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_u32), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(int64_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_u64), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(uint8_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_i8), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(uint16_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_i16), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(uint32_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_i32), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(uint64_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_i64), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(float *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_float), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(double *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_double), directOrAmp(1) {}

/**
 * @class Pid
 * @copydoc TDT_ALG_PID
 */

/**
  * @brief 构造器
  */
Pid::Pid() : planNum(0), lastPlanIndex(-1), paramPtr(0), fbValuePtr(0)
{
}

/**
  * @brief Pid类构造器，初始化传入参数
  * @param[in] planNum pid参数指针
  * @note 自定义Pid时可以直接传入参数
  */
Pid::Pid(int planNum) : planNum(planNum),
						lastPlanIndex(-1),
						paramPtr(0),fbValuePtr(0)
{
	if(planNum > 0)
		fbValuePtr = new _FbValuePtr[planNum];
}

/**
  * @brief 装载默认PID参数,数据载入new指针
  * @param[in] planNum &PID数据地址
  */
void Pid::setPlanNum(int planNum)
{
	if(this->planNum == planNum)
		return;
	if(this->planNum == 0)
	{
		fbValuePtr = new _FbValuePtr[planNum];
		this->planNum = planNum;
		return;
	}
	if(planNum == 0)
	{
		delete fbValuePtr;
		planNum = 0;
		return;
	}
	
	auto tmp = new _FbValuePtr[planNum];
	int num = planNum > this->planNum?this->planNum:planNum;
	
	memcpy(tmp,fbValuePtr,num * sizeof(_FbValuePtr));
	
	delete fbValuePtr;
	fbValuePtr = tmp;
	this->planNum = planNum;
}


int Pid::getPlanNum()
{
	return planNum;
}

/**
  * @brief 清零具有累加性质的PID变量
  */
void Pid::Clear()
{
	lastError = 0;
	integralError = 0;
	deltaError = 0;
	result = 0;
	pidCycle.getCycleT();
}

/**
  * @brief 检查PID参数是否加载
  * @return [bool]参数是否加载
  * @note 实际上只检查了输出限幅和积分限幅
  */
bool Pid::LoadCheck(int planIndex)
{
	if (planIndex > planNum || fbValuePtr == 0 || paramPtr == 0 || !fbValuePtr[planIndex])
		return false;
	if (paramPtr[planIndex].resultMax == 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

/**
  * @brief PID计算函数
  * @param _setValue 设定值
  * @param _fbValue 反馈值
  * @param planIndex 方案号
  * @param _result 结果指针（默认空）
  * @param _T 周期（函数内可自己获取）
  * @return [bool]参数是否加载
  * @note 实际上只检查了输出限幅和积分限幅
  */
float Pid::Calculate(double _setValue, double _fbValue, int planIndex, float *_result, float _T)
{
	if(!LoadCheck(planIndex))
		return 0;
	if (planIndex != lastPlanIndex)//方案号更改时
	{
		nowPlanIndex = planIndex;			
		nowParamPtr = paramPtr+planIndex;

		lastPlanIndex = planIndex;
		Clear();
		return 0;
	}


	//默认结果地址
	if (_result == 0)
	{
		_result = &(this->result);
	}
	/* 设定值 */
	setValue = _setValue;
	/* 反馈值 */
	fbValue = _fbValue;
	/* 偏差 = 设定值 - 反馈值 */
	error = setValue - fbValue;
	
	//默认周期时间
	if (_T == 0)
	{
		_T = pidCycle.getCycleT();
		//如果间隔时间过长, 不计算积分微分
		if (_T > 0.1f) //100ms
		{
			Clear();
			result = paramPtr[planIndex].kp * error;
			if (this->paramPtr[planIndex].positiveFBFlag == 1)
			{
				result *= -1;
			}
			/* 总的输出不能超出电机给定值的范围 */
			result = LIMIT(result, -paramPtr[planIndex].resultMax, paramPtr[planIndex].resultMax);
			//输出
			*_result = result;
			return result;
		}
	}
	
	/* 偏差进行积分 */
	//正常积分
	if (paramPtr[planIndex].integralMethod == 0)
	{
		integralError += 0.5f * (error + lastError) * _T;
	}
	//积分分离
	else if (paramPtr[planIndex].integralMethod == 1)
	{
		if (ABS(error) < paramPtr[planIndex].integralThreshold)
		{
			integralError += 0.5f * (error + lastError) * _T;
		}
		else
		{
			integralError = 0;
			if(paramPtr[planIndex].kpAmpForIntegralSep > 0)
				error *= ABS(error) / paramPtr[planIndex].integralThreshold * paramPtr[planIndex].kpAmpForIntegralSep;
		}
	}
	//变积分
	else if (paramPtr[planIndex].integralMethod == 2)
	{
		integralError += 0.5f * (error + lastError) * _T * LIMIT(1 - error / paramPtr[planIndex].advancedPara.VIntegralIndex, 0, 1);
	}

	/* 偏差的积分进行限制 -抗饱和*/
	integralError = LIMIT(integralError, -paramPtr[planIndex].integralErrorMax, paramPtr[planIndex].integralErrorMax);
	
	if (_T != 0)
	{
		//普通微分
		if(paramPtr[planIndex].differentialMethod == 0)
		{
			/* 偏差增量 */
			deltaError = (error - lastError) / _T;
			/* 记录本次误差 */
			lastError = error;
		}
		//微分先行
		else if(paramPtr[planIndex].differentialMethod == 1)
		{
			/* 偏差增量 */
			deltaError = (lastError - fbValue) / _T;
			/* 记录本次误差 */
			lastError = fbValue;
		}
	}
	
	/* 总的输出 = 比例项的输出 + 积分项的输出 + 微分项的输出 */
	result = paramPtr[planIndex].kp * error + paramPtr[planIndex].ki * integralError + paramPtr[planIndex].kd * ((1 - paramPtr[planIndex].advancedPara.DLPFIndex) * deltaError + paramPtr[planIndex].advancedPara.DLPFIndex * lastDeltaError);

	if (this->paramPtr[planIndex].positiveFBFlag == 1)
	{
		result *= -1;
	}
	lastDeltaError = deltaError;
	/* 总的输出不能超出电机给定值的范围 */
	result = LIMIT(result, -paramPtr[planIndex].resultMax, paramPtr[planIndex].resultMax);
	//输出
	*_result = result;
	return result;
}

float Pid::Calculate(double _setValue, int planIndex, float *result, float T_)
{
	if(!LoadCheck(planIndex))
		return 0;
	return Calculate(_setValue, *fbValuePtr[planIndex], planIndex, result, T_);
}

double Pid::getFbValue(int planIndex)
{
	if (planIndex >= planNum || planIndex < 0)
		return 0;
	return *fbValuePtr[planIndex];
}

//电机堵转检测【最大误差】
u8 Pid::Is_LockTurn(float maxErr)
{
	if (ABS(error) > maxErr)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
