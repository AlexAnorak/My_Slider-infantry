/*****************************************************************************
File name: TDT_Alg\src\curve_model.h
Description: 曲线模型
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.20
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.20 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __CURVE_MODEL_H__
#define __CURVE_MODEL_H__

#include "board.h"
#include <typeinfo>
#include "cycle.h"
/***************LED GPIO定义******************/
typedef struct{
	float nowValue;		//当前值
	float startValue;	//开始值
	float useTime;		//总时间宽度，决定曲线速度，即斜率
	float nowTime;		//当前时间，决定当前输出值
	float errValue;		//曲线高度
}CurveValue;



//Sin Cos曲线
#define PI 3.14159265358979f


//曲线模型基类，负责输入值的处理
class Curve{
	
	private:
	CurveValue New;
	CurveValue Old;
	float dT;			//单位时间宽度
	float lastSetValue;	//上次设定值-设定值跳变检测用
	Cycle CurveCycle;
	
public:
	float (*curve)(float,float);	//函数指针
	Curve();
	u8 CurveModel(float _setValue,float* _nowValue,float _valueWidth,float planNum=0);
	void Clear();
	
};

//子类：五次三项
class FivePower:public Curve
{
private:
	//曲线的具体实现
	static float Curve_Model(float nowTime,float useTime);
public:

	FivePower();

};

//子类-逻辑斯谛
class Logistic:public Curve
{
private:
	//曲线的具体实现
	static float Curve_Model(float nowTime,float useTime);
public:

	Logistic();

};

//子类-正余弦
class SinCos:public Curve
{
private:
	//曲线的具体实现
	static float Curve_Model(float nowTime,float useTime);
public:

	SinCos();

};

//子类-斜坡
class Slop:public Curve
{
private:
	//曲线的具体实现
	static float Curve_Model(float nowTime,float useTime);
public:

	Slop();

};


#endif
