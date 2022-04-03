/******************************
File name: TDT_Alg\src\curve_model.cpp
Description: w曲线模型
function:
	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.20
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.20 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "curve_model.h"
#include "cycle.h"
#include "math.h"
#include "my_math.h"
//#include <string>
//#include <vector>
/*预估包含-缓启模型，正弦函数模型，五次三项模型*/
//斜坡函数贝塞尔曲线逻辑斯特函数

//所有函数输入参数为 [目标值，结果地址,长度（时间），*方案号]
//模型总是0~1的



/*基类-曲线模型*/

Curve::Curve()
{
	memset(&New,0,sizeof(New));
	memset(&Old,0,sizeof(Old));
	lastSetValue=0;
}

//基类
u8 Curve::CurveModel(float _setValue,float* _nowValue,float _valueWidth,float planNum)
{
	//设定值更改-或者曲线计划更改-两个曲线同时运行取其和
	//运行一个新的曲线进行合成跳变曲线
	if(_setValue != lastSetValue)
	{
		//只保证二阶合成，在合成时不允许第三个曲线
		if(New.nowTime == New.useTime)
		{
			//上次曲线还在运行中-运行新的曲线
			if(Old.nowTime < Old.useTime)
			{
				New.nowTime=0;
				New.errValue=_setValue-Old.errValue;
				New.startValue=0;//记录一次开始值
				lastSetValue=_setValue;
				New.useTime=_valueWidth;
			}
			else
			{
				memset(&New,0,sizeof(New));
				Old.nowTime=0;
				Old.errValue=_setValue-*_nowValue;
				Old.startValue=*_nowValue;//记录一次开始值
				lastSetValue=_setValue;
				Old.useTime=_valueWidth;
			}
		}
	}
	dT=CurveCycle.getCycleT();
	if(Old.useTime!=0)
	{
		Old.nowTime+=dT;
		Old.nowTime=LIMIT(Old.nowTime,0,Old.useTime);
		Old.nowValue=Old.startValue + Old.errValue *curve(Old.nowTime,Old.useTime);
	}
	if(New.useTime!=0)
	{
		New.nowTime+=dT;
		New.nowTime=LIMIT(New.nowTime,0,New.useTime);
		New.nowValue=New.startValue + New.errValue *curve(New.nowTime,New.useTime);
	}
	*_nowValue=New.nowValue+Old.nowValue;
	
	if(Old.nowTime == Old.useTime && New.nowTime == New.useTime)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void Curve::Clear()
{
	memset(&New,0,sizeof(New));
	memset(&Old,0,sizeof(Old));
	memset(&CurveCycle,0,sizeof(CurveCycle));
	lastSetValue=0;
	dT=0;
}




/*子类-五次三项式*/

float FivePower::Curve_Model(float nowTime,float useTime)
{
	return 10*pow(nowTime/useTime,3) - 15*pow(nowTime/useTime,4) + 6*pow(nowTime/useTime,5) ;
}


FivePower::FivePower()
{
	this->curve=this->Curve_Model;
}



//子类-逻辑斯谛曲线-指数
float Logistic::Curve_Model(float nowTime,float useTime)
{
	return exp(nowTime/useTime)/(100+exp(nowTime/useTime)) ;
}


Logistic::Logistic()
{
	this->curve=this->Curve_Model;
}



//子类-正余弦
float SinCos::Curve_Model(float nowTime,float useTime)
{
	return sin(nowTime-PI/2)+1 ;
}


SinCos::SinCos()
{
	this->curve=this->Curve_Model;
}





//子类-斜坡函数-一次函数
float Slop::Curve_Model(float nowTime,float useTime)
{
	return  nowTime/useTime;
}


Slop::Slop()
{
	this->curve=this->Curve_Model;
}



//贝塞尔曲线-二维



//通用公式
/*
		Pi	k=0;
Pik=	(i-t)Pi(k-1)+tP(i+1)(k-1)	k=1,2,..n , i=0,1,2..n-k
*/

//class Bezier{
//private:

//	u8 n;//阶数
////	float 
//public:
////阶数n-递归
//void Bezier_Mode()
//{
//	for(u8 i=0;i<n;i++)
//	{
//		
//		
//	}
//	
//}

//};





















