/*****************************************************************************
File name: TDT_Alg\src\my_math.h
Description: 快速数学计算
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.16 肖银河-改写my_deathzoom函数-解决遥控器最大值会小于660问题
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __MY_MATH_H__
#define __MY_MATH_H__

#include "stm32f4xx.h"

/**
 * @ingroup TDT_ALG
 * @defgroup TDT_MY_MATH TDT_MY_MATH
 * @brief 该库包含了各种常用数学计算公式的快速解法
 * @{
 */

/// 取绝对值
#define ABS(x) ((x) > 0 ? (x) : -(x))
/// 输出最接近 (min, max) 的x的值
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
/// 输出a与b的最小值
#define MIN(a, b) ((a) < (b) ? (a) : (b))
/// 输出a与b的最大值
#define MAX(a, b) ((a) > (b) ? (a) : (b))

///tan列表的最小值
#define TAN_MAP_RES 0.003921569f /* (smallest non-zero value in table) */
///rad/°的算术值
#define RAD_PER_DEG 0.017453293f
///tan列表的大小
#define TAN_MAP_SIZE 256
#define MY_PPPIII 3.14159f
#define MY_PPPIII_HALF 1.570796f

///快速正切计算
float fast_atan2(float yy, float xx);
///平方
float my_pow(float a);
///快速开方
float my_sqrt(float number);
///快速正弦
double mx_sin(double rad);
///快速正弦
double my_sin(double rad);
///快速余弦
float my_cos(double rad);
///死区，绝对值小于zoom的xx会返回0，否则返回xx
float my_deathzoom(float xx, float zoom);
///将-540°~540°的角度变为-180°~180°
float To_180_degrees(float xx);
///6值之中返回最大的
float Math_Max(float a, float b, float c, float d, float e, float f);
/** @brief 转速转换
 * @{ */
float rpmToDps(float rpm);
float dpsToRpm(float dps);
float rpmToRadps(float rpm);
float RadpsToRpm(float radps);
/** @} */

float Ramp_function_cos(double Input, double min, double max);
float Ramp_function_sin(double Input, double min, double max);

/** @} */

#endif
