/*****************************************************************************
File name: TDT_Device\src\led.h
Description: LED灯
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __LED_H__
#define __LED_H__

#include "board.h"
/**
 * @addtogroup LED 
 * @{
 */

///LED特殊状态码
typedef enum
{
	///状态码——快闪
	LedES_BlinkFast = -2,
	///状态码——慢闪
	LedES_BlinkSlow = -1,
	///状态码——常亮
	LedES_ConstantLight = -3,
	///状态码——常灭
	LedES_ConstantDark = -4,
	///状态码——该状态码下的优先级无效
	LedES_Disable = 0,
} LedES;

/// @brief 该LED类除了基本的拉高拉低以外，对led基本的用途——状态显示进行了封装，并兼容控制阴极和控制阳极的两种LED不同接法
class Led
{
private:
	uint32_t RCC_AHB1Periph_GPIOx; ///<RCC_LED
	GPIO_TypeDef *GPIOx;		   ///<LED_PORT
	uint16_t GPIO_Pin_x;		   ///<LED_Pin

	int8_t stateLast;	 ///<上一次运行时的状态值
	int8_t stateCounter; ///<状态值计数器

	u8 showState;	   ///<当前led的状态
	u8 LHNagation = 1; ///<是否取反,若(state^LHNagation)则拉低，否则拉高

	///LED亮持续时间
	///@sa stateShow()
	u16 stateOnSpan;
	///LED灭持续时间
	///@sa stateShow()
	u16 stateOffSpan;
	///状态值显示冷却时间
	///@sa stateShow()
	u16 stateResetTime;

	///慢闪持续间隔
	///@sa stateShow()
	u16 slowBlinkInterval;
	///快闪持续间隔
	///@sa stateShow()
	u16 fastBlinkInterval;

	u16 blinkTimer; ///<闪烁计数器

public:
	Led(uint32_t RCC_AHB1Periph_GPIOx, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin_x);

	///@brief led初始化
	///@details 根据构造器出入的参数对led的gpio外设初始化
	void init(void);

	///@brief 设置led输出是否取反 若led期望的亮灭状态与现实不同，建议取反LHNagation
	///@sa LHNagation
	inline void setLHNagation(u8 LHNagation) { this->LHNagation = LHNagation; }

	///@brief 设置stateOnSpan
	///@sa stateOnSpan
	inline void setStateOnSpan(u16 stateOnSpan) { this->stateOnSpan = stateOnSpan; };

	///@brief 设置stateOffSpan
	///@sa stateOffSpan
	inline void setStateOffSpan(u16 stateOffSpan) { this->stateOffSpan = stateOffSpan; };

	///@brief 设置slowBlinkInterval
	///@sa slowBlinkInterval
	inline void setSlowBlinkInterval(u16 slowBlinkInterval) { this->slowBlinkInterval = slowBlinkInterval; };

	///@brief 设置fastBlinkInterval
	///@sa fastBlinkInterval
	inline void setFastBlinkInterval(u16 fastBlinkInterval) { this->fastBlinkInterval = fastBlinkInterval; };

	///@brief 设置stateResetTime
	///@sa stateResetTime
	inline void setStateResetTime(u16 stateResetTime) { this->stateResetTime = stateResetTime; };

	///@brief led闪烁状态控制
	///@param intervalMs 距离上一次多长时间
	///@image html led.jpg
	void stateShow(u16 intervalMs);

	///@brief led的GPIO输出
	///@param state 1为亮, 0为灭
	void show(u8 state = 1);
	///@brief 获取led状态
	inline u8 getState() { return ((GPIOx->ODR & GPIO_Pin_x) == GPIO_Pin_x) ^ LHNagation; };
	///@brief led取反
	inline void toggle() { GPIOx->ODR ^= GPIO_Pin_x; };

	u8 MAX_PRIORITY_NUM;
	int8_t *ErrorCode; ///<状态值缓存区，最多允许同时存在MAX_PRIORITY_NUM个状态值

	///@brief 设置状态值
	///@param priority 优先级
	///@param ES_State 状态值
	///@sa getError()
	void setError(int8_t priority, int8_t ES_State);
	///@brief 设置状态值
	///@param priority 优先级
	///@param ES_State 状态值
	///@sa getError()
	///@sa LedEsState
	inline void setError(int8_t priority, LedES ES_State) { setError(priority, (int)ES_State); };

	///@brief 获取优先级最高且有效的状态值
	///该方法会从最高优先级开始遍历所有状态码，当该优先级对应的状态码不为 LedES_Disable 时该优先级对应的状态码，若所有错误码都是 LedES_Disable 则返回 LedES_BlinkSlow
	///@sa LedEsState
	int8_t getError();
	///@brief 获取当前优先级的状态值
	inline int8_t getError(u8 priority)
	{
		if (priority >= MAX_PRIORITY_NUM)
			return 0;
		return ErrorCode[priority];
	}
};

/** @} */

#endif
