/**
 * @file led.cpp
 * @author 梁文生
 * @brief 
 * @version 0.1
 * @date 2021-10-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "led.h"

/**
 * @ingroup TDT_DEVICE
 * @defgroup LED LED
 * @brief 该LED类除了基本的拉高拉低以外，对led基本的用途——状态显示进行了封装，并兼容控制阴极和控制阳极的两种LED不同接法
 * @details 通过调用setError可设置模块的优先级
 * 通过调用stateShow，将当前各个模块按优先级排序后的状态值显示出来
 */

/**
 * @class Led
 * @details 通过调用 stateShow ，将当前各个模块按优先级排序后的状态值显示出来，要理解源码不难，主要在于对于 Led::ErrorCode 数组的理解  
 * 构造 Led 时，会自动申请int8_t[MAX_PRIORITY_NUM](MAX_PRIORITY_NUM = 10)给ErrorCode 
<table>
<caption id="ledErrorCodeTable_Default">Led::ErrorCode 默认值</caption>
<tr><th>Index  <th>ErrorCode[Index] <th>真实值
<tr><td>0 <td>LedES_Disable <td>0
<tr><td>2 <td>LedES_Disable <td>0
<tr><td>3 <td>LedES_Disable <td>0
<tr><td>4 <td>LedES_Disable <td>0
<tr><td>5 <td>LedES_Disable <td>0
<tr><td>6 <td>LedES_Disable <td>0
<tr><td>7 <td>LedES_Disable <td>0
<tr><td>8 <td>LedES_Disable <td>0
<tr><td>9 <td>LedES_Disable <td>0
</table>
 * 此时 getError() 会返回 LedES_BlinkSlow （因为所有ErrorCode都是LedES_Disable）  
 * 在调用 setError(3,LedES_BlinkFast), setError(-2,2) 之后
<table>
<caption id="ledErrorCodeTable_1">Led::ErrorCode after setError(3,LedES_BlinkFast), setError(-2,2)</caption>
<tr><th>Index  <th>ErrorCode[Index]  <th>真实值
<tr><td>0 <td>LedES_Disable 	<td>0
<tr><td>1 <td>LedES_Disable 	<td>0
<tr><td>2 <td>LedES_Disable 	<td>0
<tr><td>3 <td>LedES_BlinkFast  	<td>-2
<tr><td>4 <td>LedES_Disable 	<td>0
<tr><td>5 <td>LedES_Disable 	<td>0
<tr><td>6 <td>LedES_Disable 	<td>0
<tr><td>7 <td>LedES_Disable 	<td>0
<tr><td>8 <td>2 				<td>2
<tr><td>9 <td>LedES_Disable 	<td>0
</table>
 * 此时 getError() 会返回 LedES_BlinkFast （因为最前面的优先级最高）
 * @note 如果需要修改所有都是 LedES_Disable 时的反应，只需对最低优先级（9）设置一个非LedES_Disable的Error
<table>
<caption id="ledErrorCodeTable_2">Led::ErrorCode 修改后</caption>
<tr><th>Index  <th>ErrorCode[Index] <th>真实值
<tr><td>0 <td>LedES_Disable <td>0
<tr><td>2 <td>LedES_Disable <td>0
<tr><td>3 <td>LedES_Disable <td>0
<tr><td>4 <td>LedES_Disable <td>0
<tr><td>5 <td>LedES_Disable <td>0
<tr><td>6 <td>LedES_Disable <td>0
<tr><td>7 <td>LedES_Disable <td>0
<tr><td>8 <td>LedES_Disable <td>0
<tr><td>9 <td>newError <td>newError
</table>
 * 这时 getError() 会返回 newError （因为最前面的高优先级都是LedES_Disable，不起作用）
 */

/**
  * @brief  LED类构造器
  */
Led::Led(uint32_t RCC_AHB1Periph_GPIOx, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin_x) : RCC_AHB1Periph_GPIOx(RCC_AHB1Periph_GPIOx), GPIOx(GPIOx), GPIO_Pin_x(GPIO_Pin_x), stateOnSpan(50), stateOffSpan(200), slowBlinkInterval(500), fastBlinkInterval(100), stateResetTime(1000), MAX_PRIORITY_NUM(10), ErrorCode(new int8_t[MAX_PRIORITY_NUM])
{
	for (int i = 0; i < MAX_PRIORITY_NUM; i++)
		ErrorCode[i] = LedES_Disable;
}

/**
  * @brief  LED初始化
  */
void Led::init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOx, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
	GPIOx->BSRRL = GPIO_Pin_x;
	show(0);
}

/**
  * @note	可根据状态值闪烁不同的次数
  */
void Led::stateShow(u16 intervalMs)
{
	int8_t state = getError();
	blinkTimer += intervalMs;
	if (state == LedES_BlinkSlow)
	{
		if (blinkTimer < slowBlinkInterval)
			return;
		blinkTimer = 0;
		stateCounter = 0;
		toggle();
		return;
	}
	if (state == LedES_BlinkFast)
	{
		if (blinkTimer < fastBlinkInterval)
			return;
		blinkTimer = 0;
		stateCounter = 0;
		toggle();
		return;
	}
	if (state == LedES_ConstantLight)
	{
		show(1);
		stateCounter = 0;
		return;
	}
	if (state == LedES_ConstantDark)
	{
		show(0);
		stateCounter = 0;
		return;
	}

	if (state == 0)
	{
		state = -1;
	}

	if (state != stateLast) //判断是否值改变
	{
		blinkTimer = 0;
		stateLast = state;
		show(0);
		return;
	}

	if (blinkTimer > stateResetTime + stateOffSpan && stateCounter >= state)
	{
		stateCounter = 0;
	}

	if (showState == 0 && blinkTimer > stateOffSpan && stateCounter < state)
	{
		show(1);
		blinkTimer = 0;
		return;
	}

	if (showState == 1 && blinkTimer > stateOnSpan && stateCounter < state)
	{
		show(0);
		blinkTimer = 0;
		stateCounter++;
		return;
	}
}

void Led::show(u8 state)
{
	showState = state;
	if (state ^ LHNagation)
	{
		GPIOx->BSRRL = GPIO_Pin_x;
		return;
	}
	GPIOx->BSRRH = GPIO_Pin_x;
}

void Led::setError(int8_t priority, int8_t ES_State)
{
	if (priority < 0)
		priority = MAX_PRIORITY_NUM + priority;
	if (priority < 0)
		return;
	if (priority >= MAX_PRIORITY_NUM)
		return;
	ErrorCode[priority] = ES_State;
}

int8_t Led::getError()
{
	//LED只展示位置最先的异常（表现为位于数组的第一个有效数据）
	for (u8 i = 0; i < MAX_PRIORITY_NUM; i++)
	{
		if (ErrorCode[i] != LedES_Disable)
		{
			return ErrorCode[i];
		}
	}
	return LedES_BlinkSlow;
}

