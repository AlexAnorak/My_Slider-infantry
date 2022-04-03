/*****************************************************************************
File name: TDT_Device\src\dbus.h
Description: 遥控器接收机
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.18 肖银河-增加切换为原始数据处理的变量
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef _Dbus_H
#define _Dbus_H

#include "stm32f4xx.h"
#include "TDT_User.h"
#include <stdio.h>
#include "my_math.h"

/**
 * @addtogroup DBUS 
 * @{
 */
#define SBUS_RX_BUF_NUM 36u ///<DMA数据长度
#define RC_FRAME_LENGTH 18u ///<遙控器接收数据包长度

#define KEY_B 0x8000
#define KEY_V 0x4000
#define KEY_C 0x2000
#define KEY_X 0x1000
#define KEY_Z 0x0800
#define KEY_G 0x0400
#define KEY_F 0x0200
#define KEY_R 0x0100
#define KEY_E 0x0080
#define KEY_Q 0x0040
#define KEY_CTRL 0x0020
#define KEY_SHIFT 0x0010
#define KEY_D 0x0008
#define KEY_A 0x0004
#define KEY_S 0x0002
#define KEY_W 0x0001

///RemoteCtrl nameSpace
namespace RCS
{
	///拨杆位置
	enum SWPos
	{
		Lost = 0, ///<遥控器信号丢失
		Up = 1,	  ///<拨杆在上
		Mid = 3,  ///<拨杆在中
		Down = 2  ///<拨杆在下
	};
	///按键跳变枚举，如Up_Mid指上面拨到中间，值为当前值减上一个位置值
	///@warning 当未出现跳变时，会保持上一次跳变状态
	enum SWTick
	{
		UnDefined = 0, ///<遥控器信号丢失或当前不足以判断跳变（比如第一帧数据）
		Up_Mid = 2,	   ///<上跳变到中
		Mid_Up = -2,   ///<中跳变到上
		Mid_Down = -1, ///<中跳变到下
		Down_Mid = 1   ///<下跳变到中
	};
}

///包括摇杆、拨杆、鼠标、键盘在内的各种信息
///@image html RC_Sketch_Map.jpg "遥控器摇杆拨杆示意图"
typedef struct
{
	int16_t CH[11];
	RCS::SWPos SW1;
	RCS::SWPos SW2;
	u8 left_jump;
	u8 Right_jump;
	union
	{
		uint16_t keyValue;
		///必须按顺序
		struct
		{
			uint16_t W : 1;		///<0x0001
			uint16_t S : 1;		///<0x0002
			uint16_t A : 1;		///<0x0004
			uint16_t D : 1;		///<0x0008
			uint16_t SHIFT : 1; ///<0x0010
			uint16_t CTRL : 1;	///<0x0020
			uint16_t Q : 1;		///<0x0040
			uint16_t E : 1;		///<0x0080
			uint16_t R : 1;		///<0x0100
			uint16_t F : 1;		///<0x0200
			uint16_t G : 1;		///<0x0400
			uint16_t Z : 1;		///<0x0800
			uint16_t X : 1;		///<0x1000
			uint16_t C : 1;		///<0x2000
			uint16_t V : 1;		///<0x4000
			uint16_t B : 1;		///<0x8000
		};
	};
} _Key;

class _RC
{
public:
	void init();
	///1000Hz调用函数，进行数据处理、离线检测、按键与VisualTask调用
	void run_1000Hz();
	///数据解析
	///@image html RC_Sketch_Map.jpg "遥控器摇杆拨杆示意图"
	void handleData(volatile const uint8_t *sbus_buf);
	///获取对应键盘按键的状态
	int Get_Keypress(uint16_t Key);

	///摇杆拨杆特定手势检测与调用
	void rstCount();

	_Key Key;			 //当前值
	_Key LastKey;		 //上次值
	_Key KeyTick;		 //跳变值
	_Key KeyPress;		 //按下值
	RCS::SWTick SW1Tick; //左上角拨杆跳变值
	RCS::SWTick SW2Tick; //右上角拨杆跳变值
private:
	void Dbus_Config(void);
	void remoteUpdate();
	void taskSchedule();
public:
	u8 updated;
	uint64_t updateTime;
};

///遥控器实例句柄
extern _RC RC;
///脱力标志位
extern uint8_t deforceFlag;
///遥控器在线标志位
extern uint8_t dbusOnlineFlag;

///@warning C++必须调用将中断的函数放置于extern"C"中，以避免C++编译器对变量名碎片化
#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

	void USART2_IRQHandler(void);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

/** @} */

#endif /*_Dbus_H*/
