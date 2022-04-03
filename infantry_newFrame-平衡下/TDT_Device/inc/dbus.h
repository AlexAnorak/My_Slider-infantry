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

#pragma pack(1)
//WARN 判断长度是否为8
struct ChassisRecv
{
	uint8_t deforceFlag : 1;
	uint8_t rcSwitch1 : 2;
	uint8_t rcSwitch2 : 2;
	uint8_t keyCtrl : 1;
	uint8_t keyShift : 1;
	uint8_t resetFlag : 1;
	uint8_t selfCheck : 1;
	uint8_t chassisMode : 3;
	int64_t speedWS : 13;
	int64_t speedAD : 13;
	int64_t speedYaw : 11;
	int64_t zeroYaw : 13;
	uint8_t tryUsingBackup : 1;
};
#pragma pack()

class _RC
{
public:
	void init();
	///1000Hz调用函数，进行数据处理、离线检测、按键与VisualTask调用
	u8 run_1000Hz();
	///数据解析
	///@image html RC_Sketch_Map.jpg "遥控器摇杆拨杆示意图"
	void handleData(volatile const uint8_t *sbus_buf);
	///获取对应键盘按键的状态
	int Get_Keypress(uint16_t Key);

	///摇杆拨杆特定手势检测与调用
	void rstCount();

	ChassisRecv chassisRecv, lastChassisRecv;

	uint8_t rcDataFromCan[8];

private:
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


/** @} */

#endif /*_Dbus_H*/
