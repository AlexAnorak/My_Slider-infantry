/******************************
File name: TDT_Device\src\dbus.cpp
Description: 遥控器接收机
function:
	——————————————————————————————————————————————————————————————————————————
	void Dbus_Config(void)
	——————————————————————————————————————————————————————————————————————————
	void USART2_IRQHandler(void)
	——————————————————————————————————————————————————————————————————————————
	static void Handle_data(volatile const uint8_t *sbus_buf, struct _RC *rc_ctrl)
	——————————————————————————————————————————————————————————————————————————
	int Get_Keypress(uint16_t Key)
	——————————————————————————————————————————————————————————————————————————
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.18 肖银河-增加切换为原始数据处理的变量
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次记录
	——————————————————————————————————————————————————————————————————————————
****************************  */
#define __DBUS_DRIVER_GLOBALS
#include "dbus.h"
#include "board.h"
#include "string.h"
#include "task_virtual.h"
#include "KeyProcess.h"
#include "state_task.h"
#include "fast_selfCheck.h"

/**
 * @ingroup TDT_DEVICE
 * @defgroup DBUS 遥控器解算
 * @brief 该类提供了遥控器的数据解算，以及常用的重要标志位
 */
_RC RC;
uint8_t deforceFlag = 0;
uint8_t dbusOnlineFlag = 0;

unsigned char sbus_rx_buffer[RC_FRAME_LENGTH]; //原始数据
using namespace RCS;

uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];//DMA的原始数据

void _RC::init()
{
	deforceFlag = 1;
	/*DBUS初始化*/
	Dbus_Config();
}



/**
  * @brief 遥控器初始化
  */
void _RC::Dbus_Config(void)
{
/* -------------- 初始化时钟资源 ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); //PA3  usart3 rx

/* -------------- 配置GPIO ---------------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_DeInit(USART2);

	USART_InitStructure.USART_BaudRate = 100000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2, &USART_InitStructure);

	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

	USART_ClearFlag(USART2, USART_FLAG_IDLE);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

	USART_Cmd(USART2, ENABLE);

/* -------------- 中断配置 ---------------------------------------*/
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

/* -------------- 配置 DMA -----------------------------------------*/
	DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(DMA1_Stream5);

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SBUS_rx_buf[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = SBUS_RX_BUF_NUM;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);

	DMA_DoubleBufferModeConfig(DMA1_Stream5, (uint32_t)SBUS_rx_buf[1], DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);
	DMA_Cmd(DMA1_Stream5, DISABLE); //Add a disable
	DMA_Cmd(DMA1_Stream5, ENABLE);
}


///@sa VirtualTask
void _RC::run_1000Hz()
{
	if(updated)
	{
		handleData(sbus_rx_buffer);
		updated = 0;
		dbusOnlineFlag = 1;
		RC.taskSchedule();
		State_Ctrl_RC_Info();
		return;
	}
	if (timeIntervalFrom(updateTime) > 500000) //500ms
	{
		deforceFlag = 1;
		/*离线检测*/
		if (dbusOnlineFlag == 1)//后面置一代表相当于离线跳变
		{
			//清除旧数据
			memset(&Key, 0, sizeof(Key));
			memset(&LastKey, 0, sizeof(LastKey));
			memset(&KeyPress, 0, sizeof(KeyPress));
			memset(&KeyTick, 0, sizeof(KeyTick));
			SW1Tick = UnDefined;
			SW2Tick = UnDefined;
			//仍然进行按键处理，不过所有键松开
			KeyProcess::keyHandle(0);
			deforceFlag = 1;
			//仍然进行任务调度
			taskSchedule();
		}
		dbusOnlineFlag = 0;
	}
}

void _RC::handleData(volatile const uint8_t *SBUS_buf)
{
	//▲ 记录当前值
	memcpy(&LastKey, &Key, sizeof(Key));

	//右摇杆横向  范围+-660
	Key.CH[0] = my_deathzoom(((SBUS_buf[0] | (SBUS_buf[1] << 8)) & 0x07ff)-1024,5); //Channel 0
	//右摇杆纵向   范围+-660
	Key.CH[1] = my_deathzoom((((SBUS_buf[1] >> 3) | (SBUS_buf[2] << 5)) & 0x07ff) - 1024, 5); //Channel 1
	//左摇杆横向   范围+-660
	Key.CH[2] = my_deathzoom((((SBUS_buf[2] >> 6) | (SBUS_buf[3] << 2) | (SBUS_buf[4] << 10)) & 0x07ff) - 1024, 5); //Channel 2
	//左摇杆纵向   范围+-660
	Key.CH[3] = my_deathzoom((((SBUS_buf[4] >> 1) | (SBUS_buf[5] << 7)) & 0x07ff) - 1024, 5); //Channel 3
	//左边开关  132 上中下
	Key.CH[4] = ((SBUS_buf[5] >> 4) & 0x000C) >> 2; //Switch left
	//右边开关  132 上中下
	Key.CH[5] = ((SBUS_buf[5] >> 4) & 0x0003); //Switch right

	/***鼠标X值***/
	Key.CH[6] = ((SBUS_buf[6]) | (SBUS_buf[7] << 8)); //x
	/***鼠标Y值***/
	Key.CH[7] = -((SBUS_buf[8]) | (SBUS_buf[9] << 8)); //y
	/***鼠标左键***/
	Key.CH[8] = SBUS_buf[12];
	Key.left_jump = Key.CH[8];
	/***鼠标右键***/
	Key.CH[9] = SBUS_buf[13];
	Key.Right_jump = Key.CH[9];

	/***键盘值***/
	Key.CH[10] = SBUS_buf[14] | (SBUS_buf[15] << 8);

	Key.SW1 = (SWPos)Key.CH[4];
	Key.SW2 = (SWPos)Key.CH[5];

	/*▲ 跳变检测*/
	//拨杆跳变检测,刚连接上，进入此函数时dbusOnlineFlag仍然为0
	if(dbusOnlineFlag)
	{
		SW1Tick = (SWTick)(Key.SW1 - LastKey.SW1);
		SW2Tick = (SWTick)(Key.SW2 - LastKey.SW2);
	}
	
	//按键跳变检测-异或操作，相同为零，不同为一，那么跳变的位就为1，再赋值给单独的变量
	KeyTick.CH[10] = Key.CH[10] ^ LastKey.CH[10];
	KeyTick.left_jump = Key.left_jump ^ LastKey.left_jump;
	KeyTick.Right_jump = Key.Right_jump ^ LastKey.Right_jump;

	//按下值检测-跳变值并上上一次值的取反-那么跳变且上一次为0的键为1
	KeyPress.CH[10] = KeyTick.CH[10] & (~LastKey.CH[10]);
	KeyPress.left_jump = KeyTick.left_jump & (~LastKey.left_jump);
	KeyPress.Right_jump = KeyTick.Right_jump & (~LastKey.Right_jump);

	/*▲ 根据偏移值，定位按键变量，并赋值*/
	*((uint16_t *)&(Key.keyValue)) = Key.CH[10];
	*((uint16_t *)&(KeyTick.keyValue)) = KeyTick.CH[10];
	*((uint16_t *)&(KeyPress.keyValue)) = KeyPress.CH[10];
}

/**
  * @param  Key 键值
  * @retval True or False
  */
int _RC::Get_Keypress(uint16_t Key)
{
    if(this->Key.CH[10] & Key)
	{
		return 1;
	}
    else
	{
		return 0;
	}
}

#include "imu_task.h"

void _RC::rstCount(void)
{
	static u8 RST_Delay_Cnt = 0;
	static u8 HasRST = 1;
	if (((int)Key.CH[0]) > 600 && ((int)Key.CH[1]) < -600 && ((int)Key.CH[2]) < -600 && ((int)Key.CH[3]) < -600)
	{
		//▼ 自检
		if (Key.SW1 == Mid)
		{
			fastSelfCheck.selfCheckingFlag = 1;
		}
		//▼ 重启
		if (Key.SW1 == Down && HasRST == 0)
		{
			RST_Delay_Cnt++;
			if (RST_Delay_Cnt > 150) //150*14ms大约为2s
			{
				chassisSend.resetFlag = true;
				void sendChassisSend();
				sendChassisSend();
				__disable_irq(); //关闭所有中断
				NVIC_SystemReset(); //复位
				while (1)
				{
				} //仅等待复位
			}
		}
	}
	//校准陀螺仪
	else if (((int)Key.CH[0]) < -600 && ((int)Key.CH[1]) < -600 && ((int)Key.CH[2]) > 600 && ((int)Key.CH[3]) < -600)
	{
		//▼ 校准陀螺仪
		if (Key.SW1 == Mid)
		{
			boardImu->forceGetOffset = 1;
		}
	}
	else
	{
		RST_Delay_Cnt = 0; //清空计数器
		HasRST = 0;
	}
}

void _RC::remoteUpdate()
{
	//调用其他任务的遥控器更新函数
	for (int i = 0; i < VirtualTask::taskNum; i++)
	{
		VirtualTask::taskList[i]->remoteCtrlUpdate();
	}
	KeyProcess::keyHandle(Key.keyValue);
}


/**
  * @brief 任务调度
  * @note 通过遥控器指令执行相关任务的挂起和恢复
  * @warning 脱力控制也在这边，注意一下
  */
void _RC::taskSchedule()
{
	remoteUpdate();
	//▲ 如果到脱力键，发送通知到紧急停止任务，不带通知值，且不保留接受任务的通知值，接受任务的通知值加一
	if (Key.SW2 == RCS::Down || Key.SW2 == RCS::Lost)
	{
		if (!deforceFlag)
		{ //调用其他任务的脱力回调函数
			for (int i = 0; i < VirtualTask::taskNum; i++)
			{
				VirtualTask::taskList[i]->deforceCallBack();
			}
		}
		deforceFlag = 1;
		//执行重启计数函数，等待特定的重启操作后将重启
		rstCount();
	}

	//脱力状态解除，恢复挂起的任务，执行一些初始化
	else
	{
		if (deforceFlag)
		{
			fastSelfCheck.selfCheckingFlag = 0;
			boardImu->forceGetOffset = 0;
			//调用其他任务的脱力回调函数
			for (int i = 0; i < VirtualTask::taskNum; i++)
			{
				VirtualTask::taskList[i]->deforceCancelCallBack();
			}
		}
		deforceFlag = 0;
		//清除相关隐患变量
		//轮询所有使能电机
	}
}

/**
  * @brief 遥控器串口接收中断
  */
void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		static uint16_t this_time_rx_len = 0;
		USART_ReceiveData(USART2);

		if (DMA_GetCurrentMemoryTarget(DMA1_Stream5) == 0)
		{
			//重新设置DMA
			DMA_Cmd(DMA1_Stream5, DISABLE);
			this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);
			DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);
			DMA1_Stream5->CR |= DMA_SxCR_CT;
			//清DMA中断标志
			DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
			DMA_Cmd(DMA1_Stream5, ENABLE);
			if (this_time_rx_len == RC_FRAME_LENGTH)
			{
				memcpy(sbus_rx_buffer, SBUS_rx_buf[0], RC_FRAME_LENGTH);
			}
		}
		else
		{
			//重新设置DMA
			DMA_Cmd(DMA1_Stream5, DISABLE);
			this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);
			DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);
			DMA1_Stream5->CR &= ~(DMA_SxCR_CT);
			//清DMA中断标志
			DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
			DMA_Cmd(DMA1_Stream5, ENABLE);
			if (this_time_rx_len == RC_FRAME_LENGTH)
			{
				memcpy(sbus_rx_buffer, SBUS_rx_buf[1], RC_FRAME_LENGTH);
			}
		}
		RC.updateTime = getSysTimeUs();
		RC.updated = 1;
	}
}
