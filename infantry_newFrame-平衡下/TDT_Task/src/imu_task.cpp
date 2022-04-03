/******************************
File name: TDT_Task\src\imu_task.cpp
Description: 陀螺仪姿态解算任务
function:
	——————————————————————————————————————————————————————————————————————————
	void Imu_Task(void *pvParameters)
	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "imu_task.h"
/**TDT_Device************************/
#include "mpu6050.h"
#include "icm20602.h"
#include "icm20608.h"
#include "scha634_03.h"
#include "TimeMatch.h"
#include "cycle.h"
#include "curve_model.h"
#include "parameter.h"
#include "flash_var.h"

extern TimeSimultaneity imuTimeMatch;

MultiImu *boardImu;

//j scope观测变量用
eulerAngle angleForWatch,angleForWatch2;
accdata accForWatch, accForWatch1, accForWatch2;
gyrodata gyroForWatch, gyroForWatch1, gyroForWatch2;
int currentImuUsed=0;

float *visionSendYaw, *visionSendPitch;
/**
  * @brief 陀螺仪任务
  * @note 负责数据读取和解算
  */
void Imu_Task()
{
	//强制校准获取补偿
	if (boardImu->forceGetOffset)
	{
		boardImu->getOffset();
	}
	/*陀螺仪读取*/
	uint64_t readImuTime = boardImu->TDT_IMU_update();

	//采用非应答模式
#if !ANSWER_MODE
	void vision_Send_Data();
	vision_Send_Data();
#endif
	angleForWatch = boardImu->Angle;
	angleForWatch2 = boardImu->nowAngle;
	accForWatch1 = boardImu->validImu[0]->acc;
	accForWatch2 = boardImu->validImu[1]->acc;
	accForWatch = boardImu->acc;
	gyroForWatch1 = boardImu->validImu[0]->gyro;
	gyroForWatch2 = boardImu->validImu[1]->gyro;
	gyroForWatch = boardImu->gyro;
	currentImuUsed=boardImu->currentImuUsed;
}

#include "flash_var.h"
void imuInit()
{
	u8 currentImuInit = 0;
	boardImu = new MultiImu;
	
	boardImu->validImu[currentImuInit] = new Icm20602(SPI1, SPI_BaudRatePrescaler_64);
	float gyroScaleFactor[3][3] = {DEFAULT_BOARD_INSTALL_SPIN_MATRIX};
	boardImu->validImu[currentImuInit]->setGyroScaleFactor(gyroScaleFactor);
	float accScaleFactor[3][3] = {DEFAULT_BOARD_INSTALL_SPIN_MATRIX};
	boardImu->validImu[currentImuInit]->setAccScaleFactor(accScaleFactor);
	IFlash.link(boardImu->validImu[currentImuInit]->gyro.offset, 2);
	IFlash.link(boardImu->validImu[currentImuInit]->acc.offset, 3);
	currentImuInit++;
	

	

	boardImu->forceAppoint=0;
//	强制使用第0个陀螺仪(icm20602)，测试用
	boardImu->forceImuUsed = 0;
	/*icm20602以及MPU6050初始化*/
	boardImu->init();
	boardImu->getOffset();
	boardImu->initalAngle();
	
	boardImu->imu_OK = 1;

	//视觉发送的值的初始化
	visionSendYaw = &boardImu->Angle.yaw;
	visionSendPitch = &boardImu->Angle.pitch;
}

