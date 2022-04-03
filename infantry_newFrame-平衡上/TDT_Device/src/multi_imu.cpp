#include "multi_imu.h"
#include "TimeMatch.h"
#include "flash_var.h"
#include "iwdg.h"
#include "led_task.h"
#include "stdlib.h"

/**
 * @defgroup TDT_DEVICE_MULT_IMU 多陀螺仪组合
 * @ingroup TDT_DEVICE
 * @brief 该类主要是由于为防止陀螺仪突然掉线或数据异常而准备的。具有判断陀螺仪是否出现问题的功能并且切换成备用陀螺仪。同时也集成了同时校准多个陀螺仪的功能
 * @details 使用与普通陀螺仪设备基本一致
 * @code {.cpp}
void Imu_Task()
{	
	if (boardImu->forceGetOffset)
	{
		boardImu->getOffset();
	}
	uint64_t readImuTime = boardImu->TDT_IMU_update();
	if (visionSendYaw != NULL && visionSendPitch != NULL)
		imuTimeMatch.top(float(readImuTime) / 1e6f) << (vec2f({*visionSendYaw, *visionSendPitch}));
}

void imuInit()
{
	u8 currentImuInit = 0;
	boardImu = new MultiImu;
	
	boardImu->validImu[currentImuInit] = new Icm20602(SPI1, SPI_BaudRatePrescaler_64);
	float gyroScaleFactor[3][3] = {{-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	boardImu->validImu[currentImuInit]->setGyroScaleFactor(gyroScaleFactor);
	float accScaleFactor[3][3] = {{-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	boardImu->validImu[currentImuInit]->setAccScaleFactor(accScaleFactor);
	IFlash.link(boardImu->validImu[currentImuInit]->gyro.offset, 2);
	IFlash.link(boardImu->validImu[currentImuInit]->acc.offset, 3);
	currentImuInit++;
	
	{
		boardImu->validImu[currentImuInit] = new Mpu6050(GPIOC, GPIO_Pin_2, GPIO_Pin_1);
		float gyroScaleFactor[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
		boardImu->validImu[currentImuInit]->setGyroScaleFactor(gyroScaleFactor);
		float accScaleFactor[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
		boardImu->validImu[currentImuInit]->setAccScaleFactor(accScaleFactor);
		IFlash.link(boardImu->validImu[currentImuInit]->gyro.offset, 4);
		IFlash.link(boardImu->validImu[currentImuInit]->acc.offset, 5);
		currentImuInit++;
	}
	
	{
		boardImu->validImu[currentImuInit] = new Scha634_03(SPI3,SPI_BaudRatePrescaler_32,{GPIOA, GPIO_Pin_15},CsPin());
		float gyroScaleFactor[3][3] = {{0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
		boardImu->validImu[currentImuInit]->setGyroScaleFactor(gyroScaleFactor);
		float accScaleFactor[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
		boardImu->validImu[currentImuInit]->setAccScaleFactor(accScaleFactor);
		IFlash.link(boardImu->validImu[currentImuInit]->gyro.offset, 6);
		IFlash.link(boardImu->validImu[currentImuInit]->acc.offset, 7);
		currentImuInit++;
	}

	boardImu->init();
	boardImu->getOffset();
	boardImu->initalAngle();

	boardImu->imu_OK = 1;

}
 *@endcode
 *
 */

void MultiImu::init()
{
	imuView = this;
	for (int i = 0; i < VALID_IMU_NUM; i++)
	{
		if (validImu[i] == nullptr)
			break;
		validImu[i]->init();
		safestImu=i;
	}
}
u8 OPEN_PROTECT = 1;

void MultiImu::getOffset()
{
	if (!forceGetOffset && IFlash.read() == 0) //成功读取并且不需要强行矫正
	{
		initalAngle();
		return;
	}

	forceGetOffset = 1;
	boardLed.setError(0, LedES_BlinkFast);
	laser.setError(0, LedES_BlinkFast);

	for (int i = 0; i < VALID_IMU_NUM; i++)
	{
		if (validImu[i] == nullptr)
			break;

		//读取失败或需要强行校准时，只校准陀螺仪，不校准加速度，并存入flash
		while (!validImu[i]->gyroAutoCalibration())
		{
			if (!forceGetOffset)
			{
				boardLed.setError(0, LedES_BlinkSlow);
				laser.setError(0, LedES_BlinkSlow);
				return;
			}
			validImu[i]->gyroAccUpdate();
			cycle.getCycleT();
			iwdgFeed();
			boardLed.stateShow(2);
			laser.stateShow(2);
			delayMs(2);
			boardLed.stateShow(2);
			laser.stateShow(2);
		}
	}

	boardLed.setError(0, LedES_ConstantLight);
	laser.setError(0, LedES_ConstantLight);
	boardLed.show(1);
	laser.show(1);

	IFlash.save();
	__set_FAULTMASK(1);							  //关闭所有中断
	NVIC_SystemReset();							  //复位
	while (1)
	{
	} //仅等待复位
}

void MultiImu::gyroAccUpdate()
{
	lastCurrentImuUsed = currentImuUsed;
	currentImuShouldUsed = 0;	
	
	for (int i = 0; i < VALID_IMU_NUM; i++)
	{
		if (validImu[i] == nullptr)
		{
			if (currentImuShouldUsed < VALID_IMU_NUM - 1 && currentImuShouldUsed == i)
				currentImuShouldUsed++;
			break;
		}

		validImu[i]->gyroAccUpdate();
		validImu[i]->caliSolve();
		validImu[i]->ImuCalc::gyroAccUpdate(); //进行单位换算

		memcpy(&lastGyroOriginData[i], &thisGyroOriginData[i], sizeof(vec3int16));
		memcpy(&lastAccOriginData[i], &thisAccOriginData[i], sizeof(vec3int16));

		memcpy(&thisGyroOriginData[i], &validImu[i]->gyro.origin, sizeof(vec3int16));
		memcpy(&thisAccOriginData[i], &validImu[i]->acc.origin, sizeof(vec3int16));
	}


	for (int i = 0; i < VALID_IMU_NUM; i++)
	{
		if (validImu[i] == nullptr)
		{
			if (currentImuShouldUsed < VALID_IMU_NUM - 1 && currentImuShouldUsed == i)
				currentImuShouldUsed++;
			break;
		}


		u8 inValidDataCnt = 0;
		for (int j = 0; j < 3; j++)
		{
			if (ABS(validImu[i]->gyro.dps.data[j] - validImu[safestImu]->gyro.dps.data[j]) > gyroSwitchThresold)
			{
				currentImuShouldUsed=safestImu;
			}

			if (ABS(validImu[i]->acc.accValue.data[j] - validImu[safestImu]->acc.accValue.data[j]) > accSwitchThresold)
			{
				currentImuShouldUsed=safestImu;
			}
		}
	}

	if (OPEN_PROTECT)
	{
		currentImuUsed = currentImuShouldUsed;
	}

	if (forceAppoint)
	{
		currentImuUsed = forceImuUsed;
	}

	memcpy(&gyro, &validImu[currentImuUsed]->gyro, sizeof(gyrodata));
	memcpy(&acc, &validImu[currentImuUsed]->acc, sizeof(accdata));
	accValueFector = validImu[currentImuUsed]->accValueFector;
	gyroDpsFector = validImu[currentImuUsed]->gyroDpsFector;
	memcpy(&accScaleFactor, &validImu[currentImuUsed]->accScaleFactor, sizeof(mat3f));
	memcpy(&gyroScaleFactor, &validImu[currentImuUsed]->gyroScaleFactor, sizeof(mat3f));
}