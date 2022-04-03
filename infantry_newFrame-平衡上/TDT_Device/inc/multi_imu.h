#ifndef __MULTI_IMU_H
#define __MULTI_IMU_H

#include "board.h"
#include "imu.h"

/**
 * @addtogroup TDT_DEVICE_MULT_IMU
 * @{
 */

#define VALID_IMU_NUM  2
struct MultiImu : public ImuCalc
{
	ImuCalc *validImu[VALID_IMU_NUM]={nullptr};
	vec3int16 thisGyroOriginData[VALID_IMU_NUM];
	vec3int16 thisAccOriginData[VALID_IMU_NUM];
	vec3int16 lastGyroOriginData[VALID_IMU_NUM];
	vec3int16 lastAccOriginData[VALID_IMU_NUM];

	ImuCalc *imuView;
	u8 currentImuUsed =0;///<当前使用的陀螺仪
	u8 lastCurrentImuUsed = 0;	 ///<上次使用的陀螺仪
	u8 currentImuShouldUsed = 0; ///<根据当前陀螺仪数据判断应该使用哪个陀螺仪
	u8 forceAppoint = 0;///<是否强行指定某个陀螺仪
	u8 forceImuUsed = 0; ///<强制使用哪个陀螺仪，仅在 forceAppoint 为1时可用
	u8 safestImu = 0;///<最稳定的陀螺仪（也是数组中最后一个），当异常时直接使用此陀螺仪
	float gyroSwitchThresold=50;///<偏差大于该值时使用safestImu
	float accSwitchThresold=5;///<偏差大于该值时使用safestImu
	virtual void init() override;
	virtual void getOffset() override;
	virtual void gyroAccUpdate() override;
};

/** @} */

#endif
