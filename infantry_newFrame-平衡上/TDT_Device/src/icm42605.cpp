#include "icm42605.h"
#define NSSPort GPIOA
#define NSSPin GPIO_Pin_4


/**
  * @brief 陀螺仪构造器、主要spi初始化
  */
Icm42605::Icm42605(SPI_TypeDef* spix,int baud) :Spi(spix, baud)
{


}



/**
  * @brief CS选中
  * @note 暂时只有SPI1，全功能SPUI预计下版本完善
  */
 void Icm42605::csOn()
{
	GPIO_ResetBits(NSSPort, NSSPin);
}
void Icm42605::csOff()
{
	GPIO_SetBits(NSSPort, NSSPin);
}



/**
  * @brief 陀螺仪初始化函数
  */
	u8 who_am_i = 0;
void Icm42605::init(void)
{
	

	// Reset Icm42605

	while (who_am_i != 0x42)
	{
		delayMs(5);
		who_am_i = readByte(ICM42605_WHO_AM_I);
	}
	
	// PWR_MGMT_1 0x6B
	writeByte(ICM42605_DEVICE_CONFIG, 0x01); //Reset ICM20602
	// PWR_MGMT_1 0x6B
	writeByte(ICM42605_PWR_MGMT0, 0x2f); // Enable Temperature sensor(bit4-0), enable acc gyro

	writeByte(ICM42605_GYRO_CONFIG0, 0x63);//250dps,8000Hz

	writeByte(ICM42605_ACCEL_CONFIG0, 0x43); // +-4g,8000Hz

	writeByte(ICM42605_GYRO_CONFIG1, 0x36); // DLPF BW = 170Hz; DLPF Latency = 1ms

	return; //OK
}



/**
  * @brief 陀螺仪获取六轴原始数据
  * @note  注意数据顺序和ICM20602不一样
  */
void Icm42605::get6AxisRawData(void)
{
	u8 data[14] = { 0 };
	readBytes(ICM42605_TEMP_DATA1, 14, data);

	tempRaw = (short)((data[0] << 8) | data[1]);
	
	accRaw[0] = (short)((data[2] << 8) | data[4]);
	accRaw[1] = (short)((data[4] << 8) | data[5]);
	accRaw[2] = (short)((data[6] << 8) | data[7]);

	gyroRaw[0] = (short)((data[8] << 8) | data[9]);
	gyroRaw[1] = (short)((data[10] << 8) | data[11]);
	gyroRaw[2] = (short)((data[12] << 8) | data[13]);
}



/**
  * @brief 陀螺仪获取三轴陀螺仪轴原始数据
  */
void Icm42605::get3AxisGyroRawData(void)
{
	u8 data[6] = { 0 };
	readBytes(ICM42605_GYRO_DATA_X1, 6, data);

	gyroRaw[0] = (short)((data[0] << 8) | data[1]);
	gyroRaw[1] = (short)((data[2] << 8) | data[3]);
	gyroRaw[2] = (short)((data[4] << 8) | data[5]);
}



/**
  * @brief 陀螺仪获取三轴加速度原始数据
  */
void Icm42605::get3AxisAccRawData(void)
{
	u8 data[6] = { 0 };
	readBytes(ICM42605_ACCEL_DATA_X1, 6, data);

	accRaw[0] = (short)((data[0] << 8) | data[1]);
	accRaw[1] = (short)((data[2] << 8) | data[3]);
	accRaw[2] = (short)((data[4] << 8) | data[5]);
}



/**
  * @brief 陀螺仪获取温度原始数据
  */
void Icm42605::getTempRawData(void)
{
	u8 data[2] = { 0 };
	readBytes(ICM42605_TEMP_DATA1, 2, data);

	tempRaw = (short)((data[0] << 8) | data[1]);
}
