/******************************
File name: TDT_Device\src\cpp
Description: 陀螺仪
Class:
	——————————————————————————————————————————————————————————————————————————

	——————————————————————————————————————————————————————————————————————————
Author: 祖传
Version: 1.3.1.191119_alpha
Date: 19.10.15
History: 
	——————————————————————————————————————————————————————————————————————————
	20.11.12 彭阳-封装成类
	——————————————————————————————————————————————————————————————————————————
	19.10.15 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "mpu6050.h"
#include "cycle.h"

using namespace imu;
/**
  * @brief MPU6050构造函数
  * @param[in] iicPort SCL端口
  * @param[in] iicSclPin SCL引脚
  * @param[in] iicSdaPin SDA引脚
  * @param[in] iicSdaPort SDA端口
  * @param[in] IIC 硬件IIC外设口
  */
Mpu6050::Mpu6050(GPIO_TypeDef *iicPort, uint16_t iicSclPin, uint16_t iicSdaPin,GPIO_TypeDef *iicSdaPort,I2C_TypeDef *IIC)
#if _USE_HARDIIC
		: Hardiic(iicPort, iicSclPin, iicSdaPin,IIC,400000,iicSdaPort)
#else
        : Softiic(iicPort, iicSclPin, iicSdaPin,IIC,1000000,iicSdaPort)
#endif
{
	accValueFector = 1 / 4095.875f * 9.81f;
	gyroDpsFector = TO_ANGLE;
	imuView = this;
}

//MPU6050初始化，传入参数：采样率，低通滤波频率
void Mpu6050::init(uint16_t sample_rate, uint16_t lpf)
{
	/*IIC初始化*/
	iicInit();
	//延时100ms
	delayMs(100);

    uint8_t default_filter;

    switch (lpf)
    {
        case 5:
            default_filter = MPU6050_LPF_5HZ;
            break;
        case 10:
            default_filter = MPU6050_LPF_10HZ;
            break;
        case 20:
            default_filter = MPU6050_LPF_20HZ;
            break;
        case 42:
            default_filter = MPU6050_LPF_42HZ;
            break;
        case 98:
            default_filter = MPU6050_LPF_98HZ;
            break;
        case 188:
            default_filter = MPU6050_LPF_188HZ;
            break;
        case 256:
            default_filter = MPU6050_LPF_256HZ;
            break;
        default:
            default_filter = MPU6050_LPF_98HZ;
            break;
    }

    //设备复位
    iicRegWriteByte(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x80);

    delayMs(500);

    //陀螺仪采样率，0x00(1000Hz)   采样率 = 陀螺仪的输出率 / (1 + SMPLRT_DIV)
    iicRegWriteByte(MPU6050_ADDRESS, MPU_RA_SMPLRT_DIV, (1000 / sample_rate - 1));
    //设置设备时钟源，陀螺仪Z轴
    iicRegWriteByte(MPU6050_ADDRESS, MPU_RA_PWR_MGMT_1, 0x03);
    //IIC旁路模式
    iicRegWriteByte(MPU6050_ADDRESS, MPU_RA_INT_PIN_CFG,
                             0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);
    //INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, iicBYPASS_EN, CLOCK_DIS
    //低通滤波频率，0x03(42Hz)
    iicRegWriteByte(MPU6050_ADDRESS, MPU_RA_CONFIG, default_filter);
    //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
    iicRegWriteByte(MPU6050_ADDRESS, MPU_RA_GYRO_CONFIG, 0x18);
    //加速计自检、测量范围(不自检，+-8G)			
    iicRegWriteByte(MPU6050_ADDRESS, MPU_RA_ACCEL_CONFIG, 2 << 3);
	
#if USE_AHRS_LIB
	acc.offset.data[x] = (-3878.29297+4298.54297)/2.0f;
	acc.offset.data[y] = (-4152.43896+3991.14893)/2.0f;
	acc.offset.data[z] = (3496.28198 - 4785.27295)/2.0f;
#endif
	ImuCalc::init();
}

//读取加速度
void Mpu6050::Mpu6050_Read_Acc_Data(void)
{
    u8 dataBuffer[2];

    dataBuffer[0] = iicRegReadByte(MPU6050_ADDRESS, MPU_RA_ACCEL_XOUT_L);
    dataBuffer[1] = iicRegReadByte(MPU6050_ADDRESS, MPU_RA_ACCEL_XOUT_H);
    acc.origin.data[x] = ((((int16_t) dataBuffer[1]) << 8) | dataBuffer[0]);  //加速度X轴

    dataBuffer[0] = iicRegReadByte(MPU6050_ADDRESS, MPU_RA_ACCEL_YOUT_L);
    dataBuffer[1] = iicRegReadByte(MPU6050_ADDRESS, MPU_RA_ACCEL_YOUT_H);
    acc.origin.data[y] = ((((int16_t) dataBuffer[1]) << 8) | dataBuffer[0]);  //加速度Y轴

    dataBuffer[0] = iicRegReadByte(MPU6050_ADDRESS, MPU_RA_ACCEL_ZOUT_L);
    dataBuffer[1] = iicRegReadByte(MPU6050_ADDRESS, MPU_RA_ACCEL_ZOUT_H);
    acc.origin.data[z] = ((((int16_t) dataBuffer[1]) << 8) | dataBuffer[0]);  //加速度Z轴
	
	//减去校准值
    acc.calibration.data[x] = acc.origin.data[x] - acc.offset.data[x];
    acc.calibration.data[y] = acc.origin.data[y] - acc.offset.data[y];
    acc.calibration.data[z] = acc.origin.data[z] - acc.offset.data[z];
	
	acc.accValue.data[x] = acc.calibration.data[x] * 4096.0f * 9.81f;
	acc.accValue.data[y] = acc.calibration.data[y] * 4096.0f * 9.81f;
	acc.accValue.data[z] = acc.calibration.data[z] * 4096.0f * 9.81f;

}




//读取角速度
void Mpu6050::Mpu6050_Read_Gyro_Data(void)
{
    u8 dataBuffer[2];
    dataBuffer[0] = iicRegReadByte(MPU6050_ADDRESS, MPU_RA_GYRO_XOUT_L);
    dataBuffer[1] = iicRegReadByte(MPU6050_ADDRESS, MPU_RA_GYRO_XOUT_H);
    gyro.origin.data[x] = ((((int16_t) dataBuffer[1]) << 8) | dataBuffer[0]);     //陀螺仪X轴

    dataBuffer[0] = iicRegReadByte(MPU6050_ADDRESS, MPU_RA_GYRO_YOUT_L);
    dataBuffer[1] = iicRegReadByte(MPU6050_ADDRESS, MPU_RA_GYRO_YOUT_H);
    gyro.origin.data[y] = ((((int16_t) dataBuffer[1]) << 8) | dataBuffer[0]);     //陀螺仪Y轴

    dataBuffer[0] = iicRegReadByte(MPU6050_ADDRESS, MPU_RA_GYRO_ZOUT_L);
    dataBuffer[1] = iicRegReadByte(MPU6050_ADDRESS, MPU_RA_GYRO_ZOUT_H);
    gyro.origin.data[z] = ((((int16_t) dataBuffer[1]) << 8) | dataBuffer[0]);  //陀螺仪Z轴		
}

//mpu6050读取数据
void Mpu6050::Mpu6050_Read(void)
{
    Mpu6050_Read_Acc_Data();
    Mpu6050_Read_Gyro_Data();
}

