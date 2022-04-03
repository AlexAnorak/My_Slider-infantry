#include "icm20608.h"
#include "icm20608_reg.h"

/**
  * @brief �����ǹ���������Ҫspi��ʼ��
  */
Icm20608::Icm20608(SPI_TypeDef* spix,int baud) :Spi(spix, baud)
{
	imuView = this;
	accValueFector = 1/2048.0f*9.81;
	gyroDpsFector = 1/16.4f;
}


/**
  * @brief �����ǳ�ʼ������
  */
void Icm20608::init(void)
{
	/*SPi��ʼ��*/
	Spi::init();
	csInit({GPIOA, GPIO_Pin_4});
	uint8_t who_am_i = 0;
	uint8_t byte;
	// Reset ICM20602

	// check WHO_AM_I (0x75)
	while(who_am_i!=0xAF)
	{
		//todo setInitialFail
		delayMs(5);
		who_am_i = readByte(0x75); 
	}
	// PWR_MGMT_1 0x6B
	writeByte(PWR_MGMT_1, 0x80); //Reset ICM20602
	delayMs(50);
	
	byte = readByte(USER_CTRL); //Reset ICM20602
	delayMs(10);
	
	byte = readByte(USER_CTRL); //Reset ICM20602
	delayMs(10);
	byte|=1<<4;
	
	writeByte(USER_CTRL, byte); //Reset ICM20602
	
	delayMs(50);

	// PWR_MGMT_1 0x6B
	writeByte(PWR_MGMT_1, 0x01); // Enable Temperature sensor(bit4-0), Use PLL(bit2:0-01)
									
	delayMs(50);

	// PWR_MGMT_2 0x6C
	writeByte(PWR_MGMT_2, 0x00); // enable Acc(bit5:3-111), Enable Gyro(bit2:0-000)
	//ICM20602_Writebyte( PWR_MGMT_2, 0x00 ); // Enable Acc(bit5:3-000), Enable Gyro(bit2:0-000)
	delayMs(50);
	
	// set sample rate to 1000Hz and apply a software filter
	writeByte(SMPLRT_DIV, 0x00);
	delayMs(50);
	
	// Gyro DLPF Config
	//ICM20602_Writebyte(CONFIG, 0x00); // Gyro LPF fc 250Hz(bit2:0-000)
	writeByte(CONFIG, 0x03); // Gyro LPF fc 20Hz(bit2:0-100) at 1kHz sample rate
	delayMs(50);

	// GYRO_CONFIG 0x1B
	writeByte(GYRO_CONFIG, 0x18); // Gyro sensitivity 2000 dps(bit4:3-11), FCHOICE (bit1:0-00)
	delayMs(50);

	// ACCEL_CONFIG 0x1C
	writeByte(ACCEL_CONFIG, 0x18); // Acc sensitivity 16g
	delayMs(50);
	
	// ACCEL_CONFIG2 0x1D
	writeByte(ACCEL_CONFIG2, 0x03); // Acc FCHOICE 1kHz(bit3-0), DLPF fc 44.8Hz(bit2:0-011)
	delayMs(50);
	
	ImuCalc::init();
	return; //OK
}



/**
  * @brief �����ǻ�ȡ����ԭʼ����
  */
void Icm20608::get6AxisRawData(void)
{
	u8 data[14] = { 0 };
	readBytes(ACCEL_XOUT_H, 14, data);

	accRaw[0] = (short)((data[0] << 8) | data[1]);
	accRaw[1] = (short)((data[2] << 8) | data[3]);
	accRaw[2] = (short)((data[4] << 8) | data[5]);

	tempRaw = (short)((data[6] << 8) | data[7]);

	gyroRaw[0] = (short)((data[8] << 8) | data[9]);
	gyroRaw[1] = (short)((data[10] << 8) | data[11]);
	gyroRaw[2] = (short)((data[12] << 8) | data[13]);	
}



/**
  * @brief �����ǻ�ȡ������������ԭʼ����
  */
void Icm20608::get3AxisGyroRawData(void)
{
	u8 data[6] = { 0 };
	readBytes(GYRO_XOUT_H, 6, data);

	gyroRaw[0] = (short)((data[0] << 8) | data[1]);
	gyroRaw[1] = (short)((data[2] << 8) | data[3]);
	gyroRaw[2] = (short)((data[4] << 8) | data[5]);
}



/**
  * @brief �����ǻ�ȡ������ٶ�ԭʼ����
  */
void Icm20608::get3AxisAccRawData(void)
{
	u8 data[6] = { 0 };
	readBytes(ACCEL_XOUT_H, 6, data);

	accRaw[0] = (short)((data[0] << 8) | data[1]);
	accRaw[1] = (short)((data[2] << 8) | data[3]);
	accRaw[2] = (short)((data[4] << 8) | data[5]);
}



/**
  * @brief �����ǻ�ȡ�¶�ԭʼ����
  */
void Icm20608::getTempRawData(void)
{
	u8 data[2] = { 0 };
	readBytes(TEMP_OUT_H, 2, data);

	tempRaw = (short)((data[0] << 8) | data[1]);
}


void Icm20608::gyroAccUpdate()
{
	get6AxisRawData();
	acc.origin.data[0] = accRaw[0];
	acc.origin.data[1] = accRaw[1];
	acc.origin.data[2] = accRaw[2];
	
	gyro.origin.data[0] = gyroRaw[0];
	gyro.origin.data[1] = gyroRaw[1];
	gyro.origin.data[2] = gyroRaw[2];
}
