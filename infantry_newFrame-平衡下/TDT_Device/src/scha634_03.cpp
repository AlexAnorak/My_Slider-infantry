#include "scha634_03.h"

//#define SOFT_SPI

#define DUE_R_RATE_Z 0x01
#define DUE_R_RATE_Y 0x03
#define DUE_R_TEMP 0x07
#define DUE_R_RATE_Z2 0x0B
#define DUE_R_RATE_Y2 0x0D
#define DUE_R_S_SUM 0x0E
#define DUE_RW_S_CTRL 0x0F
#define DUE_R_R_S1 0x10
#define DUE_R_R_S2 0x11
#define DUE_R_A_S1 0x12
#define DUE_R_C_S1 0x14
#define DUE_R_C_S2 0x15
#define DUE_RW_G_FILT_DYN 0x16
#define DUE_RW_SYS_TEST 0x17
#define DUE_RW_RES_CTRL 0x18
#define DUE_RW_MODE 0x19
#define DUE_R_C_ID 0x1B
#define DUE_R_T_ID2 0x1C
#define DUE_R_T_ID0 0x1D
#define DUE_R_T_ID1 0x1E
#define DUE_RW_SEL_BNK 0x1F

#define UNO_R_RATE_X 0x01
#define UNO_R_ACC_X 0x04
#define UNO_R_ACC_Y 0x05
#define UNO_R_ACC_Z 0x06
#define UNO_R_TEMP 0x07
#define UNO_R_RATE_X2 0x0B
#define UNO_R_S_SUM 0x0E
#define UNO_RW_S_CTRL 0x0F
#define UNO_R_R_S1 0x10
#define UNO_R_R_S2 0x11
#define UNO_R_A_S1 0x12
#define UNO_R_C_S1 0x14
#define UNO_R_C_S5 0x15
#define UNO_RW_G_FILT_DYN 0x16
#define UNO_RW_SYS_TEST 0x17
#define UNO_RW_RES_CTRL 0x18
#define UNO_RW_MODE 0x19
#define UNO_RW_A_FILT_DYN 0x1A
#define UNO_R_C_ID 0x1B
#define UNO_R_T_ID2 0x1C
#define UNO_R_T_ID0 0x1D
#define UNO_R_T_ID1 0x1E
#define UNO_RW_SEL_BNK 0x1F

#define SCHA_WRITE_REG(REG) ((REG<<3) | 0x80)
#define SCHA_READ_REG(REG) ((REG<<3) & 0x7f)

static uint8_t SCHA_CRC8(uint8_t BitValue, uint8_t _CRC)
{
 uint8_t Temp;
 Temp = (uint8_t)(_CRC & 0x80);
 if (BitValue == 0x01)
 {
 Temp ^= 0x80;
 }
 _CRC <<= 1;
 if (Temp > 0)
 {
 _CRC ^= 0x1D;
 }
 return _CRC;
}

// Calculate CRC for 24 MSB's of the 32 bit dword
// (8 LSB's are the CRC field and are not included in CRC calculation)
uint8_t SCHA_CalculateCRC(uint32_t Data)
{
 uint8_t BitIndex;
 uint8_t BitValue;
 uint8_t _CRC;
 _CRC = 0xFF;
 for (BitIndex = 31; BitIndex > 7; BitIndex--)
 {
 BitValue = (uint8_t)((Data >> BitIndex) & 0x01);
 _CRC = SCHA_CRC8(BitValue, _CRC);
 }
 _CRC = (uint8_t)~_CRC;
 return _CRC;
}

#define SCHA_SCL_CLR()	GPIO_ResetBits(GPIOA,GPIO_Pin_5)
#define SCHA_SCL_SET()	GPIO_SetBits(GPIOA,GPIO_Pin_5)
		
#define SCHA_SDO_CLR()	GPIO_ResetBits(GPIOA,GPIO_Pin_7)
#define SCHA_SDO_SET()	GPIO_SetBits(GPIOA,GPIO_Pin_7)
		
#define SCHA_SDI_RED()	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)

Scha634_03::Scha634_03(SPI_TypeDef* spix, int baud, CsPin pinDUE, CsPin pinUNO)
{
	this->gyroDpsFector = 1/80.0f;
	this->accValueFector = 1/4905.0f*9.81;
	imuView = this;
	spiDUE = new Spi(spix, baud);
	spiDUE->csPin = pinDUE;
	spiUNO = new Spi(spix, baud);
	spiUNO->csPin = pinUNO;
}

void Scha634_03::init()
{	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOA时钟
	#ifdef SOFT_SPI
	GPIO_InitTypeDef gpioInitStructure;
	gpioInitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7;		
	gpioInitStructure.GPIO_Mode = GPIO_Mode_OUT;
	gpioInitStructure.GPIO_OType = GPIO_OType_PP;
	gpioInitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	gpioInitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &gpioInitStructure);
	gpioInitStructure.GPIO_Pin =  GPIO_Pin_6;
	gpioInitStructure.GPIO_Mode = GPIO_Mode_IN;
	gpioInitStructure.GPIO_OType = GPIO_OType_OD;
	gpioInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpioInitStructure);
	#else
	spiDUE->init();
	spiUNO->init();
	#endif

//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_SPI3); //PA5复用为 SPI1  CLK//PB3

	spiDUE->csInit(spiDUE->csPin);
	spiUNO->csInit(spiUNO->csPin);
	commInit();
}

#ifdef SOFT_SPI
uint32_t Scha634_03::spi_send_byte32(uint32_t data)
{
	uint8_t i = 0;  
    uint32_t temp=0x00000000;
 
    for(i=32;i>0;i--){  
		if(i%4 == 0)
		{
			delayUsTask(1000);
		}
        if(data&0x80000000){ //if((data&0x80000000) == 0x80000000){
            SCHA_SDO_SET();//MOSI_H;//写1
        }else{
            SCHA_SDO_CLR();//MOSI_L;//写0
        }
        data<<=1;//高位在前
		delayUsTask(5);
        SCHA_SCL_SET();//SCLK_H;//sck高
		delayUsTask(5);
        if(SCHA_SDI_RED()==1){
            temp+=0x1<<(i-1);//读到1
        }
        SCHA_SCL_CLR();//SCLK_L;//sck低
    }  
    return temp;  
}
#endif


void Scha634_03::sendRecvMsg()
{
#ifdef SOFT_SPI
	recvFrame.u32data = spi_send_byte32(sendFrame.u32data); //通过外设SPIx发送一个byte  数据
#else
	for(int i = 0; i < 4; i++)
	{
		recvFrame.u8data[3-i] = spiDUE->readWriteByte(sendFrame.u8data[3-i]);//两个共用1个SPI，可以使用同一个SPI对象
	}
#endif
	if(SCHA_CalculateCRC(recvFrame.u32data) != recvFrame.infoData.Checksum)
	{
		crc8WrongNum++;
	}

}

void Scha634_03::DUE_changeData()
{
	spiDUE->csOn();
	spiUNO->csOff();
	delayUs(1);
	sendRecvMsg();
	spiDUE->csOff();
	spiUNO->csOn();
	delayUs(1);
}

void Scha634_03::UNO_changeData()
{
	delayUs(1);
	spiUNO->csOn();
	spiDUE->csOff();
	delayUs(1);
	sendRecvMsg();
	delayUs(1);
	spiUNO->csOff();
	spiDUE->csOn();
	delayUs(1);
}
	
	
void Scha634_03::commInit()
{	
	if(startup_attempt == 0)
	{
		delayMs(25);
		//mode on
		sendFrame.u32data = 0xE4000067;
		DUE_changeData();
		DUE_changeData();
		UNO_changeData();
		delayMs(100);

		//selete UNO filter
		sendFrame.u32data = 0xD812129E;
		sendFrame.infoData.Checksum = SCHA_CalculateCRC(sendFrame.u32data);
		UNO_changeData();
		sendFrame.u32data = 0xE8022248;
		sendFrame.infoData.Checksum = SCHA_CalculateCRC(sendFrame.u32data);
		UNO_changeData();

		//reset due
		sendFrame.u32data = 0xE000017C;
		DUE_changeData();
		delayMs(25);
		
		//due mode on
		sendFrame.u32data = 0xE4000067;	
		DUE_changeData();
		DUE_changeData();
		delayMs(5);
		
		//select due filter
		sendFrame.u32data = 0xD812129E;	
		sendFrame.infoData.Checksum = SCHA_CalculateCRC(sendFrame.u32data);
		DUE_changeData();
	}
	else
	{
		//mode on
		sendFrame.u32data = 0xE4000067;	
		DUE_changeData();
		DUE_changeData();
		UNO_changeData();
		delayMs(100);

		//selete gyro filter
		sendFrame.u32data = 0xD812129E;	
		sendFrame.infoData.Checksum = SCHA_CalculateCRC(sendFrame.u32data);
		DUE_changeData();
		UNO_changeData();
		
		//select acc filter
		sendFrame.u32data = 0xE8022248;	
		sendFrame.infoData.Checksum = SCHA_CalculateCRC(sendFrame.u32data);
		UNO_changeData();
	}

	
	delayMs(600);

	//set EOI bit
	sendFrame.u32data = 0xE000025B;	
	UNO_changeData();

	sendFrame.u32data = 0x380000D5;
	UNO_changeData();
	delayMs(10);
	UNO_changeData();
	delayMs(10);
	UNO_changeData();
	if((recvFrame.infoData.data>>15) != 0x01)
	{
		if(startup_attempt != 0)
		{
			startup_attempt++;
			return;
			while(1)
			{
			}
		}
		startup_attempt = 1;
		
		//reset scha634
		sendFrame.u32data = 0xE000017C;
		UNO_changeData();
		DUE_changeData();
		
		delayMs(25);

		commInit();
		return;
	}
	
	sendFrame.u32data = 0xE000025B;
	DUE_changeData();
	delayMs(10);
	
	sendFrame.u32data = 0x380000D5;
	DUE_changeData();
	delayMs(10);
	DUE_changeData();
	delayMs(10);
	DUE_changeData();
	if((recvFrame.infoData.data>>15) != 0x01)
	{
		if(startup_attempt != 0)
		{
			startup_attempt++;
			return;
			while(1)
			{
			}
		}
		startup_attempt = 1;
		//reset scha634
		sendFrame.u32data = 0xE000017C;
		UNO_changeData();
		DUE_changeData();
		delayMs(10);

		commInit();
		return;
	}
}

void Scha634_03::gyroAccUpdate()
{
	sendFrame.u32data = 0x040000F7;
	UNO_changeData();
	
	sendFrame.u32data = 0x100000E9;
	UNO_changeData();
	gyro.origin.data[0] = (int16_t)recvFrame.infoData.data;//0x040000F7的数据
	
	sendFrame.u32data = 0x140000EF;
	UNO_changeData();
	acc.origin.data[0] = -(int16_t)recvFrame.infoData.data;//0x100000E9的数据
	
	sendFrame.u32data = 0x180000E5;
	UNO_changeData();
	acc.origin.data[1] = -(int16_t)recvFrame.infoData.data;//0x140000EF的数据
	sendFrame.u32data = 0;
	UNO_changeData();
	acc.origin.data[2] = (int16_t)recvFrame.infoData.data;//0x180000E5的数据
	
	sendFrame.u32data = 0x0C0000FB;
	DUE_changeData();
	
	sendFrame.u32data = 0x040000F7;
	DUE_changeData();
	gyro.origin.data[1] = (int16_t)recvFrame.infoData.data;//0x0C0000FB的数据
	sendFrame.u32data = 0;
	DUE_changeData();
	gyro.origin.data[2] = (int16_t)recvFrame.infoData.data;//0x040000F7的数据
	
}
