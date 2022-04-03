#ifndef __SCHA634_03_H
#define __SCHA634_03_H

#include "board.h"
#include "imu.h"
#include "spi.h"

class Scha634_03:public ImuCalc
{
public:
	Spi *spiDUE;
	Spi *spiUNO;
	union communFrame
	{
		uint8_t u8data[4];
		uint16_t u16data[2];
		uint32_t u32data;
		struct
		{
			uint32_t Checksum:8;//Checksum
			uint32_t data:16;//Return status
			uint32_t returnStatus:2;//Return status
			uint32_t RW_Address:5;//Register address
			uint32_t RW_Operation:1;//R=0 W=1
		}infoData;
	}sendFrame, recvFrame;
	Scha634_03(SPI_TypeDef* spix, int baud, CsPin pinDUE, CsPin pinUNO);
	void init()override;
	virtual void gyroAccUpdate() override;
//	void get6AxisRawData(void);
//	void get3AxisAccRawData(void);
//	void get3AxisGyroRawData(void);
//	void getTempRawData(void);
//	void gyroAccUpdate()override;
	
	uint32_t spi_send_byte32(uint32_t data);
	void sendRecvMsg();
	
	void DUE_changeData();
	void UNO_changeData();
	
	void commInit();
	int startup_attempt = 0;
	int crc8WrongNum = 0;
	ImuCalc *imuView;
};




#endif
