#ifndef _ICM20608_H
#define _ICM20608_H

#include "spi.h"
#include "board.h"
#include "imu.h"

/*
 @class	Icm20608
 @base	Spi
 @note	ICM-20608 SPI Operational Features
	1. Data is delivered MSB first and LSB last
	2. Data is latched on the rising edge of SPC
	3. Data should be transitioned on the falling edge of SPC
	4. The maximum frequency of SPC is 10MHz
	5. SPI read and write operations are completed in 16 or more clock cycles (two or more bytes). The first byte contains the
	SPI Address, and the following byte(s) contain(s) the SPI data. The first bit of the first byte contains the Read/Write bit
	and indicates the Read (1) or Write (0) operation. The following 7 bits contain the Register Address. In cases of multiple 
	byte Read/Writes, data is two or more bytes:
*/
/*¼Ä´æÆ÷µØÖ·ºê¶¨Òå*/
	/**
	 * @brief Definition for connected to SPI1 (APB2 PCLK = 84MHz)
	 */
class Icm20608 : public Spi, public ImuCalc
{
private:

public:
	Icm20608(SPI_TypeDef* spix,int baud);
	short accRaw[3];
	short gyroRaw[3];
	short tempRaw;
	virtual void init(void)override;
	void get6AxisRawData(void);
	void get3AxisAccRawData(void);
	void get3AxisGyroRawData(void);
	void getTempRawData(void);
	void gyroAccUpdate()override;
	ImuCalc *imuView;
};




#endif //#ifndef _ICM20608_H