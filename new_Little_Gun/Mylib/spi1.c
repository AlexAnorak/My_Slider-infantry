#include "main.h"
void SPI1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_SPI1,  ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

//    GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		//设置SPI的数据大小:SPI发送接收8位帧结构
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为高电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;		//定义波特率预分频的值:波特率预分频值为256
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
//	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
    SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

    SPI_Cmd(SPI1, ENABLE); //使能SPI外设

//	SPI1_ReadWriteByte(0xff);//启动传输
}

//uint16_t SPI_WriteByte(uint16_t TxData)
//{
//GPIO_ResetBits(GPIOB, GPIO_Pin_12);
//SPI_I2S_SendData(SPI2, TxData);
//while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
//GPIO_SetBits(GPIOB, GPIO_Pin_12);
//return SPI_I2S_ReceiveData(SPI2);
//}


uint16_t SPI1_ReadWrite16Bit(uint16_t Txdata)
{
    while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET) {}
    SPI_I2S_SendData(SPI1,Txdata);
    while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET) {}
    return SPI_I2S_ReceiveData(SPI1);
}


