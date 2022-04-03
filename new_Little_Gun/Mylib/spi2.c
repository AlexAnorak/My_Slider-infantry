#include "main.h"
void SPI2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );
    RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
//	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
    SPI_Init(SPI2, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���

    SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����

//	SPI1_ReadWriteByte(0xff);//��������
}

//uint16_t SPI_WriteByte(uint16_t TxData)
//{
//GPIO_ResetBits(GPIOB, GPIO_Pin_12);
//SPI_I2S_SendData(SPI2, TxData);
//while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
//GPIO_SetBits(GPIOB, GPIO_Pin_12);
//return SPI_I2S_ReceiveData(SPI2);
//}


uint16_t SPI2_ReadWrite16Bit(uint16_t Txdata)
{
    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET) {}
    SPI_I2S_SendData(SPI2,Txdata);
    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE) == RESET) {}
    return SPI_I2S_ReceiveData(SPI2);
}


