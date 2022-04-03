#include "flash_var.h"
#include "stm32f4xx_flash.h"
#include "iwdg.h"

/**
 * @ingroup TDT_DEVICE
 * @defgroup TDT_FLASH_STORAGE FLASH存储管理
 * @brief 该类方便提供了将变量存储到FLASH（最后1个扇区）里
 * @warning 该变量最好是长生命周期（全局变量）的，至少是在save和load时有效
 * @details 下面介绍如何使用该类
 * @section htu_flash 如何使用
 * @subsection htu_flash_bind 绑定变量
 * @code {.cpp}
 * float floatData;
 * float *ptrfloatData = new float[3];
 * ImuCalc *imuStruct = new ImuCalc;
 * ...
 * void linkVar()
 * {
 * 	IFlash.link(floatData,0);//IFalsh知道你真实要存储的变量——浮点型，长度4个字节
 * 	IFlash.link(imuStruct->gyro.offset,1);//每个变量匹配一个ID，来保证变量唯一对应
 * 	IFlash.link(imuStruct->acc.offset,2);
 * 	IFlash.link(ptrfloatData, 3*sizeof(float), 3);//IFalsh不知道你真实要存储的指针对应的变量长度——浮点型指针，此时应手动指定长度
 * }
 * @endcode
 * @sa FlashVar::link(T&,u16)
 * @sa FlashVar::link(u8*,u8,u16)
 * @warning 在保存变量前，必须将所有变量都绑定，否则会丢失未绑定的信息
 * @subsection htu_flash_save 保存变量
 * @code {.cpp}
 * void saveVar()
 * {
 * 	IFlash.save();
 * }
 * @endcode
 * @warning 在保存变量前，必须将所有变量都绑定，否则会丢失未绑定的信息
 * @sa FlashVar::save()
 * @subsection htu_flash_read 读取变量
 * @code {.cpp}
 * void readVar()
 * {
 * 	//after link variable
 * 	IFlash.read();
 * }
 * @endcode
 * @note 读取时可不绑定所有变量
 * @sa FlashVar::read()
 */

#define InnerSECTOR				ADDR_FLASH_SECTOR_11		//参数存储区

//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
 

//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//扇区0起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//扇区1起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//扇区2起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//扇区3起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//扇区4起始地址, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//扇区5起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//扇区6起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//扇区7起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//扇区8起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//扇区9起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//扇区10起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//扇区11起始地址,128 Kbytes  



/**
 * @brief 读取指定地址的半字(16位数据) 
 * @param  faddr            读取的地址
 * @return u32 				对应数据
 */
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32 *)faddr;
}

/**
 * @brief 获取某个地址所在的flash扇区
 * @param  addr             flash地址
 * @return uint16_t 		0~11,即addr所在的扇区
 */
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if (addr < ADDR_FLASH_SECTOR_1)
		return FLASH_Sector_0;
	if (addr < ADDR_FLASH_SECTOR_2)
		return FLASH_Sector_1;
	if (addr < ADDR_FLASH_SECTOR_3)
		return FLASH_Sector_2;
	if (addr < ADDR_FLASH_SECTOR_4)
		return FLASH_Sector_3;
	if (addr < ADDR_FLASH_SECTOR_5)
		return FLASH_Sector_4;
	if (addr < ADDR_FLASH_SECTOR_6)
		return FLASH_Sector_5;
	if (addr < ADDR_FLASH_SECTOR_7)
		return FLASH_Sector_6;
	if (addr < ADDR_FLASH_SECTOR_8)
		return FLASH_Sector_7;
	if (addr < ADDR_FLASH_SECTOR_9)
		return FLASH_Sector_8;
	if (addr < ADDR_FLASH_SECTOR_10)
		return FLASH_Sector_9;
	if (addr < ADDR_FLASH_SECTOR_11)
		return FLASH_Sector_10;
	return FLASH_Sector_11;
}

/**
 * @brief  从指定地址开始写入指定长度的数据
 * @param  WriteAddr        起始地址(此地址必须为4的倍数!!)
 * @param  pBuffer          数据指针
 * @param  NumToWrite       字(32位)数(就是要写入的32位数据的个数.)
 * @note   //特别注意:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
 *         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
 *         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
 *         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
 */
void STMFLASH_Write(u32 WriteAddr, u32 *pBuffer, u32 NumToWrite)
{
	FLASH_Status status = FLASH_COMPLETE;
	u32 addrx = 0;
	u32 endaddr = 0;
	__disable_irq();

	iwdgFeed(); //reload

	if (WriteAddr < STM32_FLASH_BASE || WriteAddr % 4)
		return;					 //非法地址
	FLASH_Unlock();				 //解锁
	FLASH_DataCacheCmd(DISABLE); //FLASH擦除期间,必须禁止数据缓存
	
	addrx = WriteAddr;					  //写入的起始地址
	endaddr = WriteAddr + NumToWrite * 4; //写入的结束地址
	if (addrx < 0X1FFF0000)				  //只有主存储区,才需要执行擦除操作!!
	{
		while (addrx < endaddr) //扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if (STMFLASH_ReadWord(addrx) != 0XFFFFFFFF) //有非0XFFFFFFFF的地方,要擦除这个扇区
			{
				iwdgFeed();
				status = FLASH_EraseSector(STMFLASH_GetFlashSector(addrx), VoltageRange_3); //VCC=2.7~3.6V之间!!
				iwdgFeed();
				if (status != FLASH_COMPLETE)
					break; //发生错误了
			}
			else
				addrx += 4;
		}
	}
	if (status == FLASH_COMPLETE)
	{
		while (WriteAddr < endaddr) //写数据
		{
			if (FLASH_ProgramWord(WriteAddr, *pBuffer) != FLASH_COMPLETE) //写入数据
			{
				break; //写入异常
			}
			WriteAddr += 4;
			pBuffer++;
			iwdgFeed();
		}
	}
	FLASH_DataCacheCmd(ENABLE); //FLASH擦除结束,开启数据缓存
	FLASH_Lock();				//上锁

	__enable_irq();
}

/**
 * @brief 从指定地址开始读出指定长度的数据
 * @param  ReadAddr         起始地址
 * @param  pBuffer          数据指针
 * @param  NumToRead        字(4位)数
 */
void STMFLASH_Read(u32 ReadAddr, u32 *pBuffer, u32 NumToRead)
{
	u32 i;
	for (i = 0; i < NumToRead; i++)
	{
		pBuffer[i] = STMFLASH_ReadWord(ReadAddr); //读取4个字节.
		ReadAddr += 4;							  //偏移4个字节.
	}
}

/*保存至存储介质*/
void SaveToMEMORY(u32 *pBuff, u32 size)
{
	//看门狗复位时间1.5s
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对IWDG->PR IWDG->RLR的写
	auto oldIWDG_P = IWDG->PR;
	auto oldIWDG_RL = IWDG->RLR;
	IWDG_SetPrescaler(IWDG_Prescaler_64); //设置IWDG分频系数
	IWDG_SetReload(750);				  //设置IWDG装载值
	u32 frameTailer=0xA5D25A2D;
	STMFLASH_Write(InnerSECTOR, pBuff, size);
	STMFLASH_Write(InnerSECTOR+4*size, &frameTailer, 1);
	//恢复看门狗时间
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对IWDG->PR IWDG->RLR的写
	IWDG_SetPrescaler(oldIWDG_P);				  //设置IWDG分频系数
	IWDG_SetReload(oldIWDG_RL);					  //设置IWDG装载值
	iwdgFeed();									  //reload
}

/**
 * @brief 从存储介质读出，返回new创建的数组指针
 * @return u32* 			返回创建的数组指针
 * @note  
 */
u32 *ReadFromMEMORY()
{
	u32 size = 0;
	STMFLASH_Read(InnerSECTOR, &size, 1);
	if (size == 0 || size == 0xFFFFFFFF)
		return NULL;
	u32 *pBuff = new u32[size];
	STMFLASH_Read(InnerSECTOR, pBuff, size);
	u32 frameHeader;
	STMFLASH_Read(InnerSECTOR+4*size, &frameHeader, 1);
	if(frameHeader != 0xA5D25A2D)
	{
		delete[] pBuff;
		return NULL;
	}
	return pBuff;
}

FlashVar IFlash(SaveToMEMORY,ReadFromMEMORY);
