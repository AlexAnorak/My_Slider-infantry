#include "main.h"
_keyvalue kv;
void KEY_Configuration(void)
{
    GPIO_InitTypeDef gpio;
//    EXTI_InitTypeDef exti;
//    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_5|GPIO_Pin_14|GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &gpio);

//    exti.EXTI_Line = EXTI_Line11;
//    exti.EXTI_Mode = EXTI_Mode_Interrupt;
//    exti.EXTI_Trigger = EXTI_Trigger_Falling;
//    exti.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&exti);
//
//    nvic.NVIC_IRQChannel = EXTI15_10_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 0;
//    nvic.NVIC_IRQChannelSubPriority = 0;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);
//
//    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource11);
}


void  get_key(void)
{
    if(PBin(0)==0)
    {
        delay_ms(4);
        if(PBin(0)==0)
            kv.keyv[0]=SET;
        else
            kv.keyv[0]=RESET;
    } else
        kv.keyv[0]=RESET;

    if(PBin(1)==0)
    {
        delay_ms(4);
        if(PBin(1)==0)
            kv.keyv[1]=SET;
        else
            kv.keyv[1]=RESET;
    } else
        kv.keyv[1]=RESET;

    if(PBin(2)==0)
    {
        delay_ms(4);
        if(PBin(2)==0)
            kv.keyv[2]=SET;
        else
            kv.keyv[2]=RESET;
    } else
        kv.keyv[2]=RESET;

    if(PBin(5)==0)
    {
        delay_ms(4);
        if(PBin(5)==0)
            kv.keyv[3]=SET;
        else
            kv.keyv[3]=RESET;
    } else
        kv.keyv[3]=RESET;

    if(PBin(14)==0)
    {
        delay_ms(4);
        if(PBin(14)==0)
            kv.keyv[4]=SET;
        else
            kv.keyv[4]=RESET;
    } else
        kv.keyv[4]=RESET;

    if(PBin(15)==0)
    {
        delay_ms(4);
        if(PBin(15)==0)
            kv.keyv[5]=SET;
        else
            kv.keyv[5]=RESET;
    } else
        kv.keyv[5]=RESET;


    if(kv.keyv!=kv.keyv_last)
        kv.kflag[0]=1;
    else
        kv.kflag[0]=0;

    kv.keyv_last[0]=kv.keyv[0];
    kv.keyv_last[1]=kv.keyv[1];
    kv.keyv_last[2]=kv.keyv[2];
    kv.keyv_last[3]=kv.keyv[3];
    kv.keyv_last[4]=kv.keyv[4];
    kv.keyv_last[5]=kv.keyv[5];
}
//void EXTI15_10_IRQHandler(void)
//{
//    if(EXTI_GetITStatus(EXTI_Line11)!=RESET)
//    {
//	LED_ON;
//				RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE);
//        EXTI_ClearITPendingBit(EXTI_Line11);
//    }
//}
