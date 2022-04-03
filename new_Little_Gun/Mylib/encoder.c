#include "main.h"

_encoder encoder;

/*
 * ��������NVIC_Configuration
 * ����  ������Ƕ�������жϿ�����NVIC
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* ����P[A|B|C|D|E]0Ϊ�ж�Դ */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//encoder.INPUT_A---PB0
//encoder.INPUT_B---PB1

void Encoder_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    /* config the extiline(PB0) clock and AFIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);

    /* config the NVIC(PB0) */
    NVIC_Configuration();

    /* EXTI line gpio config(PB0) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	 // ��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* EXTI line(PB0) mode config */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //���������ж�
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}


int Encoder_Get_CNT(void)
{
    int cnt = 0;
    return cnt;
}
