#include "main.h"

//PA8----EN
//PA9----PWM---TIM1_CH2
//PA10---EN
//PA11---PWM---TIM1_CH4
//#define THROTTLE_TRAVEL_CALIORATION //油门行程校准
//#define SIMulative_PWM //模拟高低油门，手动低油门，可用于C615改转向
int pass = 0;//模拟油门校准行程用
/*************************************************************************
                              PWM初始化
*************************************************************************/
void PWM_Configuration(int arr)
{
	int i;
    TIM_TimeBaseInitTypeDef  tim;
    TIM_OCInitTypeDef        oc;
    GPIO_InitTypeDef         gpio;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    tim.TIM_Period=arr-1;    //arr             Fpwm=72M/((arr+1)*(psc+1))
    tim.TIM_Prescaler=18-1;   //psc             duty circle= TIMx->CCRx/arr
    tim.TIM_ClockDivision=TIM_CKD_DIV1;
    tim.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &tim);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM1, &oc);
    TIM_OC2Init(TIM1, &oc);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1,ENABLE);
	TIM_Cmd(TIM1, ENABLE);
    

#ifdef THROTTLE_TRAVEL_CALIORATION
    TIM1->CCR1 = 8000; //拉高
    TIM1->CCR2 = 8000;

	for(i = 0;i < 40;i++) //等待
	{
		delay_ms(100);
		LED_TOGGLE;
	}
    TIM1->CCR1 = 4000; //拉低
    TIM1->CCR2 = 4000;
	while(1)
	{
	}
#endif
#ifdef SIMulative_PWM	//先上信号电，待LED稳定闪烁后再上动力电，相当于高油门上电	
	//模拟大油门
    TIM1->CCR1 = 8000;	
    TIM1->CCR2 = 8000;
	while(!pass)		//debug界面中改pass的值,相当于拉低油门
	{
		LED_TOGGLE;
		delay_ms(200);
	}
	//模拟小油门
    TIM1->CCR1 = 4000;	
    TIM1->CCR2 = 4000;
	while(1)
	{
	}
#endif
    TIM1->CCR1 = 4000;
    TIM1->CCR2 = 4000;
	delay_ms(5000);//2000//1500//如果上电之后摩擦轮只响了4下，并且开环无法转，增大这一行的延时秒数，并反馈
	delay_ms(600);//不要更改这行代码
	state.ReadyToStart = 1;
}

/*************************************************************************
                              电压输出
*************************************************************************/
void PWM_OutSet(int value)
{
    int output = limit(value, -3000, 3000);						//给定值限幅

    if(output > 0)																		//根据给定值设置上桥输出和下桥通断
    {
        TIM2->CCR1 = output;
        TIM2->CCR2 = 0;
    }
    else
    {
        TIM2->CCR1 = 0;
        TIM2->CCR2 = -output;
    }
}
/*************************************************************************
															DIR
*************************************************************************/
void DIR_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6|GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    DIR1_H;
    DIR2_H;
    DIR3_H;
}
