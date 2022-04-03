#include "main.h"

//HVD232---CAN_TX---PB9
//HVD232---CAN_RX---PB8
canFeedback_t canFeedback;


void CAN_Configuration(void)
{
    CAN_InitTypeDef can;
    CAN_FilterInitTypeDef can_filter;
    GPIO_InitTypeDef gpio;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_8;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);    //CAN_RX

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);    //CAN_TX

    nvic.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    CAN_DeInit(CAN1);

    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_5tq;
    can.CAN_BS2 = CAN_BS2_3tq;
    can.CAN_Prescaler = 4;     //CAN BaudRate 36/(1+5+3)/4=1Mbps
    CAN_Init(CAN1, &can);

    can_filter.CAN_FilterNumber = 0;
    can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh = 0;
    can_filter.CAN_FilterIdLow = 0;
    can_filter.CAN_FilterMaskIdHigh = 0;
    can_filter.CAN_FilterMaskIdLow = 0;
    can_filter.CAN_FilterFIFOAssignment = 0;
    can_filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&can_filter);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);
}


void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg Can_Rx;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &Can_Rx);
    }
    switch (Can_Rx.StdId)
    {
        case 0x100:
            state.offline_check = 0;
            if (Can_Rx.Data[7] == 1)
            {
                state.ForceOpenLoop = 1;
                state.ForceOpenloop_setspeed_A = (int16_t)((Can_Rx.Data[0] << 8) | (Can_Rx.Data[1]));
                state.ForceOpenloop_setspeed_B = (int16_t)((Can_Rx.Data[2] << 8) | (Can_Rx.Data[3]));
                if (state.ForceOpenloop_setspeed_A == 4000)
                {
                    canFeedback.Set_speed_A = 0;
                    canFeedback.Set_speed_B = 0;
					state.StopState = 1;
                }
            } else
            {
                state.ForceOpenLoop = 0;
                canFeedback.Set_speed_A = (int16_t)((Can_Rx.Data[0] << 8) | (Can_Rx.Data[1]));
                canFeedback.Set_speed_B = (int16_t)((Can_Rx.Data[2] << 8) | (Can_Rx.Data[3]));
            }
			state.INFANTRY_ID = 3;//Can_Rx.Data[4]; //1;
			if(state.INFANTRY_ID!=state.last_INFANTRY_ID)
			{
				state.paraLoadFlag=0;
				reloadparameter(state.INFANTRY_ID);
				state.last_INFANTRY_ID=state.INFANTRY_ID;
			}
            CH4_LED_OFF;
            CH3_LED_OFF;
            break;
        default:
            break;
    }
}

void USB_HP_CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_TME) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
    }
}

void TDT_State_Updata(void)
{
    CanTxMsg Can1_State_Updata;
    Can1_State_Updata.IDE = 0;             //��׼֡
    Can1_State_Updata.RTR = 0;             //����֡
    Can1_State_Updata.DLC = 8;             //֡����

    Can1_State_Updata.StdId = 0x101;      //ID:0x101

    Can1_State_Updata.Data[0] = (u8)((int16_t)(state.ShootNum) >> 8);
    Can1_State_Updata.Data[1] = (u8)((int16_t)(state.ShootNum));

    Can1_State_Updata.Data[2] = (u8)((int16_t)(state.SN_now_spd_A) >> 8);
    Can1_State_Updata.Data[3] = (u8)((int16_t)(state.SN_now_spd_A));

    Can1_State_Updata.Data[4] = (u8)((int16_t)(state.SN_now_spd_B) >> 8);
    Can1_State_Updata.Data[5] = (u8)((int16_t)(state.SN_now_spd_B));

    if (state.SN_now_spd_A < 1000 || state.SN_now_spd_B < 1000)
    {
        state.ReadyToFire = 0;
    }//ǿ������
    Can1_State_Updata.Data[7] = (u8)((int16_t)(state.ReadyToFire));
    CAN_Transmit(CAN1, &Can1_State_Updata);
}