#include "main.h"

struct _schedule schedule;
struct _state state;
int TestA, TestB;

void Loop_1000Hz(void)    //1msִ��һ��
{
//	as5048_read_angle();
    as5048_singelread_angle();
    as5048_data_prepare();
    if (state.AS5048_A_Offline == 0 && state.AS5048_B_Offline == 0)//������
    {
        LED_State(as5048_A.speed_ef, as5048_B.speed_ef, 0.001);//LED��˸Ƶ�����ٶȳ�����
    }
    TestA = ABS(as5048_A.speed_ef);
    TestB = ABS(as5048_B.speed_ef);

    state.SN_now_spd_A = ABS(as5048_A.speed_ef);
    state.SN_now_spd_B = ABS(as5048_B.speed_ef);

    /*ʵ�ʷ������(A���½��ز���)*/
    if (canFeedback.Set_speed_A >= 1000 && state.ReadyToFire == 1)
    {
        /*B*/
        if (state.SN_last_spd_B < state.SN_now_spd_B)
        {
            if (state.SN_start_spd_B - state.SN_now_spd_B > 150)
            {
                state.SN_error_spd_B = state.SN_start_spd_B - state.SN_now_spd_B;
                state.ShootNum_f += 0.5f;
            }
            state.SN_start_spd_B = state.SN_now_spd_B;
        }

        /*A*/
        if (state.SN_last_spd_A < state.SN_now_spd_A)
        {

            if (state.SN_start_spd_A - state.SN_now_spd_A > 150)
            {
                state.SN_error_spd_A = state.SN_start_spd_A - state.SN_now_spd_A;
                state.ShootNum_f += 0.5f;
            }
            state.SN_start_spd_A = state.SN_now_spd_A;
        }

        state.ShootNum = (int) state.ShootNum_f;

    }


    /* �ű��������߼�� */
    if (state.ForceOpenLoop == 0)//�������ż��
    {
        if (TIM1->CCR1 > state.Standby_PWM)//���ֵ>4200�ſ�ʼ���
        {
            if (as5048_A.speed_ef == 0)
            {
                state.AS5048_A_Offline = 1;
            } else
            {
                state.AS5048_A_Offline = 0;
            }
        }

        if (TIM1->CCR2 > state.Standby_PWM)//���ֵ>4200�ſ�ʼ���
        {
            if (as5048_B.speed_ef == 0)
            {
                state.AS5048_B_Offline = 1;
            } else
            {
                state.AS5048_B_Offline = 0;
            }
        }
    }

    state.SN_last_spd_B = ABS(as5048_B.speed_ef);
    state.SN_last_spd_A = ABS(as5048_A.speed_ef);

    TDT_State_Updata();//״̬����
}

void Loop_500Hz(void)     //2msִ��һ��
{

}

void Loop_200Hz(void)     //5msִ��һ��
{
    if (state.offline_check < 1000)    //�ж����ߣ�10s,����ʱΪ15s(1500)
    {
        TDT_Snail_SetValue(canFeedback.Set_speed_A, canFeedback.Set_speed_B); //���ؿ���
		//TDT_Snail_SetValue(state.offline_setspeed_A, state.offline_setspeed_B); //�ֶ�
    } 
	else  //����״̬
    {
        TDT_Snail_SetValue(state.offline_setspeed_A, state.offline_setspeed_B);
    }
}

void Loop_100Hz(void)  //10msִ��һ��
{
    state.offline_check++;
    LED_Error(state.AS5048_A_Offline, state.AS5048_B_Offline);
}

void Loop_50Hz(void)    //20msִ��һ��
{


}

void Loop_20Hz(void)    //50msִ��һ��
{
    static u8 timer_50ms = 0;//��¼50ms����
    if (++timer_50ms > 10)
    {
        timer_50ms = 0;
//		 LED_OFF;
        LED_TOGGLE;
    }
}

void Loop_10Hz(void)    //100msִ��һ��
{

}

void Loop(void)
{
    if (schedule.cnt_1ms >= 1)
    {
        Loop_1000Hz();
        schedule.cnt_1ms = 0;
    }
    if (schedule.cnt_2ms >= 2)
    {
        Loop_500Hz();
        schedule.cnt_2ms = 0;
    }
    if (schedule.cnt_5ms >= 5)
    {
        Loop_200Hz();
        schedule.cnt_5ms = 0;
    }
    if (schedule.cnt_10ms >= 10)
    {
        Loop_100Hz();
        schedule.cnt_10ms = 0;
    }
    if (schedule.cnt_20ms >= 20)
    {
        Loop_50Hz();
        schedule.cnt_20ms = 0;
    }
    if (schedule.cnt_50ms >= 50)
    {
        Loop_20Hz();
        schedule.cnt_50ms = 0;
    }
    if (schedule.cnt_100ms >= 100)
    {
        Loop_10Hz();
        schedule.cnt_100ms = 0;
    }
}
