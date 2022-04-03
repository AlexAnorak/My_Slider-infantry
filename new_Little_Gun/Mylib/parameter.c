#include "main.h"

//#define SUNNYSKY//������  /*��ע��У׼�����г�*/
//#define SNAIL//SNAIL���
//#define C615//C615���
#define FOUR_INFANTRY
/***********���¾ֲ��������ڱ��ļ�ʹ��***********/
#ifdef SUNNYSKY
#define LOADED
static const float Friction_kp=0.5f;					//����ϵ����Ӱ����ת������
static const float Friction_ki=0.9f;					//����ϵ����Ӱ�����������ߡ��˴����ܻ���Friction_iMaxͬ���޸�
static const float Friction_iMin = 0;					//�̶�Ϊ0
static const float Friction_iMax = 600;					//�ջ�10-30m/s����������ޣ���ʵ�ʻ���ʱ�����ֵ��100���ң��������̫��
static const float Friction_OutLimitMin = 0;			//�̶�Ϊ0
static const float Friction_OutLimitMax = 600;			//����ת�ٴﵽ30m/sʱ��PWMֵ��ȥStandby_PWM 
                                                        //4800 - state.Standby_PWM
static const int16_t Standby_PWM = 4200;				//����ת�ٴﵽ10m/sʱ��PWMֵ
/***********�ű���������ʱǿ���ٶ�***********/
static const int16_t AS5048_Offline_setspeed_A = 4600;	//����ת�ٴﵽ28m/sʱ��PWMֵ
static const int16_t AS5048_Offline_setspeed_B = 4600;	//����ת�ٴﵽ28m/sʱ��PWMֵ
/***********ǿ�ƿ���ʱǿ���ٶ�***********/
static const int16_t ForceOpenloop_setspeed_A = 4600;	//����ת�ٴﵽ28m/sʱ��PWMֵ
static const int16_t ForceOpenloop_setspeed_B = 4600;	//����ת�ٴﵽ28m/sʱ��PWMֵ
static const int16_t OpenLoopMaxSetSpd_A = 4750;
static const int16_t OpenLoopMaxSetSpd_B = 4750;

static const u8 ForceOpenLoop = 0;						//�ɼ�����ʱ���ģ�����������
#endif

#ifdef SNAIL//δ����ı�����ɼ�
#ifdef  LOAD
#error �������ز���ȷ
#endif
#define LOADED
/*΢�֣���ǰ����*/
static const float Friction_kp = 1.0f;//2.0f;                    //����ϵ����Ӱ����ת������                                      /*������������ٶ�����أ����ȶ��Ը����*/
static const float Friction_ki = 2;//5.0f;//5.0f;                    //����ϵ����Ӱ�����������ߡ��˴����ܻ���Friction_iMaxͬ���޸�   /*���֣�������̬���*/
static const float Friction_kd=0;
static const float Friction_iMin = 0;                    //�̶�Ϊ0
static const float Friction_iMax = 1000;                //�ջ�10-30m/s����������ޣ���ʵ�ʻ���ʱ�����ֵ��100���ң��������̫��
static const float Friction_OutLimitMin = 0;            //�̶�Ϊ0
static const float Friction_OutLimitMax = 2000;            //����ת�ٴﵽ30m/sʱ��PWMֵ��ȥStandby_PWM 
//4800 - state.Standby_PWM
static const int16_t Standby_PWM = 5000;//1430; //����ת�ٴﵽ8m/sʱ��PWMֵ
/***********�ű���������ʱǿ���ٶ�***********/
static const int16_t AS5048_Offline_setspeed_A = 5200;//1720 ����ת�ٴﵽ14m/sʱ��PWMֵ
static const int16_t AS5048_Offline_setspeed_B = 5200;//1720 ����ת�ٴﵽ14m/sʱ��PWMֵ
/***********ǿ�ƿ���ʱǿ���ٶ�***********/
static const int16_t ForceOpenloop_setspeed_A = 5200;   //����ת�ٴﵽ14m/sʱ��PWMֵ  //�������һ����ֵ
static const int16_t ForceOpenloop_setspeed_B = 5200;    //����ת�ٴﵽ14m/sʱ��PWMֵ  //�������һ����ֵ
static const int16_t OpenLoopMaxSetSpd_A = 6000;//2820;
static const int16_t OpenLoopMaxSetSpd_B = 6000;//2820;

static const u8 ForceOpenLoop = 0;                        /*�ɼ�����ʱ���ģ�����������*/
#endif

#ifdef C615//δ����ı�����ɼ�
#ifdef  LOAD
#error �������ز���ȷ
#endif
#define LOADED
/*΢�֣���ǰ����*/         
static const float Friction_kp=1.8f;					//����ϵ����Ӱ����ת������                                      /*������������ٶ�����أ����ȶ��Ը����*/
static const float Friction_ki=2.0f;					//����ϵ����Ӱ�����������ߡ��˴����ܻ���Friction_iMaxͬ���޸�   /*���֣�������̬���*/
static const float Friction_kd=0;
static const float Friction_iMin = 0;				    //�̶�Ϊ0
static const float Friction_iMax = 420;	    			//�ջ�10-30m/s����������ޣ���ʵ�ʻ���ʱ�����ֵ��100���ң��������̫��
static const float Friction_OutLimitMin = 0;			//�̶�Ϊ0
static const float Friction_OutLimitMax = 1300;			//����ת�ٴﵽ30m/sʱ��PWMֵ��ȥStandby_PWM 
//4600 - state.Standby_PWM
static const int16_t Standby_PWM = 4575;				//����ת�ٴﵽ10m/sʱ��PWMֵ
/***********�ű���������ʱǿ���ٶ�***********/
static const int16_t AS5048_Offline_setspeed_A = 5735;	//����ת�ٴﵽ28m/sʱ��PWMֵ
static const int16_t AS5048_Offline_setspeed_B = 5735;	//����ת�ٴﵽ28m/sʱ��PWMֵ
/***********ǿ�ƿ���ʱǿ���ٶ�***********/
static const int16_t ForceOpenloop_setspeed_A = 5600;   //����ת�ٴﵽ28m/sʱ��PWMֵ  //�������һ����ֵ
static const int16_t ForceOpenloop_setspeed_B = 5600;	//����ת�ٴﵽ28m/sʱ��PWMֵ  //�������һ����ֵ
static const int16_t OpenLoopMaxSetSpd_A = 6000;
static const int16_t OpenLoopMaxSetSpd_B = 6000;

static const u8 ForceOpenLoop = 0;						/*�ɼ�����ʱ���ģ�����������*/
#endif

#ifdef FOUR_INFANTRY
#ifdef  LOAD
#error �������ز���ȷ
#endif
#define LOADED
/*����������������������������������������������������������������������������������������������������0��������������������������������������������������������������������������������������������������������������������������������������������*/
static const float Friction_kp = 1.8f; //����ϵ����Ӱ����ת������   /*������������ٶ�����أ����ȶ��Ը����*/
static const float Friction_ki = 2; //����ϵ����Ӱ�����������ߡ��˴����ܻ���Friction_iMaxͬ���޸�   /*���֣�������̬���*/
static const float Friction_kd = 0;
static const float Friction_iMin = 0; //�̶�Ϊ0
static const float Friction_iMax = 1000; //�ջ�10-30m/s����������ޣ���ʵ�ʻ���ʱ�����ֵ��100���ң��������̫��
static const float Friction_OutLimitMin = 0; //�̶�Ϊ0
static const float Friction_OutLimitMax = 2000; //����ת�ٴﵽ30m/sʱ��PWMֵ��ȥStandby_PWM
//state.Standby_PWM
static const int16_t Standby_PWM = 4200;
/***********�ű���������ʱǿ���ٶ�***********/
static const int16_t AS5048_Offline_setspeed_A = 5000; //����ת�ٴﵽ14m/sʱ��PWMֵ
static const int16_t AS5048_Offline_setspeed_B = 5000; //����ת�ٴﵽ14m/sʱ��PWMֵ
/***********����ʱ����ٶ�***********/
static const int16_t OpenLoopMaxSetSpd_A = 5700; //����ת�ٴﵽ28m/sʱ��PWMֵ
static const int16_t OpenLoopMaxSetSpd_B = 5700; //����ת�ٴﵽ28m/sʱ��PWMֵ
/***********�ջ�ʱ����ٶ�***********/
static const int16_t MaxSetSpd_A =2780; //�ջ�ת�ٴﵽ28m/sʱ����ֵ
static const int16_t MaxSetSpd_B =2780; //�ջ�ת�ٴﵽ28m/sʱ����ֵ
/***********�����ٶ�***********/
static const int16_t offline_setspeed_A =0;//1620; //�ջ�ת�ٴﵽ14m/sʱ����ֵ
static const int16_t offline_setspeed_B =0;//1620; //�ջ�ת�ٴﵽ14m/sʱ����ֵ
/*����������������������������������������������������������������������������������������������������1��������������������������������������������������������������������������������������������������������������������������������������������*/
static const float Friction_kp1 = 1.8f; 
static const float Friction_ki1 = 2; 
static const float Friction_kd1 = 0;
static const float Friction_iMin1 = 0;
static const float Friction_iMax1 = 500; 
static const float Friction_OutLimitMin1 = 0;
static const float Friction_OutLimitMax1 = 1000; 
//state.Standby_PWM
static const int16_t Standby_PWM1 = 4100;
/***********�ű���������ʱǿ���ٶ�***********/
static const int16_t AS5048_Offline_setspeed_A1 = 4400;
static const int16_t AS5048_Offline_setspeed_B1 = 4400;
/***********ǿ�ƿ���ʱǿ���ٶ�***********/
static const int16_t OpenLoopMaxSetSpd_A1 = 4800;
static const int16_t OpenLoopMaxSetSpd_B1 = 4800;
/***********�ջ�ʱ����ٶ�***********/
static const int16_t MaxSetSpd_A1 =2780;
static const int16_t MaxSetSpd_B1 =2780;
/***********�����ٶ�***********/
static const int16_t offline_setspeed_A1 =0;//1620;
static const int16_t offline_setspeed_B1 =0;//1620;
/*����������������������������������������������������������������������������������������������������2��������������������������������������������������������������������������������������������������������������������������������������������*/
static const float Friction_kp2 = 1.8f; 
static const float Friction_ki2 = 2;
static const float Friction_kd2 = 0;
static const float Friction_iMin2 = 0;
static const float Friction_iMax2 = 850;
static const float Friction_OutLimitMin2 = 0;
static const float Friction_OutLimitMax2 = 2000;
//state.Standby_PWM
static const int16_t Standby_PWM2 = 4100;
/***********�ű���������ʱǿ���ٶ�***********/
static const int16_t AS5048_Offline_setspeed_A2 = 4500;
static const int16_t AS5048_Offline_setspeed_B2 = 4500;
/***********����ʱǿ���ٶ�***********/
static const int16_t OpenLoopMaxSetSpd_A2 = 5000;
static const int16_t OpenLoopMaxSetSpd_B2 = 5000;
/***********�ջ�ʱ����ٶ�***********/
static const int16_t MaxSetSpd_A2 =2780;
static const int16_t MaxSetSpd_B2 =2780;
/***********�����ٶ�***********/
static const int16_t offline_setspeed_A2 =0;//1620;
static const int16_t offline_setspeed_B2 =0;//1620;
/*����������������������������������������������������������������������������������������������������3��������������������������������������������������������������������������������������������������������������������������������������������*/
static const float Friction_kp3 = 1.8f;
static const float Friction_ki3 = 2;
static const float Friction_kd3 = 0;
static const float Friction_iMin3 = 0;
static const float Friction_iMax3 = 1000;
static const float Friction_OutLimitMin3 = 0;
static const float Friction_OutLimitMax3 = 2000;
//state.Standby_PWM
static const int16_t Standby_PWM3 = 4300; //1160
/***********�ű���������ʱǿ���ٶ�***********/
static const int16_t AS5048_Offline_setspeed_A3 = 5000; //1630
static const int16_t AS5048_Offline_setspeed_B3 = 5000; //
/***********����ʱǿ���ٶ�***********/
static const int16_t OpenLoopMaxSetSpd_A3 = 5700;
static const int16_t OpenLoopMaxSetSpd_B3 = 5700;
/***********�ջ�ʱ����ٶ�***********/
static const int16_t MaxSetSpd_A3 =2780;
static const int16_t MaxSetSpd_B3 =2780;
/***********�����ٶ�***********/
static const int16_t offline_setspeed_A3 =0;//1620;
static const int16_t offline_setspeed_B3 =0;//1620;
/*����������������������������������������������������������������������������������������������������4��������������������������������������������������������������������������������������������������������������������������������������������*/
static const float Friction_kp4 = 1.8f; 
static const float Friction_ki4 = 2;
static const float Friction_kd4= 0;
static const float Friction_iMin4 = 0;
static const float Friction_iMax4 = 670;
static const float Friction_OutLimitMin4 = 0;
static const float Friction_OutLimitMax4 = 2000;
//4800 - state.Standby_PWM
static const int16_t Standby_PWM4 = 4300;
/***********�ű���������ʱǿ���ٶ�***********/
static const int16_t AS5048_Offline_setspeed_A4 = 4940; //1620
static const int16_t AS5048_Offline_setspeed_B4 = 4960; //1620
/***********����ʱ����ٶ�***********/
static const int16_t OpenLoopMaxSetSpd_A4 = 5800; //2780
static const int16_t OpenLoopMaxSetSpd_B4 = 5825; //2780
/***********�ջ�ʱ����ٶ�***********/
static const int16_t MaxSetSpd_A4 =2780;
static const int16_t MaxSetSpd_B4 =2780;
/***********�����ٶ�***********/
static const int16_t offline_setspeed_A4 =0;//1620;
static const int16_t offline_setspeed_B4 =0;//1620;

static const u8 ForceOpenLoop = 0;						/*�ɼ�����ʱ���ģ�����������*/	
static const u8 ID = 3;
#endif

#ifndef LOADED
#error δ���ز���
#endif

void TDT_Get_PIDparameters(pid *pidStruct, u8 pidIndex)
{
    switch (pidIndex)
    {
        case pidFriction:
        {
            pidStruct->kp = Friction_kp;
            pidStruct->ki = Friction_ki;
			pidStruct->kd = Friction_kd;
            pidStruct->integralErrorMin = Friction_iMin;
            pidStruct->integralErrorMax = Friction_iMax;
            pidStruct->OutLimitMIN = Friction_OutLimitMin;
            pidStruct->OutLimitMAX = Friction_OutLimitMax;
            break;
        }
		#ifdef FOUR_INFANTRY
		case SHIP:
        {
            pidStruct->kp = Friction_kp1;
            pidStruct->ki = Friction_ki1;
			pidStruct->kd = Friction_kd1;
            pidStruct->integralErrorMin = Friction_iMin1;
            pidStruct->integralErrorMax = Friction_iMax1;
            pidStruct->OutLimitMIN = Friction_OutLimitMin1;
            pidStruct->OutLimitMAX = Friction_OutLimitMax1;
            break;
        }
		case TWONETWONE:
        {
            pidStruct->kp = Friction_kp2;
            pidStruct->ki = Friction_ki2;
			pidStruct->kd = Friction_kd2;
            pidStruct->integralErrorMin = Friction_iMin2;
            pidStruct->integralErrorMax = Friction_iMax2;
            pidStruct->OutLimitMIN = Friction_OutLimitMin2;
            pidStruct->OutLimitMAX = Friction_OutLimitMax2;
            break;
        }
		case AURORA:
        {
            pidStruct->kp = Friction_kp3;
            pidStruct->ki = Friction_ki3;
			pidStruct->kd = Friction_kd3;
            pidStruct->integralErrorMin = Friction_iMin3;
            pidStruct->integralErrorMax = Friction_iMax3;
            pidStruct->OutLimitMIN = Friction_OutLimitMin3;
            pidStruct->OutLimitMAX = Friction_OutLimitMax3;
            break;
        }
		case GALAXY:
        {
            pidStruct->kp = Friction_kp4;
            pidStruct->ki = Friction_ki4;
			pidStruct->kd = Friction_kd4;
            pidStruct->integralErrorMin = Friction_iMin4;
            pidStruct->integralErrorMax = Friction_iMax4;
            pidStruct->OutLimitMIN = Friction_OutLimitMin4;
            pidStruct->OutLimitMAX = Friction_OutLimitMax4;
            break;
        }
		#endif
        default:
            break;
    }
}


/*Ħ�����������̣�

������state.ForceOpenLoop Ϊ 1����ֱ�Ӵ�state.openloop_setspeed_x��ȡֵ�����

�ջ���state.ForceOpenLoop Ϊ 0����
	1.��SetSpd_xΪ0ʱ��pwm�̶�Ϊ4000��Ϊ����ģʽ����ʱ��
	2.��SetSpd_x��Ϊ0ʱ
		��ǰ��5*50��ms�ڣ�һ�ߣ�A��Ħ�����������state.Standby_PWM = 4200����һ�ߣ�B�����ֲ���
		�ڣ�5*50��ms����5*100��ms�ڣ�һ�ߣ�B��Ħ�����������state.Standby_PWM = 4200����һ�ߣ�A�����ֲ���
		�۶�������ջ�

Ħ�����쳣����
1.��ת�������У�PWM���>4200��ʱ���ű�����1s�ڶ����㶨����Ϊ�ű��������ߣ�ǿ����Ϊ������ռ�ձ�Ϊstate.AS5048_Offline_setspeed_x;

*/
void TDT_State_Init()
{
    state.ReadyToFire = 0;
    state.ReadyToStart = 0;
    state.StartingTime = 0;


    state.UnlockTime = 0;
    state.WaitReadyToUnlock = 0;

    state.ForceOpenLoop = ForceOpenLoop;
	state.INFANTRY_ID = ID;

    /*���һ��������������֮��ջ��������⣬����QuickStart*/
    state.EnableQuickStart = 0;   //��һ�κ��������

    state.StopState = 1;

    TDT_Snail_OpenLoopSwitch(1);

    state.OpenLoopMaxSetSpdLimit = 1;

    state.StartingTime = 0;
	/*����Ϊ��ʼֵ��̫������*/
	state.Standby_PWM = Standby_PWM; //����ʱ��ռ�ձ�
    /******************����ʱ���ٶ�*****************************/
    state.openloop_setspeed_A = 4000;
    state.openloop_setspeed_B = 4000;
	/******************ǿ�ƿ���ʱǿ���ٶ�**********************/
    state.ForceOpenloop_setspeed_A = 4000; //ǿ�ƿ����趨ֵ,������
    state.ForceOpenloop_setspeed_B = 4000; //ǿ�ƿ����趨ֵ
	#ifdef FOUR_INFANTRY
	state.MaxSetSpd_A = MaxSetSpd_A; //���ջ��趨�ٶ��޷�
    state.MaxSetSpd_B = MaxSetSpd_B; //���ջ��趨�ٶ��޷�
	state.offline_setspeed_A=offline_setspeed_A; //���߱ջ��趨ֵ
	state.offline_setspeed_B=offline_setspeed_B; //���߱ջ��趨ֵ
	#endif
    state.OpenLoopMaxSetSpd_A = OpenLoopMaxSetSpd_A; //��󿪻��趨�ٶ��޷�
    state.OpenLoopMaxSetSpd_B = OpenLoopMaxSetSpd_B; //��󿪻��趨�ٶ��޷�
    /******************������һ�������ٶ�(0���޷�)*************/
    Snail_NowValue.data[0] = 1200;
    Snail_NowValue.data[1] = 1200;
}

void reloadparameter(u8 parameterIndex)
{
	#ifdef FOUR_INFANTRY
	switch (parameterIndex)
    {
        case 0:
        {
			state.Standby_PWM = Standby_PWM; //����ʱ��ռ�ձ�
            state.MaxSetSpd_A = MaxSetSpd_A; //���ջ��趨�ٶ��޷�
            state.MaxSetSpd_B = MaxSetSpd_B; //���ջ��趨�ٶ��޷�
			state.offline_setspeed_A=offline_setspeed_A; //���߱ջ��趨ֵ
	        state.offline_setspeed_B=offline_setspeed_B; //���߱ջ��趨ֵ
            state.OpenLoopMaxSetSpd_A = OpenLoopMaxSetSpd_A;//��󿪻��趨�ٶ��޷�
            state.OpenLoopMaxSetSpd_B = OpenLoopMaxSetSpd_B;//��󿪻��趨�ٶ��޷�
			/******************�ű���������ʱǿ���ٶ�******************/
            state.AS5048_Offline_setspeed_A = AS5048_Offline_setspeed_A;
            state.AS5048_Offline_setspeed_B = AS5048_Offline_setspeed_B;
            break;
        }
		case SHIP:
        {
			state.Standby_PWM = Standby_PWM1; //����ʱ��ռ�ձ�
            state.MaxSetSpd_A = MaxSetSpd_A1; //���ջ��趨�ٶ��޷�
            state.MaxSetSpd_B = MaxSetSpd_B1; //���ջ��趨�ٶ��޷�
			state.offline_setspeed_A=offline_setspeed_A1; //���߱ջ��趨ֵ
	        state.offline_setspeed_B=offline_setspeed_B1; //���߱ջ��趨ֵ
            state.OpenLoopMaxSetSpd_A = OpenLoopMaxSetSpd_A1;//��󿪻��趨�ٶ��޷�
            state.OpenLoopMaxSetSpd_B = OpenLoopMaxSetSpd_B1;//��󿪻��趨�ٶ��޷�
			/******************�ű���������ʱǿ���ٶ�******************/
            state.AS5048_Offline_setspeed_A = AS5048_Offline_setspeed_A1;
            state.AS5048_Offline_setspeed_B = AS5048_Offline_setspeed_B1;
            break;
        }
		case TWONETWONE:
        {
			state.Standby_PWM = Standby_PWM2; //����ʱ��ռ�ձ�
            state.MaxSetSpd_A = MaxSetSpd_A2; //���ջ��趨�ٶ��޷�
            state.MaxSetSpd_B = MaxSetSpd_B2; //���ջ��趨�ٶ��޷�
			state.offline_setspeed_A=offline_setspeed_A2; //���߱ջ��趨ֵ
	        state.offline_setspeed_B=offline_setspeed_B2; //���߱ջ��趨ֵ
            state.OpenLoopMaxSetSpd_A = OpenLoopMaxSetSpd_A2;//��󿪻��趨�ٶ��޷�
            state.OpenLoopMaxSetSpd_B = OpenLoopMaxSetSpd_B2;//��󿪻��趨�ٶ��޷�
			/******************�ű���������ʱǿ���ٶ�******************/
            state.AS5048_Offline_setspeed_A = AS5048_Offline_setspeed_A2;
            state.AS5048_Offline_setspeed_B = AS5048_Offline_setspeed_B2;
            break;
        }
		case AURORA:
        {
			state.Standby_PWM = Standby_PWM3; //����ʱ��ռ�ձ�
            state.MaxSetSpd_A = MaxSetSpd_A3; //���ջ��趨�ٶ��޷�
            state.MaxSetSpd_B = MaxSetSpd_B3; //���ջ��趨�ٶ��޷�
			state.offline_setspeed_A=offline_setspeed_A3; //���߱ջ��趨ֵ
	        state.offline_setspeed_B=offline_setspeed_B3; //���߱ջ��趨ֵ
            state.OpenLoopMaxSetSpd_A = OpenLoopMaxSetSpd_A3;//��󿪻��趨�ٶ��޷�
            state.OpenLoopMaxSetSpd_B = OpenLoopMaxSetSpd_B3;//��󿪻��趨�ٶ��޷�
			/******************�ű���������ʱǿ���ٶ�******************/
            state.AS5048_Offline_setspeed_A = AS5048_Offline_setspeed_A3;
            state.AS5048_Offline_setspeed_B = AS5048_Offline_setspeed_B3;
            break;
        }
		case GALAXY:
        {
			state.Standby_PWM = Standby_PWM4; //����ʱ��ռ�ձ�
            state.MaxSetSpd_A = MaxSetSpd_A4; //���ջ��趨�ٶ��޷�
            state.MaxSetSpd_B = MaxSetSpd_B4; //���ջ��趨�ٶ��޷�
			state.offline_setspeed_A=offline_setspeed_A4; //���߱ջ��趨ֵ
	        state.offline_setspeed_B=offline_setspeed_B4; //���߱ջ��趨ֵ
            state.OpenLoopMaxSetSpd_A = OpenLoopMaxSetSpd_A4;//��󿪻��趨�ٶ��޷�
            state.OpenLoopMaxSetSpd_B = OpenLoopMaxSetSpd_B4;//��󿪻��趨�ٶ��޷�
			/******************�ű���������ʱǿ���ٶ�******************/
            state.AS5048_Offline_setspeed_A = AS5048_Offline_setspeed_A4;
            state.AS5048_Offline_setspeed_B = AS5048_Offline_setspeed_B4;
            break;
        }
        default:
            break;
    }
	#endif
}


