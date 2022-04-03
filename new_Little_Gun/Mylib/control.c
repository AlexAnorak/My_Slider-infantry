#include "main.h"

/**
  * @brief  Ħ����pid����
  * @param  setValue: �ٶ��趨ֵ
	*					fbValue: �ٶȷ���ֵ
	*					result: ���������
	*					T: ��������
	*					numConVar: ���ر����ĸ���
  * @retval None
  */
int Test_result1, Test_result2;
vec2int16 Snail_NowValue;
vec2int16 set_val, fb_val, result;


/*����Ħ����ת���趨ֵ*/
void TDT_Snail_SetValue(int16_t SetSpd_A, int16_t SetSpd_B)
{
    if (state.ForceOpenLoop == 0)
    {
        if (SetSpd_A == 0 || SetSpd_B == 0)
        {
            state.ReadyToFire = 0;
            TDT_Snail_OpenLoopSwitch(1);
            state.StopState = 1;
            state.StartingTime = 0;
            state.openloop_setspeed_A = 4000;
            state.openloop_setspeed_B = 4000;

            if (state.EnableQuickStart == 0)
            {
                /*���һ��������������֮��ջ��������⣬����QuickStart*/
                //���PID������������������ʱ��(��0����)
                //���ʵ�����Snail_NowValue.data�ĳ�ʼֵ
                Snail_NowValue.data[0] = 0;
                Snail_NowValue.data[1] = 0;
                result.data[0] = 0;
                result.data[1] = 0;
            }
        } else
        {
            /*��0����*/
            if (state.StopState == 1)
            {
                state.ReadyToFire = 0;
                if (state.StartingTime < 100)
                {
                    state.openloop_setspeed_A = state.Standby_PWM;
                    state.openloop_setspeed_B = TIM1->CCR2;
                } else if (state.StartingTime < 200)
                {
                    state.openloop_setspeed_B = state.Standby_PWM;
                    state.openloop_setspeed_A = TIM1->CCR1;
                } else if (state.StartingTime > 300)
                {
                    state.StopState = 0;
                    //state.ReadyToFire = 1;
                    state.NewStart = 1;
                    state.WaitReadyToUnlock = 1;
                    state.UnlockTime = 0;
                    TDT_Snail_OpenLoopSwitch(0);
                    state.StartingTime = 0;
                }
                state.StartingTime++;
            }
        }
    }
    if (state.OpenLoop == 0 && state.ForceOpenLoop == 0)//�ջ�״̬
    {
        /*����ջ����ӳٽ��������ʱ��*/
        if (state.WaitReadyToUnlock == 1)
        {
            if (SetSpd_B == Snail_NowValue.data[0] && SetSpd_A == Snail_NowValue.data[1])
            {
                if (state.UnlockTime < 100)
                {
                    state.UnlockTime++;
                    state.ReadyToFire = 0;
                } else
                {
                    state.WaitReadyToUnlock = 0;
                    state.UnlockTime = 0;
                    state.ReadyToFire = 1;
                }
            }
        } else
        {
            /*�ٶȴ�����л�����������(������)*/
            if (ABS(SetSpd_B - Snail_NowValue.data[0]) + ABS(SetSpd_A - Snail_NowValue.data[1]) < 400)
            {
                state.ReadyToFire = 1;
            } else
            {
                state.ReadyToFire = 0;
            }
        }
        /*********************************/
        Snail_NowValue.data[0] = LIMIT(SetSpd_B, 0, state.MaxSetSpd_B);
        Snail_NowValue.data[1] = LIMIT(SetSpd_A, 0, state.MaxSetSpd_A);

        fb_val.data[0] = ABS(as5048_B.speed_ef);
        fb_val.data[1] = ABS(as5048_A.speed_ef);
        TDT_Friction_PidControl(&Snail_NowValue, &fb_val, &result, 0.05, 2);
        Test_result1 = result.data[0];
        Test_result2 = result.data[1];
    }
    if (state.ForceOpenLoop == 1)
    {
        if (state.OpenLoopMaxSetSpdLimit)
        {
            state.ForceOpenloop_setspeed_A = LIMIT(state.ForceOpenloop_setspeed_A, 4000, state.OpenLoopMaxSetSpd_A);
            state.ForceOpenloop_setspeed_B = LIMIT(state.ForceOpenloop_setspeed_B, 4000, state.OpenLoopMaxSetSpd_B);
        }
        TIM1->CCR1 = state.ForceOpenloop_setspeed_A;//��������
        TIM1->CCR2 = state.ForceOpenloop_setspeed_B;//��������
    } else if ((state.ForceOpenLoop == 0 && state.OpenLoop == 0) && (state.AS5048_B_Offline == 1 || state.AS5048_A_Offline == 1))
    {
        if (state.OpenLoopMaxSetSpdLimit)
        {
            state.AS5048_Offline_setspeed_A = LIMIT(state.AS5048_Offline_setspeed_A, 4000, state.OpenLoopMaxSetSpd_A);
            state.AS5048_Offline_setspeed_B = LIMIT(state.AS5048_Offline_setspeed_B, 4000, state.OpenLoopMaxSetSpd_B);
        }
        TIM1->CCR1 = state.AS5048_Offline_setspeed_A;//�ű��������߿�������
        TIM1->CCR2 = state.AS5048_Offline_setspeed_B;//�ű��������߿�������
    } else if (state.OpenLoop == 1)
    {
        if (state.OpenLoopMaxSetSpdLimit)
        {
            state.openloop_setspeed_A = LIMIT(state.openloop_setspeed_A, 4000, state.OpenLoopMaxSetSpd_A);
            state.openloop_setspeed_B = LIMIT(state.openloop_setspeed_B, 4000, state.OpenLoopMaxSetSpd_B);
        }
        TIM1->CCR1 = state.openloop_setspeed_A;//��������
        TIM1->CCR2 = state.openloop_setspeed_B;//��������
    } else if (state.ForceOpenLoop == 0 && state.OpenLoop == 0 && state.AS5048_B_Offline == 0 && state.AS5048_A_Offline == 0)//�ջ�״̬���Ҵű�����������
    {
        TIM1->CCR1 = result.data[1];//�ջ�����
        TIM1->CCR2 = result.data[0];//�ջ�����
    }
}

void TDT_Snail_OpenLoopSwitch(u8 Switch)
{
    state.OpenLoop = Switch;
}


pid frictionMotor[2];

void TDT_Friction_PidControl(vec2int16 *setValue, vec2int16 *fbValue, vec2int16 *result, float T, u8 dimConVar)
{
    u8 indexConVar;
    if (state.NewStart == 1)
    {
        frictionMotor[0].integralError = 0;
        frictionMotor[1].integralError = 0;
    }
    state.NewStart = 0;

    /* �������û�м��أ����ز��� */
    if (!state.paraLoadFlag)
    {
        for (indexConVar = 0; indexConVar < dimConVar; indexConVar++)
        {
            TDT_Get_PIDparameters(&frictionMotor[indexConVar], state.INFANTRY_ID);
        }
        state.paraLoadFlag = 1;
    }

    /* ѭ��dimConVar�Σ��ֱ����PID���� */
    for (indexConVar = 0; indexConVar < dimConVar; indexConVar++)
    {
        /* �趨ֵ */
        frictionMotor[indexConVar].setValue = setValue->data[indexConVar];
        /* ����ֵ */
        frictionMotor[indexConVar].feedbackValue = fbValue->data[indexConVar];
        /* ƫ�� = �趨ֵ - ����ֵ */
        frictionMotor[indexConVar].error = frictionMotor[indexConVar].setValue - frictionMotor[indexConVar].feedbackValue;
		/* */
		frictionMotor[indexConVar].deltaError = frictionMotor[indexConVar].error - frictionMotor[indexConVar].lastError;
        /* ƫ����л��� */
        frictionMotor[indexConVar].integralError += frictionMotor[indexConVar].error * T;
        /* ƫ��Ļ��ֽ������� */
        frictionMotor[indexConVar].integralError = LIMIT(frictionMotor[indexConVar].integralError, -frictionMotor[indexConVar].integralErrorMin, frictionMotor[indexConVar].integralErrorMax);
        /* */
		frictionMotor[indexConVar].dOut = frictionMotor[indexConVar].kd * frictionMotor[indexConVar].deltaError;
		/* ���������� */
        frictionMotor[indexConVar].pOut = frictionMotor[indexConVar].kp * frictionMotor[indexConVar].error;
        /* ���������� */
        frictionMotor[indexConVar].iOut = frictionMotor[indexConVar].ki * frictionMotor[indexConVar].integralError;
        /* �ܵ���� = ���������� + ���������� */
        frictionMotor[indexConVar].out = frictionMotor[indexConVar].pOut + frictionMotor[indexConVar].iOut+ frictionMotor[indexConVar].dOut;
        /* �ܵ�������ܳ����������ֵ�ķ�Χ */
        result->data[indexConVar] = LIMIT(frictionMotor[indexConVar].out, frictionMotor[indexConVar].OutLimitMIN, frictionMotor[indexConVar].OutLimitMAX) + state.Standby_PWM;
    }
}


