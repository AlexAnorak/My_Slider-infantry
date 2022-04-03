#include "main.h"

/**
  * @brief  摩擦轮pid控制
  * @param  setValue: 速度设定值
	*					fbValue: 速度反馈值
	*					result: 控制器输出
	*					T: 控制周期
	*					numConVar: 被控变量的个数
  * @retval None
  */
int Test_result1, Test_result2;
vec2int16 Snail_NowValue;
vec2int16 set_val, fb_val, result;


/*更改摩擦轮转速设定值*/
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
                /*如第一次正常启动，但之后闭环出现问题，禁用QuickStart*/
                //清空PID计算量，会拖慢启动时间(从0提速)
                //可适当增加Snail_NowValue.data的初始值
                Snail_NowValue.data[0] = 0;
                Snail_NowValue.data[1] = 0;
                result.data[0] = 0;
                result.data[1] = 0;
            }
        } else
        {
            /*从0启动*/
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
    if (state.OpenLoop == 0 && state.ForceOpenLoop == 0)//闭环状态
    {
        /*切入闭环后延迟解除拨盘锁时间*/
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
            /*速度大幅度切换需视情况添加(拨盘锁)*/
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
        TIM1->CCR1 = state.ForceOpenloop_setspeed_A;//开环代码
        TIM1->CCR2 = state.ForceOpenloop_setspeed_B;//开环代码
    } else if ((state.ForceOpenLoop == 0 && state.OpenLoop == 0) && (state.AS5048_B_Offline == 1 || state.AS5048_A_Offline == 1))
    {
        if (state.OpenLoopMaxSetSpdLimit)
        {
            state.AS5048_Offline_setspeed_A = LIMIT(state.AS5048_Offline_setspeed_A, 4000, state.OpenLoopMaxSetSpd_A);
            state.AS5048_Offline_setspeed_B = LIMIT(state.AS5048_Offline_setspeed_B, 4000, state.OpenLoopMaxSetSpd_B);
        }
        TIM1->CCR1 = state.AS5048_Offline_setspeed_A;//磁编码器离线开环代码
        TIM1->CCR2 = state.AS5048_Offline_setspeed_B;//磁编码器离线开环代码
    } else if (state.OpenLoop == 1)
    {
        if (state.OpenLoopMaxSetSpdLimit)
        {
            state.openloop_setspeed_A = LIMIT(state.openloop_setspeed_A, 4000, state.OpenLoopMaxSetSpd_A);
            state.openloop_setspeed_B = LIMIT(state.openloop_setspeed_B, 4000, state.OpenLoopMaxSetSpd_B);
        }
        TIM1->CCR1 = state.openloop_setspeed_A;//开环代码
        TIM1->CCR2 = state.openloop_setspeed_B;//开环代码
    } else if (state.ForceOpenLoop == 0 && state.OpenLoop == 0 && state.AS5048_B_Offline == 0 && state.AS5048_A_Offline == 0)//闭环状态并且磁编码器不离线
    {
        TIM1->CCR1 = result.data[1];//闭环代码
        TIM1->CCR2 = result.data[0];//闭环代码
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

    /* 如果参数没有加载，加载参数 */
    if (!state.paraLoadFlag)
    {
        for (indexConVar = 0; indexConVar < dimConVar; indexConVar++)
        {
            TDT_Get_PIDparameters(&frictionMotor[indexConVar], state.INFANTRY_ID);
        }
        state.paraLoadFlag = 1;
    }

    /* 循环dimConVar次，分别进行PID控制 */
    for (indexConVar = 0; indexConVar < dimConVar; indexConVar++)
    {
        /* 设定值 */
        frictionMotor[indexConVar].setValue = setValue->data[indexConVar];
        /* 反馈值 */
        frictionMotor[indexConVar].feedbackValue = fbValue->data[indexConVar];
        /* 偏差 = 设定值 - 反馈值 */
        frictionMotor[indexConVar].error = frictionMotor[indexConVar].setValue - frictionMotor[indexConVar].feedbackValue;
		/* */
		frictionMotor[indexConVar].deltaError = frictionMotor[indexConVar].error - frictionMotor[indexConVar].lastError;
        /* 偏差进行积分 */
        frictionMotor[indexConVar].integralError += frictionMotor[indexConVar].error * T;
        /* 偏差的积分进行限制 */
        frictionMotor[indexConVar].integralError = LIMIT(frictionMotor[indexConVar].integralError, -frictionMotor[indexConVar].integralErrorMin, frictionMotor[indexConVar].integralErrorMax);
        /* */
		frictionMotor[indexConVar].dOut = frictionMotor[indexConVar].kd * frictionMotor[indexConVar].deltaError;
		/* 比例项的输出 */
        frictionMotor[indexConVar].pOut = frictionMotor[indexConVar].kp * frictionMotor[indexConVar].error;
        /* 积分项的输出 */
        frictionMotor[indexConVar].iOut = frictionMotor[indexConVar].ki * frictionMotor[indexConVar].integralError;
        /* 总的输出 = 比例项的输出 + 积分项的输出 */
        frictionMotor[indexConVar].out = frictionMotor[indexConVar].pOut + frictionMotor[indexConVar].iOut+ frictionMotor[indexConVar].dOut;
        /* 总的输出不能超出电机给定值的范围 */
        result->data[indexConVar] = LIMIT(frictionMotor[indexConVar].out, frictionMotor[indexConVar].OutLimitMIN, frictionMotor[indexConVar].OutLimitMAX) + state.Standby_PWM;
    }
}


