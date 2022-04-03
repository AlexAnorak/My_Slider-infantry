#include "main.h"

//#define SUNNYSKY//郎宇电机  /*解注释校准油门行程*/
//#define SNAIL//SNAIL电机
//#define C615//C615电机
#define FOUR_INFANTRY
/***********以下局部常量仅在本文件使用***********/
#ifdef SUNNYSKY
#define LOADED
static const float Friction_kp=0.5f;					//比例系数，影响电机转速曲线
static const float Friction_ki=0.9f;					//积分系数，影响电机回速曲线。此处可能会与Friction_iMax同步修改
static const float Friction_iMin = 0;					//固定为0
static const float Friction_iMax = 600;					//闭环10-30m/s均不会此上限，比实际积分时的最大值大100左右，但无需大太多
static const float Friction_OutLimitMin = 0;			//固定为0
static const float Friction_OutLimitMax = 600;			//开环转速达到30m/s时的PWM值减去Standby_PWM 
                                                        //4800 - state.Standby_PWM
static const int16_t Standby_PWM = 4200;				//开环转速达到10m/s时的PWM值
/***********磁编码器离线时强制速度***********/
static const int16_t AS5048_Offline_setspeed_A = 4600;	//开环转速达到28m/s时的PWM值
static const int16_t AS5048_Offline_setspeed_B = 4600;	//开环转速达到28m/s时的PWM值
/***********强制开环时强制速度***********/
static const int16_t ForceOpenloop_setspeed_A = 4600;	//开环转速达到28m/s时的PWM值
static const int16_t ForceOpenloop_setspeed_B = 4600;	//开环转速达到28m/s时的PWM值
static const int16_t OpenLoopMaxSetSpd_A = 4750;
static const int16_t OpenLoopMaxSetSpd_B = 4750;

static const u8 ForceOpenLoop = 0;						//采集数据时更改，需脱离主控
#endif

#ifdef SNAIL//未定义的变量需采集
#ifdef  LOAD
#error 参数加载不明确
#endif
#define LOADED
/*微分：超前控制*/
static const float Friction_kp = 1.0f;//2.0f;                    //比例系数，影响电机转速曲线                                      /*比例：与调节速度正相关，与稳定性负相关*/
static const float Friction_ki = 2;//5.0f;//5.0f;                    //积分系数，影响电机回速曲线。此处可能会与Friction_iMax同步修改   /*积分：消除稳态误差*/
static const float Friction_kd=0;
static const float Friction_iMin = 0;                    //固定为0
static const float Friction_iMax = 1000;                //闭环10-30m/s均不会此上限，比实际积分时的最大值大100左右，但无需大太多
static const float Friction_OutLimitMin = 0;            //固定为0
static const float Friction_OutLimitMax = 2000;            //开环转速达到30m/s时的PWM值减去Standby_PWM 
//4800 - state.Standby_PWM
static const int16_t Standby_PWM = 5000;//1430; //开环转速达到8m/s时的PWM值
/***********磁编码器离线时强制速度***********/
static const int16_t AS5048_Offline_setspeed_A = 5200;//1720 开环转速达到14m/s时的PWM值
static const int16_t AS5048_Offline_setspeed_B = 5200;//1720 开环转速达到14m/s时的PWM值
/***********强制开环时强制速度***********/
static const int16_t ForceOpenloop_setspeed_A = 5200;   //开环转速达到14m/s时的PWM值  //填跟上面一样的值
static const int16_t ForceOpenloop_setspeed_B = 5200;    //开环转速达到14m/s时的PWM值  //填跟上面一样的值
static const int16_t OpenLoopMaxSetSpd_A = 6000;//2820;
static const int16_t OpenLoopMaxSetSpd_B = 6000;//2820;

static const u8 ForceOpenLoop = 0;                        /*采集数据时更改，以脱离主控*/
#endif

#ifdef C615//未定义的变量需采集
#ifdef  LOAD
#error 参数加载不明确
#endif
#define LOADED
/*微分：超前控制*/         
static const float Friction_kp=1.8f;					//比例系数，影响电机转速曲线                                      /*比例：与调节速度正相关，与稳定性负相关*/
static const float Friction_ki=2.0f;					//积分系数，影响电机回速曲线。此处可能会与Friction_iMax同步修改   /*积分：消除稳态误差*/
static const float Friction_kd=0;
static const float Friction_iMin = 0;				    //固定为0
static const float Friction_iMax = 420;	    			//闭环10-30m/s均不会此上限，比实际积分时的最大值大100左右，但无需大太多
static const float Friction_OutLimitMin = 0;			//固定为0
static const float Friction_OutLimitMax = 1300;			//开环转速达到30m/s时的PWM值减去Standby_PWM 
//4600 - state.Standby_PWM
static const int16_t Standby_PWM = 4575;				//开环转速达到10m/s时的PWM值
/***********磁编码器离线时强制速度***********/
static const int16_t AS5048_Offline_setspeed_A = 5735;	//开环转速达到28m/s时的PWM值
static const int16_t AS5048_Offline_setspeed_B = 5735;	//开环转速达到28m/s时的PWM值
/***********强制开环时强制速度***********/
static const int16_t ForceOpenloop_setspeed_A = 5600;   //开环转速达到28m/s时的PWM值  //填跟上面一样的值
static const int16_t ForceOpenloop_setspeed_B = 5600;	//开环转速达到28m/s时的PWM值  //填跟上面一样的值
static const int16_t OpenLoopMaxSetSpd_A = 6000;
static const int16_t OpenLoopMaxSetSpd_B = 6000;

static const u8 ForceOpenLoop = 0;						/*采集数据时更改，以脱离主控*/
#endif

#ifdef FOUR_INFANTRY
#ifdef  LOAD
#error 参数加载不明确
#endif
#define LOADED
/*——————————————————————————————————————————————————0——————————————————————————————————————————————————————————————————————*/
static const float Friction_kp = 1.8f; //比例系数，影响电机转速曲线   /*比例：与调节速度正相关，与稳定性负相关*/
static const float Friction_ki = 2; //积分系数，影响电机回速曲线。此处可能会与Friction_iMax同步修改   /*积分：消除稳态误差*/
static const float Friction_kd = 0;
static const float Friction_iMin = 0; //固定为0
static const float Friction_iMax = 1000; //闭环10-30m/s均不会此上限，比实际积分时的最大值大100左右，但无需大太多
static const float Friction_OutLimitMin = 0; //固定为0
static const float Friction_OutLimitMax = 2000; //开环转速达到30m/s时的PWM值减去Standby_PWM
//state.Standby_PWM
static const int16_t Standby_PWM = 4200;
/***********磁编码器离线时强制速度***********/
static const int16_t AS5048_Offline_setspeed_A = 5000; //开环转速达到14m/s时的PWM值
static const int16_t AS5048_Offline_setspeed_B = 5000; //开环转速达到14m/s时的PWM值
/***********开环时最大速度***********/
static const int16_t OpenLoopMaxSetSpd_A = 5700; //开环转速达到28m/s时的PWM值
static const int16_t OpenLoopMaxSetSpd_B = 5700; //开环转速达到28m/s时的PWM值
/***********闭环时最大速度***********/
static const int16_t MaxSetSpd_A =2780; //闭环转速达到28m/s时的设值
static const int16_t MaxSetSpd_B =2780; //闭环转速达到28m/s时的设值
/***********离线速度***********/
static const int16_t offline_setspeed_A =0;//1620; //闭环转速达到14m/s时的设值
static const int16_t offline_setspeed_B =0;//1620; //闭环转速达到14m/s时的设值
/*——————————————————————————————————————————————————1——————————————————————————————————————————————————————————————————————*/
static const float Friction_kp1 = 1.8f; 
static const float Friction_ki1 = 2; 
static const float Friction_kd1 = 0;
static const float Friction_iMin1 = 0;
static const float Friction_iMax1 = 500; 
static const float Friction_OutLimitMin1 = 0;
static const float Friction_OutLimitMax1 = 1000; 
//state.Standby_PWM
static const int16_t Standby_PWM1 = 4100;
/***********磁编码器离线时强制速度***********/
static const int16_t AS5048_Offline_setspeed_A1 = 4400;
static const int16_t AS5048_Offline_setspeed_B1 = 4400;
/***********强制开环时强制速度***********/
static const int16_t OpenLoopMaxSetSpd_A1 = 4800;
static const int16_t OpenLoopMaxSetSpd_B1 = 4800;
/***********闭环时最大速度***********/
static const int16_t MaxSetSpd_A1 =2780;
static const int16_t MaxSetSpd_B1 =2780;
/***********离线速度***********/
static const int16_t offline_setspeed_A1 =0;//1620;
static const int16_t offline_setspeed_B1 =0;//1620;
/*——————————————————————————————————————————————————2——————————————————————————————————————————————————————————————————————*/
static const float Friction_kp2 = 1.8f; 
static const float Friction_ki2 = 2;
static const float Friction_kd2 = 0;
static const float Friction_iMin2 = 0;
static const float Friction_iMax2 = 850;
static const float Friction_OutLimitMin2 = 0;
static const float Friction_OutLimitMax2 = 2000;
//state.Standby_PWM
static const int16_t Standby_PWM2 = 4100;
/***********磁编码器离线时强制速度***********/
static const int16_t AS5048_Offline_setspeed_A2 = 4500;
static const int16_t AS5048_Offline_setspeed_B2 = 4500;
/***********开环时强制速度***********/
static const int16_t OpenLoopMaxSetSpd_A2 = 5000;
static const int16_t OpenLoopMaxSetSpd_B2 = 5000;
/***********闭环时最大速度***********/
static const int16_t MaxSetSpd_A2 =2780;
static const int16_t MaxSetSpd_B2 =2780;
/***********离线速度***********/
static const int16_t offline_setspeed_A2 =0;//1620;
static const int16_t offline_setspeed_B2 =0;//1620;
/*——————————————————————————————————————————————————3——————————————————————————————————————————————————————————————————————*/
static const float Friction_kp3 = 1.8f;
static const float Friction_ki3 = 2;
static const float Friction_kd3 = 0;
static const float Friction_iMin3 = 0;
static const float Friction_iMax3 = 1000;
static const float Friction_OutLimitMin3 = 0;
static const float Friction_OutLimitMax3 = 2000;
//state.Standby_PWM
static const int16_t Standby_PWM3 = 4300; //1160
/***********磁编码器离线时强制速度***********/
static const int16_t AS5048_Offline_setspeed_A3 = 5000; //1630
static const int16_t AS5048_Offline_setspeed_B3 = 5000; //
/***********开环时强制速度***********/
static const int16_t OpenLoopMaxSetSpd_A3 = 5700;
static const int16_t OpenLoopMaxSetSpd_B3 = 5700;
/***********闭环时最大速度***********/
static const int16_t MaxSetSpd_A3 =2780;
static const int16_t MaxSetSpd_B3 =2780;
/***********离线速度***********/
static const int16_t offline_setspeed_A3 =0;//1620;
static const int16_t offline_setspeed_B3 =0;//1620;
/*——————————————————————————————————————————————————4——————————————————————————————————————————————————————————————————————*/
static const float Friction_kp4 = 1.8f; 
static const float Friction_ki4 = 2;
static const float Friction_kd4= 0;
static const float Friction_iMin4 = 0;
static const float Friction_iMax4 = 670;
static const float Friction_OutLimitMin4 = 0;
static const float Friction_OutLimitMax4 = 2000;
//4800 - state.Standby_PWM
static const int16_t Standby_PWM4 = 4300;
/***********磁编码器离线时强制速度***********/
static const int16_t AS5048_Offline_setspeed_A4 = 4940; //1620
static const int16_t AS5048_Offline_setspeed_B4 = 4960; //1620
/***********开环时最大速度***********/
static const int16_t OpenLoopMaxSetSpd_A4 = 5800; //2780
static const int16_t OpenLoopMaxSetSpd_B4 = 5825; //2780
/***********闭环时最大速度***********/
static const int16_t MaxSetSpd_A4 =2780;
static const int16_t MaxSetSpd_B4 =2780;
/***********离线速度***********/
static const int16_t offline_setspeed_A4 =0;//1620;
static const int16_t offline_setspeed_B4 =0;//1620;

static const u8 ForceOpenLoop = 0;						/*采集数据时更改，以脱离主控*/	
static const u8 ID = 3;
#endif

#ifndef LOADED
#error 未加载参数
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


/*摩擦轮启动流程：

开环（state.ForceOpenLoop 为 1）：直接从state.openloop_setspeed_x获取值并输出

闭环（state.ForceOpenLoop 为 0）：
	1.当SetSpd_x为0时，pwm固定为4000，为开环模式（暂时）
	2.当SetSpd_x不为0时
		①前（5*50）ms内，一边（A）摩擦轮输出跟随state.Standby_PWM = 4200，另一边（B）保持不变
		②（5*50）ms到（5*100）ms内，一边（B）摩擦轮输出跟随state.Standby_PWM = 4200，另一边（A）保持不变
		③而后切入闭环

摩擦轮异常处理
1.在转动过程中（PWM输出>4200）时，磁编码器1s内读数恒定，认为磁编码器离线，强制切为开环，占空比为state.AS5048_Offline_setspeed_x;

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

    /*如第一次正常启动，但之后闭环出现问题，禁用QuickStart*/
    state.EnableQuickStart = 0;   //第一次后快速启动

    state.StopState = 1;

    TDT_Snail_OpenLoopSwitch(1);

    state.OpenLoopMaxSetSpdLimit = 1;

    state.StartingTime = 0;
	/*以下为初始值无太大意义*/
	state.Standby_PWM = Standby_PWM; //待机时的占空比
    /******************开环时的速度*****************************/
    state.openloop_setspeed_A = 4000;
    state.openloop_setspeed_B = 4000;
	/******************强制开环时强制速度**********************/
    state.ForceOpenloop_setspeed_A = 4000; //强制开环设定值,测试用
    state.ForceOpenloop_setspeed_B = 4000; //强制开环设定值
	#ifdef FOUR_INFANTRY
	state.MaxSetSpd_A = MaxSetSpd_A; //最大闭环设定速度限幅
    state.MaxSetSpd_B = MaxSetSpd_B; //最大闭环设定速度限幅
	state.offline_setspeed_A=offline_setspeed_A; //离线闭环设定值
	state.offline_setspeed_B=offline_setspeed_B; //离线闭环设定值
	#endif
    state.OpenLoopMaxSetSpd_A = OpenLoopMaxSetSpd_A; //最大开环设定速度限幅
    state.OpenLoopMaxSetSpd_B = OpenLoopMaxSetSpd_B; //最大开环设定速度限幅
    /******************提升第一次启动速度(0到限幅)*************/
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
			state.Standby_PWM = Standby_PWM; //待机时的占空比
            state.MaxSetSpd_A = MaxSetSpd_A; //最大闭环设定速度限幅
            state.MaxSetSpd_B = MaxSetSpd_B; //最大闭环设定速度限幅
			state.offline_setspeed_A=offline_setspeed_A; //离线闭环设定值
	        state.offline_setspeed_B=offline_setspeed_B; //离线闭环设定值
            state.OpenLoopMaxSetSpd_A = OpenLoopMaxSetSpd_A;//最大开环设定速度限幅
            state.OpenLoopMaxSetSpd_B = OpenLoopMaxSetSpd_B;//最大开环设定速度限幅
			/******************磁编码器离线时强制速度******************/
            state.AS5048_Offline_setspeed_A = AS5048_Offline_setspeed_A;
            state.AS5048_Offline_setspeed_B = AS5048_Offline_setspeed_B;
            break;
        }
		case SHIP:
        {
			state.Standby_PWM = Standby_PWM1; //待机时的占空比
            state.MaxSetSpd_A = MaxSetSpd_A1; //最大闭环设定速度限幅
            state.MaxSetSpd_B = MaxSetSpd_B1; //最大闭环设定速度限幅
			state.offline_setspeed_A=offline_setspeed_A1; //离线闭环设定值
	        state.offline_setspeed_B=offline_setspeed_B1; //离线闭环设定值
            state.OpenLoopMaxSetSpd_A = OpenLoopMaxSetSpd_A1;//最大开环设定速度限幅
            state.OpenLoopMaxSetSpd_B = OpenLoopMaxSetSpd_B1;//最大开环设定速度限幅
			/******************磁编码器离线时强制速度******************/
            state.AS5048_Offline_setspeed_A = AS5048_Offline_setspeed_A1;
            state.AS5048_Offline_setspeed_B = AS5048_Offline_setspeed_B1;
            break;
        }
		case TWONETWONE:
        {
			state.Standby_PWM = Standby_PWM2; //待机时的占空比
            state.MaxSetSpd_A = MaxSetSpd_A2; //最大闭环设定速度限幅
            state.MaxSetSpd_B = MaxSetSpd_B2; //最大闭环设定速度限幅
			state.offline_setspeed_A=offline_setspeed_A2; //离线闭环设定值
	        state.offline_setspeed_B=offline_setspeed_B2; //离线闭环设定值
            state.OpenLoopMaxSetSpd_A = OpenLoopMaxSetSpd_A2;//最大开环设定速度限幅
            state.OpenLoopMaxSetSpd_B = OpenLoopMaxSetSpd_B2;//最大开环设定速度限幅
			/******************磁编码器离线时强制速度******************/
            state.AS5048_Offline_setspeed_A = AS5048_Offline_setspeed_A2;
            state.AS5048_Offline_setspeed_B = AS5048_Offline_setspeed_B2;
            break;
        }
		case AURORA:
        {
			state.Standby_PWM = Standby_PWM3; //待机时的占空比
            state.MaxSetSpd_A = MaxSetSpd_A3; //最大闭环设定速度限幅
            state.MaxSetSpd_B = MaxSetSpd_B3; //最大闭环设定速度限幅
			state.offline_setspeed_A=offline_setspeed_A3; //离线闭环设定值
	        state.offline_setspeed_B=offline_setspeed_B3; //离线闭环设定值
            state.OpenLoopMaxSetSpd_A = OpenLoopMaxSetSpd_A3;//最大开环设定速度限幅
            state.OpenLoopMaxSetSpd_B = OpenLoopMaxSetSpd_B3;//最大开环设定速度限幅
			/******************磁编码器离线时强制速度******************/
            state.AS5048_Offline_setspeed_A = AS5048_Offline_setspeed_A3;
            state.AS5048_Offline_setspeed_B = AS5048_Offline_setspeed_B3;
            break;
        }
		case GALAXY:
        {
			state.Standby_PWM = Standby_PWM4; //待机时的占空比
            state.MaxSetSpd_A = MaxSetSpd_A4; //最大闭环设定速度限幅
            state.MaxSetSpd_B = MaxSetSpd_B4; //最大闭环设定速度限幅
			state.offline_setspeed_A=offline_setspeed_A4; //离线闭环设定值
	        state.offline_setspeed_B=offline_setspeed_B4; //离线闭环设定值
            state.OpenLoopMaxSetSpd_A = OpenLoopMaxSetSpd_A4;//最大开环设定速度限幅
            state.OpenLoopMaxSetSpd_B = OpenLoopMaxSetSpd_B4;//最大开环设定速度限幅
			/******************磁编码器离线时强制速度******************/
            state.AS5048_Offline_setspeed_A = AS5048_Offline_setspeed_A4;
            state.AS5048_Offline_setspeed_B = AS5048_Offline_setspeed_B4;
            break;
        }
        default:
            break;
    }
	#endif
}


