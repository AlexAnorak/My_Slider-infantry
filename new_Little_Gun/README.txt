安全摩擦轮模块

一、摩擦轮启动流程：
开环（state.ForceOpenLoop 为 1）：直接从state.openloop_setspeed_x获取值并输出
闭环（state.ForceOpenLoop 为 0）：
	1.当SetSpd_x为0时，pwm固定为4000，为开环模式
	2.当SetSpd_x不为0时
		①前（5*100）ms内，一边（A）摩擦轮输出跟随state.Standby_PWM = 4200，另一边（B）保持不变
		②（5*100）ms到（5*200）ms内，一边（B）摩擦轮输出跟随state.Standby_PWM = 4200，另一边（A）保持不变
		③（5*200）ms到（5*400）ms内，都保持不变
		④而后切入闭环

二、摩擦轮异常处理
1.在转动过程中（PWM输出>4200）时，AS5048X.speed_ef读数为0，认为磁编码器离线，强制切为开环，占空比为state.AS5048_Offline_setspeed_x
2.离线可通过看LED灯判断(确定离线后哪一边闪就哪一边离线)
3.如果上电之后摩擦轮只响了4下，并且开环无法转，增大pwm.c中
	delay_ms(2300);//2000//1500//如果上电之后摩擦轮只响了4下，并且开环无法转，增大这一行的延时秒数，并反馈
一行的延时秒数，并反馈

三、当改变电调时需要将电机的变量重新采集

四、添加新电机流程

-1.按此格式添加新电机的变量

 	  //#define MOTORNAME//电机名字
    	#ifdef MOTORNAME
		#ifdef LOAD
		#error 参数加载不明确
		#endif
		#define LOADED
		
		static const float Friction_kp=0.6f;					//比例系数，影响电机转速曲线
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
		
		static const u8 ForceOpenLoop = 0;						//采集数据时更改，需脱离主控
	#endif

0.**焊好磁编码器并安装好**
1.修改 ForceOpenLoop 为 1
2.修改 ForceOpenloop_setspeed_A 和 ForceOpenloop_setspeed_B 为4000
3.编译、进debug
4.**WATCH窗口**中 state.OpenLoopMaxSetSpdLimit 置0
5.读磁编码器曲线，当达到*1000*左右时将此时的pwm填入 Standby_PWM
6.读磁编码器曲线，当达到*3000*左右时将此时的pwm*减去* Standby_PWM 后填入 Friction_OutLimitMax
7.读磁编码器曲线，当达到*2800*左右时将此值填入 AS5048_Offline_setspeed_A 和 AS5048_Offline_setspeed_B 以及 ForceOpenloop_setspeed_A 和 ForceOpenloop_setspeed_B 
8.调pid

五、油门行程校准（仅针对银燕电调）
0.**断电**
1.pwm.c中去掉以下注释
//#define THROTTLE_TRAVEL_CALIORATION
2.**编译下载，拔JLINK，保证板子没电**
3.打开电源开关，等待板子上的灯不再闪烁，**并且**持续5秒电机不发出声音
4.**断电**，pwm.c中注释#define THROTTLE_TRAVEL_CALIORATION
5.**编译下载**

六、英雄大炮主要修改speed_ef与子弹出膛速度大致相等以及发送