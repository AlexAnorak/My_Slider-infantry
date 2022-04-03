/*
 * @Author: your name
 * @Date: 2021-05-08 23:52:16
 * @LastEditTime: 2021-05-09 05:49:08
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Projectd:\TDT\TDT-Infantry\Infantry_II\TDT_Task\src\chassis_task.cpp
 */

//todo add rotate direct
#include "chassis_task.h"
#include "dbus.h"
#include "my_math.h"
#include "imu_task.h"
#include "gimbal_task.h"
#include "can.h"
#include "vision.h"
#include "state_task.h"
#include "my_math.h"
#include "filter.h"
#include "judgement.h"


#define yawLargeErr 1
#define yawSmallErr 2

float yawLargeErrThreshold=20.0/360*8196;

Chassis chassis;

Chassis::Chassis() : motorWheel{Motor(M3508, CAN2, 0x201), Motor(Motor(M3508, CAN2, 0x202))}, motorSlider{Motor(GM6020, CAN2, 0x205), Motor(GM6020, CAN2, 0x206)}
{
}

ChassisRecv chassisRecv;

extern float YawStick_Sensitivity; //灵敏度
extern float YawMouse_Sensitivity;

float speedAfterFilter = 0;
Kf speedFilter;
float speedFilterQ = 0.001, speedFilterR = 2;


void Chassis::getMaxPower()
{
	powerParam.powerMaxForKp = 1;
}


void Chassis::selectDownLimitRange()
{
    if (flySloping)
    {
        powerParam.powerDownLimit = 30; //下限30，上限50
        powerParam.powerLimitRange = 20;
        return;
    }
    if (judgeShift.doubleShift != 0)
    {
        //使用备用功率
        if (powerParam.usingBackup == 1)
        {
            powerParam.powerDownLimit = 30; //下限30，上限50
            powerParam.powerLimitRange = 20;
            return;
        }
        //急速模式
        if (judgeShift.doubleShift == 2)
        {
            powerParam.powerDownLimit = 50; //下限50，上限60
            powerParam.powerLimitRange = 10;
            return;
        }
        //普通模式,仅按下shift后不使用备用功率后的第一次判断
        if (!powerParam.oldShiftDown || powerParam.lastUsingBackup == 1)
        {
            powerParam.powerDownLimit = MAX(50, powerParam.remainPower_P - 10);
            powerParam.powerLimitRange = 10;
            powerParam.lastUsingBackup = 0;
        }
        return;
    }
    if (powerParam.oldShiftDown)
        powerParam.powerDownLimit = LIMIT(powerParam.remainPower_P - 3, 50, 90); //当前百分比减3，作为下限

    if (powerParam.remainPower_P >= 55) //大于55，不使用备用功率
    {
        if (powerParam.usingBackup)
        {
            powerParam.powerDownLimit = powerParam.remainPower_P - 3; //如果之前使用了备用功率，下限设为当前百分比减三
        }
        powerParam.usingBackup = 0;
    }
    if (powerParam.remainPower_P >= 93) //大于93则下限90上限100
    {
        powerParam.powerDownLimit = 90;
        powerParam.powerLimitRange = 10;
    }

    if (powerParam.usingBackup) //使用备用功率
    {
        powerParam.powerDownLimit = 30;
        powerParam.powerLimitRange = 20;
    }
    else //不使用备用功率即当前百分比大于55时
    {
        //动态获取下限
        powerParam.powerDownLimit = LIMIT(powerParam.remainPower_P - 10, powerParam.powerDownLimit, 90);
        powerParam.powerLimitRange = 10;
    }
}

void Chassis::powerCtrlOld(void)
{
    int Num = 0;

    /*获得当前底盘所需参数*/
    powerOfflineCheck();
    getJgmtMsg();
    getSuperPowerMsg();
    getMaxPower();
    powerParam.oldShiftDown = powerParam.shiftDown;
    powerParam.shiftDown = RC.chassisRecv.keyShift || RC.chassisRecv.chassisMode == 5;
    /*判断当前加速模式*/
    judgeShiftMode();
    if (!can1Feedback.jgmtOffline && (powerParam.powerOffline == 1 || powerParam.PowerPath_Switch == 0))
    {
        powerParam.powerLimitKp = powerParam.overFlowKp * LIMIT(((float)(powerParam.RemainPowerBuffer - 10.0f) / 50.0f), 0.0f, 1.0f); //软硬件混合限功率
        return;
    }
    /*计算限幅系数*/
    if (powerParam.HardOnly == 0)
    {
        selectDownLimitRange();
        powerParam.powerLimitKp = LIMIT((powerParam.remainPower_P - powerParam.powerDownLimit) / powerParam.powerLimitRange, 0, 1.0f); //百分之50左右
    }
    else //纯硬件
    {
        if (powerParam.powerOffline == 0) //在线
        {
            if (powerParam.usingBackup == 1)
                powerParam.powerLimitKp = powerParam.overFlowKp * LIMIT(((float)(powerParam.remainPower_P - 20.0f) / 30.0f), 0.3f, 1.0f); //百分之50左右使用会突然冲刺，但是会跑不动
            else
            {
                if (powerParam.ShiftOnly == 1)
                {
                    if (powerParam.shiftDown == 0 && deforceFlag == 0)
                    {
                        for (int a = 100 / powerParam.LimitDivPerLv; a > 0; a--)
                        {
                            if (LIMIT(((float)(powerParam.remainPower_P - powerParam.LimitDivPerLv * a) / powerParam.LimitDivPerLv), 0.2f, 1.0f) == 1.0f && (a >= powerParam.LimitLv || powerParam.LvLimitEnable == 0))
                            {
                                powerParam.LimitLv = a;
                                break;
                            }
                        }
                        powerParam.LvLimitEnable = 1;
                        powerParam.powerLimitKp = LIMIT(((float)(powerParam.remainPower_P - powerParam.LimitDivPerLv * powerParam.LimitLv) / powerParam.LimitDivPerLv), 0.2f, 1.0f);
                    }
                    else
                    {
                        powerParam.LvLimitEnable = 0;
                        powerParam.powerLimitKp = 1.0;
                    }
                    powerParam.powerLimitKp = MIN(powerParam.powerLimitKp, powerParam.overFlowKp * LIMIT(((float)(powerParam.remainPower_P - 40.0f) / 40.0f), 0.15f, 1.0f)); //保留备用功率
                }
                else
                {
                    powerParam.powerLimitKp = LIMIT(((float)(powerParam.remainPower_P - 40.0f) / 40.0f), 0.3f, 1.0f); //保留备用功率
                }
            }
            powerParam.powerLimitKp = LIMIT(((float)(powerParam.remainPower_P - 40.0f) / 40.0f), 0.3f, 1.0f);
        }
        else //离线
        {
            powerParam.powerLimitKp *= LIMIT(((float)(powerParam.RemainPowerBuffer - 20.0f) / 40.0f), 0.0f, 1.0f);
        }
    }
}

u8 Chassis::judgeShiftMode()
{
    if (flySloping)
    {
        judgeShift.doubleShift = 2;
        return 2;
    }
    if (powerParam.shiftDown)
    {
        if (!powerParam.oldShiftDown)
        {
            judgeShift.shiftFromTimeFirst = judgeShift.shiftFromTimeNext;
            judgeShift.shiftFromTimeNext = getSysTimeUs() / 1e6f;
            if (timeIntervalFrom_f(judgeShift.shiftFromTimeFirst) < 0.25f)
            {
                judgeShift.doubleShift = 2;
            }
            else
            {
                judgeShift.doubleShift = 1;
            }
        }
    }
    else if (powerParam.oldShiftDown)
    {
        judgeShift.doubleShift = 0;
    }
    return judgeShift.doubleShift;
}

float angleOverRound(float angle, float range)
{
    if (angle > range / 2)
    {
        angle -= 8192;
    }
    else if (angle < -range / 2)
    {
        angle += 8192;
    }
    return angle;
}

void Chassis::speedCompute()
{ 
    originSpeed.data[0] = RC.chassisRecv.speedWS;
    originSpeed.data[1] = RC.chassisRecv.speedAD;
	if(ABS(originSpeed.data[0]) < 400)
		originSpeed.data[0] = 0;
	if(ABS(originSpeed.data[1]) < 1000)
		originSpeed.data[1] = 0;
	
    originSpeed.data[2] = RC.chassisRecv.speedYaw;

    yawAngleCalibration = RC.chassisRecv.zeroYaw;

    yawAngleCalibration = angleOverRound(yawAngleCalibration, 8192) * 360.0 / 8192;

    //WARN 可能有符号
    if (originSpeed.data[1] != 0 || originSpeed.data[0] != 0)
        originSpeedAngle = fast_atan2(originSpeed.data[1], originSpeed.data[0])/RAD_PER_DEG;

    finalSpeedAngle = originSpeedAngle - yawAngleCalibration;
	
	if(finalSpeedAngle - yawAngle/8192.0*360 >= 90)
	{
		finalSpeedAngle-=180;
	}
	if(finalSpeedAngle - yawAngle/8192.0*360 <= -90)
	{
		finalSpeedAngle+=180;
	}	
//	ABS(finalSpeedAngle)>=90?finalSpeedAngle=angleOverRound(finalSpeedAngle-180,360):1;

    vec2f sc_yawAngleCalibration{(float)my_sin(yawAngle/8192.0*MY_PPPIII), (float)my_cos(yawAngle/8192.0*2*MY_PPPIII)};

    finalSpeed.data[0] = speedAmp * (originSpeed.data[0] * sc_yawAngleCalibration.data[1] + originSpeed.data[1] * sc_yawAngleCalibration.data[0]);

    finalSpeed.data[1] = originSpeed.data[2];
	
}

float underVoltageProtectionKp = 1;

float Chassis::underVoltageProtection()
{
    if (!powerParam.powerOffline)
    {
        underVoltageProtectionKp = LIMIT((powerParam.superPower_V * 15 / 26 - powerParam.SuperPower_RealPower), 0, 1);
        underVoltageProtectionKp *= LIMIT((15 - powerParam.SuperPower_RealPower) / 3, 0, 1);
        return underVoltageProtectionKp;
    }
    return 1;
}

void Chassis::caliSliderOffset()		//滑块复位
{
    u8 sliderOKCnt = 0;
    for (int i = 0; i < 2; i++)
    {
        if (motorSliderOffset[i])
        {
            sliderOKCnt++;
            motorSlider[i].ctrlPosition(sliderZero[i]);
            continue;
        }
        motorSliderOffset[i] =
            motorSlider[i].ctrlMotorOffset(offsetSpeed, offsetmaxerro, offsetlimit);
    }
    if (sliderOKCnt == 2)
    {
        sliderOK = 1;
        return;
    }
    lastdeforceFlag = true;
    lastDeforceTime = getSysTimeUs();
    return;
}

void Chassis::feedbackUpdate()
{
    pitchAngleFb = boardImu->Angle.pitch;                                //底盘直立反馈值
    speedFb = motorWheel[0].canInfo.speed - motorWheel[1].canInfo.speed; //底盘速度反馈值
    speedAfterFilter = speedFilter.KalmanFilter(speedFb, speedFilterQ, speedFilterR);
    speedDLFb = speedAfterFilter;
    yawFb = yawAngle;   //底盘跟随反馈值
	yawSpeedFb = boardImu->gyro.dps.data[2];
    pitchDLAngleFb = pitchAngleFb; //滑块直立反馈值
}

void Chassis::calcOutput()
{
    pitchOut = pidAngle.Calculate(pitchZero);          //底盘直立输出
    speedOut = pidSpeed.Calculate(finalSpeed.data[0]); //底盘速度输出
    speedDLOut = pidDLSpeed.Calculate(finalSpeed.data[0]);

    //todo 测试yaw双环

    if (RC.chassisRecv.chassisMode != 1)
    {
        rotateParam.timeRecord = 0;
        rotateParam.sinResult = 0;
    }
    if (RC.chassisRecv.chassisMode != 2)
    {
        swingParam.timeRecord = 0;
        swingParam.sinResult = 0;
    }

    switch (RC.chassisRecv.chassisMode)
    {
    case 0://正常
        //WARN 正负号
        yawSpeedSet = finalSpeed.data[1] * followPreFbAmp; //底盘跟随输出
        yawOut = pidYawSpeed.Calculate(yawSpeedSet);
		if(ABS(pidYaw.error) >= yawLargeErrThreshold)
			yawOut += pidYaw.Calculate(finalSpeedAngle/360.0*8192,yawLargeErr);
		else
			yawOut += pidYaw.Calculate(finalSpeedAngle/360.0*8192,yawSmallErr);
        break;
    case 1://陀螺
		rotateParam.timeRecord++;
		rotateParam.sinResult = rotateParam.sinMax * sinf(rotateParam.sinOmega * rotateParam.timeRecord) + 0.63;
        yawOut = MAX_CHASSIS_VW_SPEED * rotateParam.sinResult;
        // *(1 + (ABS(keyboardSpeed.data[0]) + ABS(keyboardSpeed.data[1])) * 0.000005);

        if (RC.chassisRecv.keyShift)
            yawOut *= 1.2;

        break;
    case 2://摇摆
        swingParam.timeRecord++;
        //todo 添加均值参数
        swingParam.sinResult = swingParam.sinMax * sinf(swingParam.sinOmiga * swingParam.timeRecord) + (MY_PPPIII_HALF / 2);
        yawOut = pidYaw.Calculate(finalSpeedAngle + swingParam.sinResult);
        if (RC.chassisRecv.keyShift)
            yawOut *= 1.2;
        break;
    case 3://不跟随云台
        yawOut = 0;
        break;
	case 4:
		rotateParam.timeRecord++;
        rotateParam.sinResult = rotateParam.sinMax * sinf(rotateParam.sinOmega * rotateParam.timeRecord) + 0.63;
        yawOut = MAX_CHASSIS_VW_SPEED * rotateParam.sinResult;
        // *(1 + (ABS(keyboardSpeed.data[0]) + ABS(keyboardSpeed.data[1])) * 0.000005);

        if (RC.chassisRecv.keyShift)
            yawOut *= 1.2;
break;
	case 5://飞坡
		break;
	case 6://自定义
		break;
	default:break;
    }

    yawOut = LIMIT(yawOut, -pidYawPara[yawLargeErr].resultMax, pidYawPara[yawLargeErr].resultMax);
}

void Chassis::motorOutput()
{
    powerCtrlOld();
    powerParam.powerLimitKp *= underVoltageProtection();

    while (chassis.pitchAngleFb > 180.0f)
    {
        chassis.pitchAngleFb -= 360.0f;
    }
    while (chassis.pitchAngleFb < -180.0f)
    {
        chassis.pitchAngleFb += 360.0f;
    }

    if (!sliderOK)
    {
        caliSliderOffset();
        return;
    }

    feedbackUpdate();

    calcOutput();

#ifdef NOT_USE_POWER_MODULE
    powerParam.powerLimitKp = 1;
#else
    powerCtrlOld();
    powerParam.powerLimitKp *= underVoltageProtection();
#endif

    if (pidAngle.error > angleMaxthreshold)
        angleMaxCnt++;
    if (angleMaxCnt > 200)
    {
        angleMaxCnt = 201; //锁定angleMaxCnt

        float directAngle = pidAngle.error / ABS(pidAngle.error);

        for (int i = 0, directPitch = -1; i < 2; i++, directPitch += 2)
        {
            motorWheel[i].ctrlSpeed(0); //刹车

            motorSlider[i].ctrlPosition(sliderZero[i] + LIMIT(+directPitch * directAngle * 6000, -5000, 5000));
        }

        if (ABS(speedAfterFilter) > 1000)
            return;
        angleMaxCnt = 0;
    }

    for (int i = 0, directPitch = -1; i < 2; i++, directPitch += 2)
    {
        if (ABS(pidSpeed.error) < speedErrorThresold || RC.chassisRecv.keyCtrl)
            speedFinalOut = 0;
        else
            speedFinalOut = -(4500-pidSpeed.error / ABS(pidSpeed.error)*(2500)+7000*(pidSpeed.setValue>=0 && pidSpeed.fbValue<=0)) * pidSpeed.error / ABS(pidSpeed.error) * 
				(speedPreAmp + speedResultAmp * (timeIntervalFrom(lastDeforceTime) > 1000000
				&& ABS(pidSpeed.result) > speedResultThreshold && ABS(pidSpeed.error) > speedErrorThresold
				&& ABS(pidYaw.error) < yawErrorThreshold));

        //输出
        totalWheelCurrent[i] = directPitch * (pitchOut + speedFinalOut) + yawOut;
        totalWheelCurrent[i] += directPitch * (SGN(totalWheelCurrent[i]) * currentAddition);

        motorWheel[i].ctrlCurrent(totalWheelCurrent[i]);
        motorSlider[i].ctrlPosition(sliderZero[i] + LIMIT(directPitch * speedDLOut, -5000, 5000));
    }
}
void Chassis::powerOfflineCheck()
{
    can1Feedback.SuperPowerOfflineCheck++;
    if (can1Feedback.SuperPowerOfflineCheck > 100)
    {
        can1Feedback.SuperPowerOffline = 1;
    }
    else
    {
        can1Feedback.SuperPowerOffline = 0;
    }
}
void Chassis::getJgmtMsg()
{
    jgmtParam.jgmtOffline = judgement.jgmtOffline;
    jgmtParam.chassisBuffer = judgement.powerHeatData.chassisPowerBuffer;
}
void Chassis::getSuperPowerMsg()
{
    powerParam.powerOffline = can1Feedback.SuperPowerOffline;
	#if USE_OLD_POWER_MODULE
    powerParam.remainPower_P = RecvStruct.capacitance_percentage;
    powerParam.superPower_V = RecvStruct.voltage;
    powerParam.SuperPower_RealPower = RecvStruct.local_Power;
	#else
    powerParam.remainPower_P = RecvStruct.capacitance_percentage;
    powerParam.superPower_V = RecvStruct.voltage;
    powerParam.SuperPower_RealPower = RecvStruct.local_Power;
	#endif
    powerParam.RemainPowerBuffer = judgement.powerHeatData.chassisPowerBuffer;
}
void Chassis::powerOverFlowCal()
{
    static u8 checkTime;
    if (checkTime++ > 100)
    {
        powerParam.overLimitCase = 1;
        checkTime = 0;
    }
    if (powerParam.overLimitCase == 1 && judgement.powerHeatData.chassisPowerBuffer <= 0)
    {
        powerParam.overLimitCase = 0;
        powerParam.overFlowKp -= 0.03f;
        powerParam.overFlowKp = LIMIT(powerParam.overFlowKp, 0.7f, 1.0f);
    }
}

void Chassis::run()
{
    /* code */
    if (deforceFlag)
    {
        lastdeforceFlag = true;
        return;
    }
    if (lastdeforceFlag)
        lastDeforceTime = getSysTimeUs();

    speedCompute();
    motorOutput();
    lastdeforceFlag = deforceFlag;
}

#include "parameter.h"

void Chassis::init()
{
    // switch(STANDARD_ID)
    // {
    // case NIRVANA:

    //todo 确定参数
    wheelStopPara.kp = 5;
    wheelStopPara.resultMax = motorWheel[0].getMotorCurrentLimit();

    pidInner.kp = 3;
    pidInner.ki = 5;
    pidInner.integralErrorMax = 0.1;
    pidInner.kd = 0;
    pidInner.resultMax = 15000;
    pidOuter.kp = 10;
    pidOuter.ki = 10;
    pidOuter.integralErrorMax = 0.1;
    pidOuter.kd = 0;
    pidOuter.resultMax = 1000;

    for (int i = 0; i < 2; i++)
    {
        motorSlider[i].pidInner.paramPtr = &pidInner;
        motorSlider[i].pidOuter.paramPtr = &pidOuter;
        motorSlider[i].pidInner.setPlanNum(1);
        motorSlider[i].pidOuter.setPlanNum(1);
        motorSlider[i].pidInner.fbValuePtr[0] = &motorSlider[i].canInfo.speed;
        motorSlider[i].pidOuter.fbValuePtr[0] = &motorSlider[i].canInfo.totalEncoder;
    }

    pidDLSpeed.setPlanNum(1); //滑块速度PID
    pidDLSpeed.paramPtr = &pidDLSpeedPara;
    pidDLSpeed.fbValuePtr[0] = &speedAfterFilter;
    pidDLSpeed.fbValuePtr[0] *= 1;
    pidDLSpeedPara.kp = 2;
    pidDLSpeedPara.ki = 3;
    pidDLSpeedPara.integralErrorMax = 1000;
    pidDLSpeedPara.resultMax = 30000;
    pidDLSpeedPara.integralMethod = 1;
    pidDLSpeedPara.integralThreshold = 800;
    pidDLSpeedPara.kiAmpForIntegralSep = 1000;

    pidSpeed.setPlanNum(1); //底盘速度PID
    pidSpeed.paramPtr = &pidSpeedPara;
    pidSpeed.fbValuePtr[0] = &speedAfterFilter;
    pidSpeed.fbValuePtr[0] *= 1;
    pidSpeedPara.kp = 0.1;
    pidSpeedPara.ki = 10;
    pidSpeedPara.integralErrorMax = 800;
    pidSpeedPara.resultMax = 8000;
    pidSpeedPara.integralMethod = 0;
    //	pidSpeedPara.kp = 1;
    //	pidSpeedPara.ki = 1;
    //	pidSpeedPara.integralErrorMax = 2000;
    //	pidSpeedPara.resultMax = 5000;
    //	pidSpeedPara.integralMethod = 0;
    //	pidSpeedPara.integralThreshold = 0;
    //	pidSpeedPara.kiAmpForIntegralSep = 0;

    pidYaw.setPlanNum(3); //底盘跟随PID
    pidYaw.fbValuePtr[0] = &yawFb;
    pidYaw.fbValuePtr[1] = &yawFb;
	pidYaw.fbValuePtr[2] = &yawFb;
	pidYaw.paramPtr = pidYawPara;
	
	pidYawPara[0].kp = 100;
    pidYawPara[0].ki = 0;    //-50;
    pidYawPara[0].kd = 0; //-50;
    pidYawPara[0].integralErrorMax = 100;
    pidYawPara[0].resultMax = 5000;
    
    pidYawPara[yawLargeErr].kp = 400;
    pidYawPara[yawLargeErr].ki = 0;    //-50;
    pidYawPara[yawLargeErr].kd = 75; //-50;
    pidYawPara[yawLargeErr].integralErrorMax = 100;
    pidYawPara[yawLargeErr].resultMax = 12000;   //20000;
	
	pidYawPara[yawSmallErr].kp = 100;
    pidYawPara[yawSmallErr].ki = 0;    //-50;
    pidYawPara[yawSmallErr].kd = 300; //-50;
	pidYawPara[yawSmallErr].differentialMethod = 1;
    pidYawPara[yawSmallErr].integralErrorMax = 100;
    pidYawPara[yawSmallErr].resultMax = 5000;
	
	pidYawSpeed.setPlanNum(1);//串级底盘跟随前馈速度PID
	pidYawSpeed.paramPtr = &pidYawSpeedPara;
	pidYawSpeed.fbValuePtr[0] = &yawSpeedFb;
	pidYawSpeed.fbValuePtr[0]*=-1;
	pidYawSpeedPara.kp = 100;
	pidYawSpeedPara.ki = 0;
	pidYawSpeedPara.kd = 0;
	pidYawSpeedPara.integralErrorMax = 100;
	pidYawSpeedPara.resultMax = 10000;

    pidAngle.setPlanNum(1); //底盘直立PID
    pidAngle.paramPtr = &pidAnglePara;
    pidAngle.fbValuePtr[0] = &pitchAngleFb;
    pidAnglePara.kp = 1500;
	pidAnglePara.ki = 0;  //1;
    pidAnglePara.kd = 100;
	pidAnglePara.integralErrorMax = 10;
    pidAnglePara.resultMax = 30000;
    pidAnglePara.positiveFBFlag = 1;
	pidAnglePara.differentialMethod = 1;

    pidDLAngle.setPlanNum(1); //滑块直立PID
    pidDLAngle.paramPtr = &pidDLAnglePara;
    pidDLAngle.fbValuePtr[0] = &pitchDLAngleFb;
    pidDLAnglePara.kp = 0; //800;
    pidDLAnglePara.kd = 0;
    pidDLAnglePara.resultMax = 5000;
    // break;
    // }
}