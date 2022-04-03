/*
 * @Author: your name
 * @Date: 2021-05-08 23:52:16
 * @LastEditTime: 2021-05-09 05:38:36
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Projectd:\TDT\TDT-Infantry\Infantry_II\TDT_Task\inc\chassis_task.h
 */
#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "board.h"
#include "mecanum.h"
#include "motor.h"
#include "pid.h"
struct Chassis
{
public:
    Motor motorWheel[2];
    PidParam wheelStopPara;
    Motor motorSlider[2];
    PidParam pidInner, pidOuter;

    struct swing_Param //摇摆计算参数
    {
        /* data */
        int timeRecord=0;
        float sinOmiga = 0.031415 * 1.3; //y = Asin(wt)  w = 2*3.14/T
        float sinMax = (MY_PPPIII / 2);
        float sinResult=0;
    } swingParam;

    struct rotate_Param //小陀螺计算参数
    {
        /* data */
        int timeRecord=0;
        float sinOmega = 0.031415f;
        float sinMax = 0.35f;
        float sinResult=0;
    } rotateParam;

    Pid pidAngle; //底盘直立PID
    float pitchAngleFb;
    float pitchOut;
    PidParam pidAnglePara;

    Pid pidDLAngle; //滑块直立PID
    float pitchDLAngleFb;
    float pitchDLOut;
    PidParam pidDLAnglePara;

    Pid pidSpeed; //底盘速度PID
    float speedFb;
    float speedOut;
    PidParam pidSpeedPara;

    Pid pidDLSpeed; //滑块速度PID
    float speedDLFb;
    float speedDLOut;
    PidParam pidDLSpeedPara;

    Pid pidYaw; //底盘跟随PID
    float yawFb;
    float yawOut;
    PidParam pidYawPara[3];
	
	Pid pidYawSpeed; //串级底盘跟随前馈速度PID
    float yawSpeedSet;
    float yawSpeedFb;
    float yawSpeedOut;
	PidParam pidYawSpeedPara;

    uint16_t angleMaxCnt = 0;
    float angleMaxthreshold = 20;

    float speedAfterFilter = 0;
    float currentAddition = 250;  //350;
    float speedResultAmp = -2.5;
    float speedErrorThresold = 2300; //7999;
    float speedResultThreshold = 0;   //7999;
    float yawErrorThreshold = 2000;
    float speedPreAmp = 0;
    float speedFinalOut = 0;

    float totalWheelCurrent[2];

    vec3f originSpeed;
    vec2f finalSpeed;
    float originSpeedAngle = 0;
    float finalSpeedAngle = 0;
    float yawAngle;            //云台角度原始值
	float yawZeroAngle = 4118;
    float yawAngleCalibration; //云台正方向角度
    float yawfinalSpeedAngle;  //沿着速度方向

    float followPreFbAmp = 1;
    float pitchZero = -6;   //-5.5; //底盘平衡零点360
    float speedAmp = -1 / 300.0f * 19 * 60;

    u8 lastdeforceFlag;
    uint64_t lastDeforceTime;

    u8 motorSliderOffset[2] = {0};
    float offsetSpeed = -50;            //滑块校零增量
    float offsetmaxerro = 40000;        // 滑块校零最大偏差
    float offsetlimit = 5000;           //滑块校零限幅
    bool sliderOK;                      //滑块校零完成
    float sliderZero[2] = {5400, 5400}; //滑块零点

    Chassis();
    void init();
    void run();

    struct power_Param //功率计算参数
    {
        /* data */
        bool usingJgmt;
        bool usingBackup = 0;
        bool lastUsingBackup = 0;
        bool shiftDown;
        bool oldShiftDown;
        bool powerOffline = 0;
        float powerLimitKp = 1;
        float remainPower_P;
        float superPower_V;
        float SuperPower_RealPower;
        float powerLimit;
        float overFlowKp = 1;
        float stepLess_P;
        bool PowerPath_Switch = 1;
        float jgmtOfflineKp = 0.5;
        float RemainPowerBuffer;
        bool HardOnly = 0;
        bool ShiftOnly = 1;
        bool checkMode = 0;
        bool ULTSMode = 0;
        bool overLimitCase = 1;
        u8 LimitDivPerLv;
        u8 LimitLv;
        u8 LvLimitEnable;
        float powerMaxForKp;
        float powerDownLimit;
        float powerLimitRange;
        float backupThresold = 60;
    } powerParam;
    struct jgmt_Param //裁判系统需要的数据
    {
        bool jgmtOffline;
        float chassisBuffer;
    } jgmtParam;

    struct JudgeShift
    {
        float shiftFromTimeFirst;
        float shiftFromTimeNext;
        int timeRecode;
        u8 doubleShift = 0;
    } judgeShift;

    void selectDownLimitRange();
    void powerCtrl();
    void powerCtrlOld(void);
    float underVoltageProtection();
    void powerOfflineCheck();
    void powerOverFlowCal();
    void getJgmtMsg();
    void getSuperPowerMsg();
    void getMaxPower();
    bool judgeIfMoving();
    u8 judgeShiftMode();

    void speedCompute();
    void caliSliderOffset();
    void feedbackUpdate();
    void calcOutput();
    void motorOutput();

    u8 flySloping = false;
};
extern Chassis chassis;

#endif
