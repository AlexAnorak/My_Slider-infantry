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
private:
    int16_t getMotorSpeed();
    float remoteSensitivity = 1;   //遥控器灵敏度
    float keyboardSensitivity = 1; //wasd灵敏度

public:
    /**
 * @brief Construct a new Chassis object
 */
    Chassis();
    vec3f remoteCtrl();
    vec3f keyboardCtrl();
    vec3f customCtrl();
    /**
 * @brief 速度合成
 *
 */
    void speedCompute();
    /**
 * @brief 云台坐标转换
 *
 * @param speedIn
 * @return vec3f
 */
    vec3f remoteSpeed, keyboardSpeed, followSpeed, rotateSpeed, swingSpeed, customSpeedIn, customSpeedOut, allSpeed;
    /**
 * @brief 速度输出
 *
 */
    void motorOutput();
    bool lockChassis = 0;     //是否锁底盘
    bool customFlag = 0;      //自定义flag
    bool rotateFlag = 0;      //小陀螺
    bool flexRotate = 0;      //是否变速陀螺
    bool swingFlag = 0;       //摇摆
    bool ifChassisFollow = 1; //是否底盘跟随，默认跟随
    bool ifTransform = 1;     //是否坐标转换，默认转换
    bool flySloping = 0;
    struct power_Param //功率计算参数
    {
        u8 PowerPath_Switch = 1;
        u8 checkMode = 0;
        u8 ULTSMode = 0;
    } powerParam;
    float chassisFollowAngle = 0; //底盘跟随角

	bool judgeIfMoving();
	
    void init();
    void run();
};
extern Chassis chassis;

#endif
