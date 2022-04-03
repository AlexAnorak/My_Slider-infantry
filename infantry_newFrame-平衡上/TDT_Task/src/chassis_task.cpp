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
#include "icm20602.h"
#include "gimbal_task.h"
#include "can.h"
#include "state_task.h"
Chassis chassis;
Chassis::Chassis()
{
}

extern float YawStick_Sensitivity; //灵敏度
extern float YawMouse_Sensitivity;

vec3f Chassis::remoteCtrl()
{
    vec3f speedOut = {0};
    speedOut.data[0] = float(RC.Key.CH[3] / 660.0f) * MAX_CHASSIS_VX_SPEED * remoteSensitivity;
    speedOut.data[1] = float(RC.Key.CH[2] / 660.0f) * MAX_CHASSIS_VY_SPEED * remoteSensitivity;
    speedOut.data[2] = float(RC.Key.CH[0] / 660.0f) * MAX_CHASSIS_VX_SPEED *remoteSensitivity * 0.07;
    return speedOut;
}
vec3f Chassis::keyboardCtrl()
{
    vec3f speedOut = {0};
    speedOut.data[0] = ((RC.Key.W) - RC.Key.S) * MAX_CHASSIS_VX_SPEED * keyboardSensitivity;
    speedOut.data[1] = (RC.Key.D - RC.Key.A) * MAX_CHASSIS_VY_SPEED * keyboardSensitivity;
    speedOut.data[2] = float(RC.Key.CH[6] * YawMouse_Sensitivity * 0.02);
    return speedOut;
}
vec3f Chassis::customCtrl()
{
    vec3f speedOut = {0};
    if (customFlag)
    {
        speedOut.data[0] = customSpeedIn.data[0];
        speedOut.data[1] = customSpeedIn.data[1];
        speedOut.data[2] = customSpeedIn.data[2];
    }
    else
    {
        memset(&speedOut, 0, sizeof(speedOut));
    }
    return speedOut;
}

void Chassis::speedCompute()
{
    if (customFlag)
    {
        customSpeedOut = customCtrl();
        for (u8 i = 0; i < 3; i++)
        {
            /* code */
            allSpeed.data[i] = customSpeedOut.data[i] + followSpeed.data[i];
        }
    }
    else
    {
        remoteSpeed = remoteCtrl();
        keyboardSpeed = keyboardCtrl();

        for (u8 i = 0; i < 3; i++)
        {
            /* code */
            allSpeed.data[i] = remoteSpeed.data[i] + keyboardSpeed.data[i];
            if (rotateFlag)
                allSpeed.data[i] += rotateSpeed.data[i];
            else if (swingFlag)
                allSpeed.data[i] += swingSpeed.data[i] + followSpeed.data[i];
            else
                allSpeed.data[i] += followSpeed.data[i];
        }
    }
    if (lockChassis) //是否锁底盘
    {
        memset(&allSpeed, 0, sizeof(allSpeed));
    }

    allSpeed.data[0] = LIMIT(allSpeed.data[0], -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);     //mm/s
    allSpeed.data[1] = LIMIT(allSpeed.data[1], -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);     //mm/s
    allSpeed.data[2] = LIMIT(allSpeed.data[2], -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);     //deg/s
}

float underVoltageProtectionKp = 1;

void Chassis::motorOutput()
{
	do{
		if(chassis.rotateFlag)
		{
			chassisSend.chassisMode = 1;
			if(chassis.flexRotate)
				chassisSend.chassisMode = 4;
			break;
		}
		if(chassis.swingFlag)
		{
			chassisSend.chassisMode = 2;
			break;
		}
		if(!chassis.ifChassisFollow)
		{
			chassisSend.chassisMode = 3;
			break;
		}
		if(chassis.flySloping)
		{
			chassisSend.chassisMode = 5;
			break;
		}
		if(chassis.customFlag)
		{
			chassisSend.chassisMode = 5;
			break;
		}
	}while(0);

	chassisSend.speedWS = allSpeed.data[0];
	chassisSend.speedAD = allSpeed.data[1];
	chassisSend.speedYaw = allSpeed.data[2];
}

bool Chassis::judgeIfMoving()
{
	static int time;
	if(ABS(allSpeed.data[0])>0||ABS(allSpeed.data[1])>0)
	{
		float speedtemp;
		for(u8 i = 0; i<4; i++)
		{
			speedtemp += ABS(powerHeatData.speedWS);
		}
		if(speedtemp>10)
		{
			time++;
			if(time>100)
			{
				time = 0;
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			time = 0;
			return 0;
		}
	}
	else
	{
		time = 0;
		return 0;
	}
}


void Chassis::run()
{
    /* code */
    if (!deforceFlag)
    {
        speedCompute();
        //chassis.powerCtrl();//当前功率在output里计算
    }
	motorOutput();
}


#include "parameter.h"


void Chassis::init()
{
}