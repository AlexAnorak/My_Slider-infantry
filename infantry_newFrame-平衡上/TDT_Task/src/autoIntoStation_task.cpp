#include "autoIntoStation.h"
#include "chassis_task.h"
#include "can.h"
#include "ammo_cover_task.h"
enum GYsenser
{
	LF = 0,LB = 1,FF = 2
};
#define FROUNT 0
#define ROTATE 1
#define LEFT 2

void quitAutoGoAmmo();

AutoIntoStation::AutoIntoStation(/* args */)
{
}

float* AutoIntoStation::getDistance()
{
    distance[LF] = can1Feedback.GYdistance[LF];
    distance[LB] = can1Feedback.GYdistance[LB];
    distance[FF] = can1Feedback.GYdistance[FF];
	averageDisOfL = (distance[LB]+distance[FF]) / 2.0f;
	errorWithL = (distance[LB] - distance[LF]);
    return distance;
}
void AutoIntoStation::run()
{
    getDistance();
    if(!intoStationFalg)
    {
		allreadyCnt = 0;
		haveInThere = 0;
		chassis.customSpeedIn.data[0] = 0;
		chassis.customSpeedIn.data[2] = 0;
		chassis.customSpeedIn.data[1] = 0;
        return;
    }
    else
    {
		chassis.customSpeedIn.data[0] = 0;
		chassis.customSpeedIn.data[1] = 0;
		chassis.customSpeedIn.data[2] = 0;

        if(haveInThere == 0)
        {
			allreadyCnt = 0;
            if(distance[FF]>distanceThreahold[FF]+40||distance[LB]>70||distance[LF]>70)
            {
				chassis.customSpeedIn.data[1] = 0;
				if(distance[FF]>distanceThreahold[FF]+150)
					chassis.customSpeedIn.data[0] = intoSpeed_Front;
				else
					chassis.customSpeedIn.data[0] = intoSpeed_Front/2;
            }
            else
            {
				chassis.customSpeedIn.data[0] = 0;
               if(distance[LB]<distanceThreahold[LB]-20||distance[LF]<distanceThreahold[LF]-20)
                {
                    chassis.customSpeedIn.data[1] = -intoSpeed_Right/2;
					
                }
                else
                {
					chassis.customSpeedIn.data[1] = 0;
                    haveInThere = 1;
                }
            }
        }
        else
        {
			if(distance[FF]>distanceThreahold[FF]+40||distance[LB]>70||distance[LF]>70 || distance[LB]<distanceThreahold[LB]-20||distance[LF]<distanceThreahold[LF]-20)
				haveInThere = 0;
			if((ABS(distance[LB] - distanceThreahold[LB]))>10 ||(ABS(distance[LF] - distanceThreahold[LF]))>10||(ABS(distance[FF] - distanceThreahold[FF]))>10)
			{
				allreadyCnt = 0;
				chassis.customSpeedIn.data[0] = -intoStation[FROUNT]->Calculate(distanceThreahold[FF]);
				chassis.customSpeedIn.data[1] = -intoStation[LEFT]->Calculate(distanceThreahold[LB]);
			}
			else
			{
				allreadyCnt++;
				if(allreadyCnt>100 )
				{
					chassis.customSpeedIn.data[0] = 0;
					chassis.customSpeedIn.data[1] = 0;
					chassis.customSpeedIn.data[2] = 0;
					chassis.customFlag = 0;
					intoStationFalg = 0;
					quitAutoGoAmmo();
				}
				else
				{
					ammoCover.setCoverOpen(1);	
					chassis.customSpeedIn.data[0] = -intoStation[FROUNT]->Calculate(distanceThreahold[FF]);
					chassis.customSpeedIn.data[1] = -intoStation[LEFT]->Calculate(distanceThreahold[LB]);				
				}
			}
        }
    }


}
void AutoIntoStation::selectPIDParam()
{

}
void AutoIntoStation::PIDParamInit()
{
    for (u8 i = 0; i < 3; i++)
    {
        /* code */
        intoStation[i] = new Pid(1);
        Senser[i] =  new PidParam;
    }
    Senser[FROUNT]->kp = 30;
    Senser[FROUNT]->ki = 20;
    Senser[FROUNT]->kd = 0;
	Senser[FROUNT]->integralErrorMax = 30;
    Senser[FROUNT]->resultMax = 2000;
	intoStation[FROUNT]->paramPtr = Senser[FROUNT];
	intoStation[FROUNT]->fbValuePtr[0] = &distance[2];

    Senser[ROTATE]->kp = 10;
    Senser[ROTATE]->ki = 0;
    Senser[ROTATE]->kd = 0;
	Senser[ROTATE]->integralErrorMax = 30;
    Senser[ROTATE]->resultMax = 300;
	intoStation[ROTATE]->paramPtr = Senser[ROTATE];
	intoStation[ROTATE]->fbValuePtr[0] = &errorWithL;

	Senser[LEFT]->kp = 60;
    Senser[LEFT]->ki = 40;
    Senser[LEFT]->kd = 0;
	Senser[LEFT]->integralErrorMax = 700;
    Senser[LEFT]->resultMax = 2000;
	intoStation[LEFT]->paramPtr = Senser[LEFT];
	intoStation[LEFT]->fbValuePtr[0] = &averageDisOfL;
	
	distanceThreahold[0] = 30;
	distanceThreahold[1] = 28;
	distanceThreahold[2] = 32;
}
struct AutoIntoStation intoStation;
