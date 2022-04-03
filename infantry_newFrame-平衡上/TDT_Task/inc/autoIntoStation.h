#ifndef __AUTO_INTO_STATION_H
#define __AUTO_INTO_STATION_H
#include "board.h"
#include "pid.h"
struct AutoIntoStation
{
private:
    /* data */
public:
    AutoIntoStation(/* args */);
    float* getDistance();
    float distance[3];
    void run();
    void selectPIDParam();
    void PIDParamInit();
    Pid *intoStation[3];
    PidParam *Senser[3];
    u8 intoStationFalg = 0;
    u8 haveInThere = 0;
    float distanceThreahold[3];
	float errorWithL = 0;
	float averageDisOfL;
	float intoSpeed_Front = 2000;
	float intoSpeed_Right = 1200;
	int allreadyCnt = 0;
};
extern AutoIntoStation intoStation;
#endif