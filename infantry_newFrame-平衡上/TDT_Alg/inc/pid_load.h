#ifndef __PID_LOAD_H
#define __PID_LOAD_H

#include "board.h"
#include "pid.h"
void loadChassisPidParam();
void loadGimbalPidParam(PidParam *paramInner, PidParam *paramOuter);
void loadAmmoBoosterPidParam(PidParam *paramInner, PidParam *paramOuter);
void loadAmmoCoverPidParam(PidParam *paramInner, PidParam *paramOuter);

#endif