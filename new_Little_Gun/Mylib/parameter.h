#ifndef __PARAMETER_H__
#define __PARAMETER_H__
#include "main.h"

#define pidFriction 0
#define SHIP 1
#define TWONETWONE 2
#define AURORA 3
#define GALAXY 4

extern int Friction1_Speed_Set;//×ó
extern int Friction2_Speed_Set;//ÓÒ
void TDT_Get_PIDparameters(pid* pidStruct, u8 pidIndex);
void TDT_State_Init(void);
void reloadparameter(u8 parameterIndex);
#endif
