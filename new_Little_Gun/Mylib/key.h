#ifndef __KEY_H__
#define __KEY_H__
#include "stm32f10x.h"
typedef struct _keyvalue {
    u16 keyv[6];
    u16 keyv_last[6];
    u8  kflag[4];
} _keyvalue;
extern _keyvalue kv;

void KEY_Configuration(void);
void LED_Configuration(void);
void  get_key(void);
#endif
