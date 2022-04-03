#ifndef _SCHEDULE_H
#define _SCHEDULE_H

#include "stm32f10x.h"


typedef struct _schedule {
    uint16_t   cnt_1ms;
    uint16_t   cnt_2ms;
    uint16_t   cnt_5ms;
    uint16_t   cnt_10ms;
    uint16_t   cnt_20ms;
    uint16_t   cnt_50ms;
    uint16_t   cnt_100ms;
    uint64_t   sysTime_ms;
    uint64_t   beatStart_ms;
} _schedule;

typedef struct _state 
{
    int16_t ShootNum;			//�����ӵ�����
	float ShootNum_f;			//�����ӵ���������ʱ��
	int16_t OldShootNum;		//��һ���ӵ�����
	int16_t Standby_PWM;		//������������ʱռ�ձ�
	int16_t StartingTime;		//��ʼʱ�������
	int16_t UnlockTime;			//����ʱ�������
	u8 INFANTRY_ID; //ID
	u8 paraLoadFlag; //�Ƿ����PID
	u8 last_INFANTRY_ID;

	int16_t SN_now_spd_A;		//��ǰ�ű����������ٶ�
	int16_t SN_last_spd_A;		//��һ�δű����������ٶ�
	int16_t SN_start_spd_A;		//��һ���ȶ�ʱ�Ĵű����������ٶ�
	int16_t SN_error_spd_A;		//�ű����������ٶȲ�

	int16_t SN_now_spd_B;		//��ǰ�ű����������ٶ�
	int16_t SN_last_spd_B;		//��һ�δű����������ٶ�
	int16_t SN_start_spd_B;		//��һ���ȶ�ʱ�Ĵű����������ٶ�
	int16_t SN_error_spd_B;		//�ű����������ٶȲ�

	
	/****ǿ�ƿ���ʱ�õ��ı���****/
	int16_t ForceOpenloop_setspeed_A;//ǿ�ƿ����趨ֵ
	int16_t ForceOpenloop_setspeed_B;//ǿ�ƿ����趨ֵ
	/****����ʱ�õ��ı���****/
	int16_t openloop_setspeed_A;//�����趨ֵ
	int16_t openloop_setspeed_B;//�����趨ֵ
	/****��������ʱ�õ��ı���****/
	int16_t offline_setspeed_A;//���������趨ֵ
	int16_t offline_setspeed_B;//���������趨ֵ
	/****�ű��������߿����õ��ı���****/
	int16_t AS5048_Offline_setspeed_A;//�ű���������ʱ�������趨ֵ
	int16_t AS5048_Offline_setspeed_B;//�ű���������ʱ�������趨ֵ
	//�ջ�ʱ�õ��ı���
	int16_t MaxSetSpd_A;		//�ջ�����趨ֵ
	int16_t MaxSetSpd_B;		//�ջ�����趨ֵ
	int16_t OpenLoopMaxSetSpd_A;		//��������趨ֵ
	int16_t OpenLoopMaxSetSpd_B;		//��������趨ֵ
	int32_t offline_check;		//�������߼��

	/****״̬��־λ****/
	u8 ReadyToFire;				//������
	u8 ReadyToStart;			//Ħ�����Ѿ�׼����
	u8 WaitReadyToUnlock;		//�ȵ�Ħ����ת���ȶ��Ž���������ı�־λ
	u8 EnableQuickStart;		//�����������
	u8 ForceOpenLoop;			//ǿ�ƿ���
	u8 OpenLoop;				//����״̬
	u8 StopState;				//ֹͣ��־λ���趨ֵΪ0ʱ��1������ջ�ʱ��0��
	u8 NewStart;				//��ʼ��־λ������ջ�ʱ��1������pid������0���������pid��
	u8 OpenLoopMaxSetSpdLimit;	//�Ƿ��������ٶ����ƣ�������ʱ���㣬���ڸ������ٶ��޷���
	
	u8 AS5048_A_Offline;		//�ű��������߱�־λ
	u8 AS5048_B_Offline;		//�ű��������߱�־λ
} _state;

void Loop_1000Hz(void); //1msִ��һ��
void Loop_500Hz(void);	//2msִ��һ��
void Loop_200Hz(void);	//5msִ��һ��
void Loop_100Hz(void);	//10msִ��һ��
void Loop_50Hz(void);	  //20msִ��һ��
void Loop_20Hz(void);	  //50msִ��һ��
void Loop_10Hz(void);	  //100msִ��һ��

void Loop(void);

extern struct _schedule schedule;
extern struct _state state;

#endif
