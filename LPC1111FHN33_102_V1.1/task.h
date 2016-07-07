#ifndef __TASK_H
#define __TASK_H

#include "LPC11xx.h"
#include "string.h"
#define TASK_MAX_NUM	6

typedef struct {
	uint8_t run; //����״̬, 1ʱ��������, 0ʱ����������
	uint16_t rest_time; //��ʱ��ʣ��ʱ��
	uint16_t interval; // �������еļ��ʱ��, ��λms
	void (*p_calbak)(void); //װ�������ִ�к���
} TASK_COMPONENTS;

extern TASK_COMPONENTS task[TASK_MAX_NUM];

int8_t task_register(void (*p_task)(void), uint8_t run_status, uint16_t interval);
int8_t task_unregister(void (*p_task)(void));

#endif
