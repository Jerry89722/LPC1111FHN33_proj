#ifndef __TASK_H
#define __TASK_H

#include "LPC11xx.h"
#include "string.h"
#define TASK_MAX_NUM	6

typedef struct {
	uint8_t run; //运行状态, 1时允许运行, 0时不允许运行
	uint16_t rest_time; //定时器剩余时间
	uint16_t interval; // 任务运行的间隔时间, 单位ms
	void (*p_calbak)(void); //装填任务的执行函数
} TASK_COMPONENTS;

extern TASK_COMPONENTS task[TASK_MAX_NUM];

int8_t task_register(void (*p_task)(void), uint8_t run_status, uint16_t interval);
int8_t task_unregister(void (*p_task)(void));

#endif
