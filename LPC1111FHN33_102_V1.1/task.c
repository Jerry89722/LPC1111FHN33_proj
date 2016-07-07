#include "task.h"
#include "uart.h"

TASK_COMPONENTS task[TASK_MAX_NUM] = {0};

int8_t task_register(void (*p_task)(void), uint8_t run_status, uint16_t interval)
{
	uint8_t i;
	if(interval <= 0 || p_task == NULL) 
	{
		LPC11xx_print("task cannot be registered", 0, 1);
		return -1;
	}
	for(i = 0; i < TASK_MAX_NUM; ++i)
	{
		if(task[i].p_calbak == NULL)
		{
			task[i].p_calbak = p_task;
			task[i].interval = interval;
			task[i].rest_time = interval;
			task[i].run = run_status;
			break;
		}
	}
	if(i == TASK_MAX_NUM)
	{
		LPC11xx_print("task queue is full", 0, 1);
		return -1;
	}
	return 0;
}

int8_t task_unregister(void (*p_task)(void))
{
	uint8_t i;
	for(i = 0; i < TASK_MAX_NUM; ++i)
	{
		if(task[i].p_calbak == p_task)
		{
			task[i].p_calbak = NULL;
			task[i].run = 0;
			break;
		}
	}
	if(i == TASK_MAX_NUM)
	{
		LPC11xx_print("no this task in queue", 0, 1);
		return -1;
	}
	return 0;
}
