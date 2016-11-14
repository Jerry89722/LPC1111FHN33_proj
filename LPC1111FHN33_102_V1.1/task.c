#include "task.h"
#include "uart.h"

TASK_COMPONENTS task[TASK_MAX_NUM] = {0};

int8_t task_register(void (*p_task)(void), uint8_t run_status, uint16_t interval)
{
	uint8_t i;
	uint8_t task_no = 0xff;
	if(interval <= 0 || p_task == NULL) 
	{
		LPC11xx_print("task cannot be registered", 0, 1);
		return -1;
	}
	for(i = 0; i < TASK_MAX_NUM; ++i)
	{
		if(task[i].p_calbak == p_task){
			task_no = 0xff;
			break;
		}
		if(task[i].p_calbak == NULL && task_no == 0xff)
			task_no = i;
	}
	if(task_no != 0xff){
		task[task_no].p_calbak = p_task;
		task[task_no].interval = interval;
		task[task_no].rest_time = interval;
		task[task_no].run = run_status;
	}else{
		LPC11xx_print("task reg fail", 0, 1);
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
			//LPC11xx_print("unregister:", (uint32_t)p_task, 1);
			task[i].p_calbak = NULL;
			task[i].run = 0;
			//break;
		}
	}
	return 0;
}
