#include "sys_cfg.h"
#include "uart.h"
#include "task.h"
#include "gpio.h"
#include "timer.h"
#include "adc.h"
#include "watchdog.h"

void test_task(void)
{
	LPC11xx_print("io0_2 = ", gpio_get_value(GPIO_GRP0, 2), 1);
}

int main(void)
{
	uint32_t i;
	//uint32_t j;
	//开启rom, ram, flash1, flash2, gpio, UART模块时钟
	SYSAHBCLK_init();
	uart_init(115200); //波特率为115200
	//clock_out();
	power_12v_on();
	key_init(); //12v电源开关按钮初始化
	
	timer_init(LPC_TMR32B1); //32位定时器1初始化, 用于delay_xx系列精确延时函数
	//timer_start(LPC_TMR32B1, 0, 1);
	
	timer_init(LPC_TMR16B0); //16位定时器0初始化, 用于风扇测速
	timer_start(LPC_TMR16B0, 1, 30000); //30s测试到数据就是测试到的转速(换算关系供应商提供), 单位: rpm
	
	timer_init(LPC_TMR16B1); //16位定时器1初始化, 用于task时间片管理定时器
	timer_start(LPC_TMR16B1, 1, 1); // 开启task时间片管理定时器
	
	adc_init();
	
	indicator_init();
	
	timer_init(LPC_TMR32B0);
	pwm_init(LPC_TMR32B0, (uint32_t*)&(LPC_IOCON->R_PIO0_11));

	speed_gpio_init();

	watchdog_init();
	
	LPC11xx_print("sys init done!", 0, 1);

	LPC11xx_print("adc_start: ", (uint32_t)adc_start, 1);
	LPC11xx_print("speed_ctrl: ", (uint32_t)speed_ctrl, 1);
	LPC11xx_print("stata_idc: ", (uint32_t)status_deal, 1);
	LPC11xx_print("long_pre: ", (uint32_t)long_press_chk, 1);
	LPC11xx_print("delay_po: ", (uint32_t)delay_poweroff, 1);
	LPC11xx_print("snd_po: ", (uint32_t)snd_poweroff, 1);


	//注册任务	任务函数void(*p_calbak)(void);	
	task_register(adc_start, 1, 1000); // 2s执行一次温度获取
	
	task_register(speed_ctrl, 1, 1000);
	task_register(status_deal, 1, 250);
	//task_register(test_task, 1, 1000);
	while(1)
	{
		for(i = 0; i < TASK_MAX_NUM; ++i)
		{
			if(task[i].run)
			{
				task[i].p_calbak();//运行任务
				task[i].run = 0; //设置任务为不可运行状态
			}
		}
		
		watchdog_feed();
	}
}

