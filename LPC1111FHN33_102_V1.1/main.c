#include "sys_cfg.h"
#include "uart.h"
#include "task.h"
#include "gpio.h"
#include "timer.h"
#include "adc.h"
#include "watchdog.h"

void test_task(void)
{
	//LPC11xx_print("io0_2 = ", gpio_get_value(GPIO_GRP0, 2), 1);
}

int main(void)
{
	uint32_t i;
	//uint32_t j;
	//����rom, ram, flash1, flash2, gpio, UARTģ��ʱ��
	SYSAHBCLK_init();
	uart_init(115200); //������Ϊ115200
	//clock_out();
	power_12v_on();
	resetkey_init(); //12v��Դ���ذ�ť��ʼ��
	
	timer_init(LPC_TMR32B1); //32λ��ʱ��1��ʼ��, ����delay_xxϵ�о�ȷ��ʱ����
	//timer_start(LPC_TMR32B1, 0, 1);
	
	timer_init(LPC_TMR16B0); //16λ��ʱ��0��ʼ��, ���ڷ��Ȳ���
	timer_start(LPC_TMR16B0, 1, 30000); //30s���Ե����ݾ��ǲ��Ե���ת��(�����ϵ��Ӧ���ṩ), ��λ: rpm
	
	timer_init(LPC_TMR16B1); //16λ��ʱ��1��ʼ��, ����taskʱ��Ƭ����ʱ��
	timer_start(LPC_TMR16B1, 1, 1); // ����taskʱ��Ƭ����ʱ��
	
	adc_init();
	
	indicator_init();
	
	//ʹ�ö�ʱ����ܽŷ���ķ���ģ��pwm
	pwm_init();

	speed_gpio_init();

	watchdog_init();
	
	LPC11xx_print("system init done!", 0, 1);
	
	//ע������	������void(*p_calbak)(void);
	task_register(indicator_ctrl, 1, 500);//ϵͳ״̬��, 1s�ж�ִ��һ��
	
	task_register(adc_start, 1, 1000); // 2sִ��һ���¶Ȼ�ȡ
	//task_register(test_task, 1, 500);
	while(1)
	{
		for(i = 0; i < TASK_MAX_NUM; ++i)
		{
			if(task[i].run)
			{
				task[i].p_calbak();//��������
				task[i].run = 0; //��������Ϊ��������״̬
			}
		}
		
		watchdog_feed();
	}
	
}

