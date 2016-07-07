#include "LPC11xx.h"
#include "uart.h"
#include "sys_cfg.h"
#include "timer.h"
#include "task.h"
#include "gpio.h"

static uint8_t level = 1;
static uint32_t speed_arr[3] = {50, 30, 10};
/*
timerʹ�����˵��:
	timer16_0, ���ڲ���pwm, ֻʹ����MR3 MR2��ƥ��, MR3 = 0.1S
	timer16_1, ����taskʱ��Ƭ������ʱ��
	timer32_0, δʹ��
	timer32_1, ���ھ�ȷ��delayϵ�к���, ʹ����MR3��ƥ��
*/

/*
	timer_init();
	����: Ӳ����ʱ����ʼ��
	����: 
		LPC_TMR_TypeDef* p_timer: ��ʱ���Ĵ�����ַ
	����ֵ: ��	
*/

void timer_init(LPC_TMR_TypeDef* p_timer)
{
	//������Ӧ�Ķ�ʱ��ģ��ʱ��, ����Ԥ��Ƶ�Ĵ���, �ж϶�ʱʱ���Ƿ�Ϸ�
	switch((uint32_t)p_timer)
	{
	case (uint32_t)LPC_TMR32B0:
		peripherals_clk_switch(AHBCLKCTRL_CT32B0, 1);
		p_timer->PR = 11;  // 1us
		break;
	case (uint32_t)LPC_TMR32B1:
		peripherals_clk_switch(AHBCLKCTRL_CT32B1, 1);
		p_timer->PR = 11;  // 1us
		break;
	
	case (uint32_t)LPC_TMR16B0:
		peripherals_clk_switch(AHBCLKCTRL_CT16B0, 1);
		p_timer->PR = 11999; //1ms
		break;
	
	case (uint32_t)LPC_TMR16B1:
		peripherals_clk_switch(AHBCLKCTRL_CT16B1, 1);  
		p_timer->PR = 11999;    // 1ms
		break;
	
	default:
		LPC11xx_print("timer reg addr is invalid", 0, 1);
		return;
	}
	
	p_timer->IR |= 0x1f; //�жϸ�λ
	p_timer->MCR |= 3<<9; //MR3ƥ��ʱ�����ж�, ��λTC
	
	//ʹ�ܼ����������ָ�λ
	p_timer->TCR &= ~3;
	p_timer->TCR |= 2;
}

/*
	timer_start();
	����: ������ʱ��
	����: 
		LPC_TMR_TypeDef* timer: ��ʱ���Ĵ�����ַ
		uint8_t loop: ѭ��/��ѭ����ʱ, 1ѭ��, 0��ѭ��
		uint32_t interval: ��ʱʱ��, λ��ʱ��ʱ�䵥λ�ֱ�Ϊus/ms
	����ֵ: ��
*/
void timer_start(LPC_TMR_TypeDef* p_timer, uint8_t loop, uint32_t interval)
{
	if(loop > 1)
	{
		LPC11xx_print("loop invalid", 0, 1);
		return;
	}
	
	//���ܼ�����, �����ָ�λ
	p_timer->TCR &= ~3;
	p_timer->TCR |= 2;
	
	if(loop) //һֱѭ��ִ��
		p_timer->MCR &= ~(1<<11); 
	else //ִ��һ��
		p_timer->MCR |= 1<<11;//3ƥ���ֹͣpc��tc, ���ܼ�����

	//���ö�ʱʱ��, ������ʱ���жϷ�����
	switch((uint32_t)p_timer)
	{
	case (uint32_t)LPC_TMR32B0:
		NVIC_EnableIRQ(TIMER_32_0_IRQn);
		break;
	case (uint32_t)LPC_TMR32B1:
		NVIC_EnableIRQ(TIMER_32_1_IRQn);
		break;
	case (uint32_t)LPC_TMR16B0:
		NVIC_EnableIRQ(TIMER_16_0_IRQn);
		break;
	case (uint32_t)LPC_TMR16B1:
		NVIC_EnableIRQ(TIMER_16_1_IRQn);
		break;
	
	default:
		LPC11xx_print("timer reg addr is invalid", 0, 1);
		break;
	}
	p_timer->MR3 = interval; //��λ:1ms/us
	//������ʹ��, ֹͣ��λ������ʼ��ʱ
	p_timer->TCR &= ~3;
	p_timer->TCR |= 1;
}

/*
	�жϷ�����
*/
void TIMER32_0_IRQHandler(void)
{
	uint8_t pwm_state = gpio_get_value(GPIO_GRP0, 3);
	
	LPC_TMR32B0->IR |= 1<<3; //�жϸ�λ
	
	gpio_set_value(GPIO_GRP0, 3, PIN_OUTPUT, !pwm_state);
	
	if(pwm_state){
		timer_start(LPC_TMR32B0, 0, speed_arr[level - 1]);
	} else {
		timer_start(LPC_TMR32B0, 0, period - speed_arr[level - 1]);
	}
	
	LPC_TMR32B0->IR |= 1<<3;
}

void TIMER32_1_IRQHandler(void)
{
	LPC_TMR32B1->IR |= 1<<3;//�жϸ�λ
	LPC11xx_print("32_1", 0, 1);
}


//���ڿ���ģ��pwm���Ʒ���ת��
void TIMER16_0_IRQHandler(void)
{
	LPC_TMR16B0->IR |= 1<<3;//�жϸ�λ
	//LPC11xx_print("rotation = ", rotation, 1);
	rotation = 0;
}

//1ms����һ���ж�
void TIMER16_1_IRQHandler(void)
{
	uint8_t i;
	LPC_TMR16B1->IR |= 1<<3;//�жϸ�λ
	
	for(i = 0; i < TASK_MAX_NUM; ++i)
	{
		if(task[i].p_calbak)
		{
			if((--task[i].rest_time) == 0) //������ʱ��Ϊ0�Ǽ�������״̬, ���������ü��
			{
				task[i].rest_time = task[i].interval;
				task[i].run = 1;
			}
		}
	}
}

/*
	��ʱ����, ʹ��32λ��ʱ��1
*/
void delay_sec(uint32_t duration) //max 4294s
{
	if(duration > 1000) //1000s
	{
		LPC11xx_print("delay duration is invaild", 0, 1);
		return;
	}
	timer_start(LPC_TMR32B1, 0, duration*1000000);
	while(LPC_TMR32B1->TCR & 1);
}

void delay_ms(uint32_t duration) //max 4294967ms
{
	if(duration> 100000) //100s
	{
		LPC11xx_print("delay duration is invaild", 0, 1);
		return;
	}
	timer_start(LPC_TMR32B1, 0, duration*1000);
	while(LPC_TMR32B1->TCR & 1);
}

void delay_us(uint32_t duration)//max 4296967296us
{
	if(duration > 1000000) //1s
	{
		LPC11xx_print("delay duration is invaild", 0, 1);
		return;
	}
	timer_start(LPC_TMR32B1, 0, duration);
	while(LPC_TMR32B1->TCR & 1);
}

//pwm_init();
//PIO0_1  010  CT32B0_MAT2
//PIO1_8  001  CT16B1_CAP0
//PIO0_2  010  CT16B1_CAP0
//PIO1_9  001  CT16B1_MAT0
//PIO0_8  010  CT16B0_MAT0
//PIO0_9  010  CT16B0_MAT0
//PIO0_10 011  CT16B0_MAT2
//PIO1_10 010  CT16B1_MAT1
//PIO0_11 011  CT32B0_MAT3
//PIO1_0  011  CT32B1_CAP0
//PIO1_1  011  CT32B1_MAT0
//PIO1_2  011  CT32B1_MAT1
//PIO1_3  011  CT32B1_MAT2
//PIO1_4  010  CT32B1_MAT3
//PIO1_5  010  CT32B0_CAP0
//PIO1_6  010  CT32B0_MAT0
//PIO1_7  010  CT32B0_MAT1



#if 0
/*
	timer_init()Ĭ�Ͽ���MR3ƥ�临λ�����ж�, pwm����Ҫ�жϿ��Թر��ж�
*/

void pwm_init(LPC_TMR_TypeDef* p_timer, __IO uint32_t* addr)
{
	//����pwm�����io��
	peripherals_clk_switch(AHBCLKCTRL_IOCON, 1);
	switch((uint32_t) addr)
	{
	case (uint32_t)&(LPC_IOCON->PIO0_1):
	case (uint32_t)&(LPC_IOCON->PIO0_2):
	case (uint32_t)&(LPC_IOCON->PIO0_8):
	case (uint32_t)&(LPC_IOCON->PIO0_9):
	case (uint32_t)&(LPC_IOCON->PIO1_10):
	case (uint32_t)&(LPC_IOCON->PIO1_4):
	case (uint32_t)&(LPC_IOCON->PIO1_5):
	case (uint32_t)&(LPC_IOCON->PIO1_6):
	case (uint32_t)&(LPC_IOCON->PIO1_7):
		//010
		*addr &= ~0x7;
		*addr |= 2;
		break;
	
	case (uint32_t)&(LPC_IOCON->PIO1_8):
	case (uint32_t)&(LPC_IOCON->PIO1_9):
		//001
		*addr &= ~7;
		*addr |= 1;
		break;
	
	case (uint32_t)&(LPC_IOCON->SWCLK_PIO0_10):
	case (uint32_t)&(LPC_IOCON->R_PIO0_11):
	case (uint32_t)&(LPC_IOCON->R_PIO1_0):
	case (uint32_t)&(LPC_IOCON->R_PIO1_1):
	case (uint32_t)&(LPC_IOCON->R_PIO1_2):
	case (uint32_t)&(LPC_IOCON->SWDIO_PIO1_3):		
		//011
		*addr &= ~7;
		*addr |= 3;
		break;
	default:
		//�Ƿ���ַ, ��������Ϊpwm
		break;
	}
	peripherals_clk_switch(AHBCLKCTRL_IOCON, 0);
	
	//����pwm�Ĳ���
	p_timer->MR2 = period;
	p_timer->CTCR &= ~3; //��ʱ��ģʽ����
	p_timer->PWMC |= (3<<2);
	
	p_timer->MCR &= ~(1<<9); //�ر�MR3ƥ��ʱ���ж�
	//NVIC_DisableIRQ(TIMER_16_0_IRQn);
}

//pwm_ctrl();

void pwm_ctrl(LPC_TMR_TypeDef* p_timer, uint8_t level)
{
	static uint8_t last_level = 0;
	if(level < 1)
		level = 1;
	else if(level > 3)
		level = 3;
	if(level != last_level)
	{
		p_timer->TCR &= ~3;  //10 ����������, ���ָ�λ
		p_timer->TCR |= 2;
		
		p_timer->MR2 = (3-level) * period / 2; //����ת��
		
		p_timer->TCR &= ~3;  //01 ������ʹ��, �����λ
		p_timer->TCR |= 1;
		
		last_level = level;
	}
}

void speed_ctrl(void)
{
	static uint8_t last_level = 0;
	uint8_t tmp = rcv_data_translate();
	if(tmp == 0xff)
		return;
	else
		level = tmp;

	LPC11xx_print("temp_level = ", level, 1);

	if(last_level != level)
	{
		last_level = level;
		if(level == 3){
			gpio_set_value(GPIO_GRP0, 3, PIN_OUTPUT, 1);
		return ;
	}
		gpio_set_value(GPIO_GRP0, 3, PIN_OUTPUT, 0);
		timer_start(LPC_TMR16B0, 0, speed_arr[level - 1]);
	}
}
#endif
