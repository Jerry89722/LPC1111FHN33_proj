#include "LPC11xx.h"
#include "sys_cfg.h"
#include "gpio.h"
#include "uart.h"
#include "task.h"
#include "timer.h"

uint32_t rotation = 0;

/*
	gpio_init();
	����: gpio�ڳ�ʼ��
	����: 
	����ֵ:
*/

void gpio_init(volatile uint32_t* reg_addr)
{
	uint32_t io_addr = (uint32_t)reg_addr;
	if((uint32_t)reg_addr < (uint32_t)(&(LPC_IOCON->PIO2_6)) || 
		(uint32_t)reg_addr > (uint32_t)(&(LPC_IOCON->RXD_LOC)))
	{
		//�����io�ڵ�ַ���Ϸ�
		return;
	}
	peripherals_clk_switch(AHBCLKCTRL_IOCON, 1);
	*reg_addr &= ~0x7;
	switch(io_addr)
	{
	case (uint32_t)&(LPC_IOCON->RESET_PIO0_0):
	case (uint32_t)&(LPC_IOCON->SWCLK_PIO0_10):
	case (uint32_t)&(LPC_IOCON->R_PIO0_11):
	case (uint32_t)&(LPC_IOCON->R_PIO1_0):
	case (uint32_t)&(LPC_IOCON->R_PIO1_1):
	case (uint32_t)&(LPC_IOCON->R_PIO1_2):
	case (uint32_t)&(LPC_IOCON->SWDIO_PIO1_3):
		*reg_addr |= 1;
		break;
	
	default:
		break;
	}
	peripherals_clk_switch(AHBCLKCTRL_IOCON, 0);
}

/* gpio_set_value()
����: ����gpio�ķ���͵�ƽ
����:
	uint8_t PIOx:gpio����� (��LPC11xx_gpio.h�к궨��)
	uint32_t gpio_num:gpiogpio������� (0~11) 
	uint8_t direction:1����/��� (��LPC11xx_gpio.h�к궨��)
	uint8_t level: ��/�͵�ƽ, ��Ϊ����ʱ,��ƽ���ò���Ч (��LPC11xx_gpio.h�к궨��)
	����ֵ: ��
*/
void gpio_set_value(uint8_t PIOx, uint32_t gpio_num, uint8_t direction, uint8_t level)
{
	LPC_GPIO_TypeDef* port[4] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3};
	
	//��������
	if(direction)
		port[PIOx]->DIR |= (1<<gpio_num);
	else 
	{
		port[PIOx]->DIR &= ~(1<<gpio_num);
		return; //����ʱֱ�ӷ���
	}
	
	//��ƽ����
	if(level)
		port[PIOx]->DATA |= (1<<gpio_num);
	else
		port[PIOx]->DATA &= ~(1<<gpio_num);
}

/*
	gpio_get_value()
	����: ��ȡgpio�ĵ�ƽ
	����: 
		uint8_t PIOx: gpio�����
		uint32_t gpio_num: gpio�������
*/
uint8_t gpio_get_value(uint8_t PIOx, uint32_t gpio_num)
{
	LPC_GPIO_TypeDef* port[4] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3};
	
	return (((port[PIOx]->DATA) >> gpio_num) & 1);
}

/*
	IO_int_init()
	����: ��ʼ��gpio���ж�, �Կ���ָ���ܽŵ��ж�
	����: 
		uint8_t PIOx: gpio�����
		uint32_t gpio_num: gpio�������
		uint8_t trigger_type: �жϵĴ�����ʽ, 0 Ϊ���ش���, 1Ϊ��ƽ����
*/
void IO_int_enable(uint8_t PIOx, uint32_t gpio_num, uint8_t trigger_type)
{
	LPC_GPIO_TypeDef* port[4] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3};
	
	if(trigger_type)
	{
		port[PIOx]->IS |= 1 << gpio_num; //��ƽ����
		port[PIOx]->IEV |= 1<<gpio_num;  //�͵�ƽ����
	} 
	else
	{
		port[PIOx]->IS &= ~(1 << gpio_num);//��Ϊ���ش���
		port[PIOx]->IEV |= (1 << gpio_num);//��Ϊ�½��ش���
	}
	port[PIOx]->IE |= 1 << gpio_num;//ȡ���ж�����
	
	NVIC_EnableIRQ(EINT3_IRQn);//EINT0_IRQn~EINT3_IRQnΪö������������, ����"-PIOx"��ȷ���жϺ�
	NVIC_EnableIRQ(EINT0_IRQn);
}

void IO_int_disable(uint8_t PIOx, uint32_t gpio_num)
{
	LPC_GPIO_TypeDef* port[4] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3};
	port[PIOx]->IE &= ~(1 << gpio_num);// �����ж�
	port[PIOx]->IC |= 1 << gpio_num;   // ����ж�
}

/////////////////////////////////����Ϊͨ�ú���, ����Ϊ������Ŀ�е���/////////////////////////////

//��12v����
void power_12v_on(void)
{
	gpio_init(&(LPC_IOCON->PIO1_9));//��Ϊgpio
	gpio_set_value(GPIO_GRP1, 9, PIN_OUTPUT, LEVEL_LOW);
}

// 12��Դ����
void power_12v_off(void)
{
	gpio_set_value(GPIO_GRP1, 9, PIN_OUTPUT, LEVEL_HIGH);		//!gpio_get_value(GPIO_GRP1, 9));
}

//////led alarm _ctrl ///////start
void indicator_init(void)
{
	// led
	gpio_init(&(LPC_IOCON->PIO0_6));
	gpio_init(&(LPC_IOCON->PIO0_7));
	gpio_init(&(LPC_IOCON->PIO0_9));
	gpio_init(&(LPC_IOCON->SWCLK_PIO0_10));
	
	//6, 9 Ϊ����
	gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, LEVEL_LOW);
	gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, LEVEL_LOW);
	//7, 10 Ϊ�Ƶ�
	gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, LEVEL_HIGH);
	gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, LEVEL_HIGH);
	
	//speaker
	gpio_init(&(LPC_IOCON->PIO1_8));
	gpio_set_value(GPIO_GRP1, 8, PIN_OUTPUT, LEVEL_LOW);
}

void indicator_ctrl(void)
{
	static uint8_t last_state = 0;
	uint8_t run_state = rcv_data_translate();
	if(run_state == 4)
		run_state = 1;
	else if(run_state < 4)
		run_state = 0;
	else if(run_state == 0xff)
		return;
	
	if(run_state != last_state){
		last_state = run_state;
		
		//6, 9 Ϊ����
		gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, run_state);
		gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, run_state);
		//7, 10 Ϊ�Ƶ�
		gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, !run_state);
		gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, !run_state);
		
		//speaker ctrl
		gpio_set_value(GPIO_GRP1, 8, PIN_OUTPUT, run_state);
	}
}
//////led_ctrl ///////end

///////key_ctrl start/////////
void resetkey_init(void)
{
	gpio_init(&(LPC_IOCON->PIO3_4));
	gpio_set_value(GPIO_GRP3, 4, PIN_INPUT, LEVEL_HIGH);
	IO_int_enable(GPIO_GRP3, 4, 0);
}

void snd_poweroff(void)
{
	LPC11xx_print("poweroff", 0, 1);
	if( rcv_data_translate() == 0xff){
		task_unregister(snd_poweroff);
		delay_sec(5);
		LPC11xx_print("poweroff done !", 0, 1);
		power_12v_off();
		rvc_data_reset();
		IO_int_enable(GPIO_GRP3, 4, 0);
	}
}

void PIOINT3_IRQHandler(void)
{
	IO_int_disable(GPIO_GRP3, 4);
	delay_ms(50);
	if(gpio_get_value(GPIO_GRP1, 9)) {
		power_12v_on();
		IO_int_enable(GPIO_GRP3, 4, 0);
	} else {
		task_register(snd_poweroff, 1, 500);
	}
}
///////key_ctrl end//////////

////// pwm gpio /////////
void gpio_pwm_init(void)
{
	gpio_init(&(LPC_IOCON->PIO0_3));
	gpio_set_value(GPIO_GRP0, 3, PIN_OUTPUT, LEVEL_LOW);
}
////// pwm gpio ////////

////// MOTOR SPEED GET ////////
void speed_gpio_init(void)
{
	gpio_init(&(LPC_IOCON->PIO0_2));
	gpio_set_value(GPIO_GRP0, 2, PIN_INPUT, LEVEL_HIGH);
	IO_int_enable(GPIO_GRP0, 2, 0);
}

void PIOINT0_IRQHandler(void)
{
	IO_int_disable(GPIO_GRP0, 2);
	++rotation;
	IO_int_enable(GPIO_GRP0, 2, 0);
}

////// MOTOR SPEED GET ////////
void pwm_init(void)
{
	gpio_pwm_init();
	timer_init(LPC_TMR32B0);
	timer_start(LPC_TMR32B0, 0, 40);
}

/*
void clock_out(void)
{
	LPC_SYSCON->CLKOUTCLKSEL = 0;
	LPC_SYSCON->CLKOUTCLKSEL = 0x3;
	LPC_SYSCON->CLKOUTUEN = 0;
	LPC_SYSCON->CLKOUTUEN = 1;
	LPC_SYSCON->CLKOUTDIV = 100;
	LPC_IOCON->PIO0_1 = 1;
}
*/