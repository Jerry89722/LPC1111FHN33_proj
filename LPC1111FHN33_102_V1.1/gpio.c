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
	IO_int_enable()
	����: ��ʼ��gpio���ж�, �Կ���ָ���ܽŵ��ж�
	����: 
		uint8_t PIOx: gpio�����
		uint32_t gpio_num: gpio�������
		uint8_t trigger_type: �жϵĴ�����ʽ, 0 Ϊ���ش���, 1Ϊ��ƽ����
*/
static void IO_int_enable(uint8_t PIOx, uint32_t gpio_num, uint8_t trigger_type)
{
	LPC_GPIO_TypeDef* port[4] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3};
	
	if(trigger_type)
	{
		port[PIOx]->IS |= 1 << gpio_num; //��ƽ����
		port[PIOx]->IEV &= ~(1<<gpio_num);  //�ߵ�ƽ����
	}
	else
	{
		port[PIOx]->IS &= ~(1 << gpio_num);//��Ϊ���ش���
		port[PIOx]->IEV &= ~(1 << gpio_num);//��Ϊ�����ش���
	}
	
	port[PIOx]->IE &= ~(1 << gpio_num);// �����ж�
	port[PIOx]->IC |= 1 << gpio_num;   // ����ж�
	while(port[PIOx]->MIS & (1 << gpio_num));
	port[PIOx]->IE |= 1 << gpio_num;//ȡ���ж�����
	
	NVIC_EnableIRQ(EINT3_IRQn);//EINT0_IRQn~EINT3_IRQnΪö������������
	NVIC_EnableIRQ(EINT1_IRQn);
	NVIC_EnableIRQ(EINT0_IRQn);
}

static void IO_int_disable(uint8_t PIOx, uint32_t gpio_num)
{
	LPC_GPIO_TypeDef* port[4] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3};
	port[PIOx]->IE &= ~(1 << gpio_num);// �����ж�
	port[PIOx]->IC |= 1 << gpio_num;   // ����ж�
	while(port[PIOx]->MIS & (1 << gpio_num));
}

uint8_t IO_int_is_on(uint8_t PIOx, uint32_t gpio_num)
{
	LPC_GPIO_TypeDef* port[4] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3};
	return (port[PIOx]->IE & (1 << gpio_num));
}

/////////////////////////////////����Ϊͨ�ú���, ����Ϊ������Ŀ�е���/////////////////////////////

//��12v����
void power_12v_on(void)
{
	//LPC11xx_print("on", 0, 1);
	gpio_init(&(LPC_IOCON->PIO1_9));//��Ϊgpio
	gpio_set_value(GPIO_GRP1, 9, PIN_OUTPUT, LEVEL_LOW);
}

// 12��Դ����
void power_12v_off(void)
{
	gpio_set_value(GPIO_GRP1, 9, PIN_OUTPUT, LEVEL_HIGH);	
}

//////led alarm _ctrl ///////start
void indicator_init(void)
{
	// led
	gpio_init(&(LPC_IOCON->PIO0_6));
	gpio_init(&(LPC_IOCON->PIO0_7));
	gpio_init(&(LPC_IOCON->PIO0_8));
	gpio_init(&(LPC_IOCON->PIO0_9));
	gpio_init(&(LPC_IOCON->SWCLK_PIO0_10));
	// speaker 
	gpio_init(&(LPC_IOCON->PIO1_8)); 
	
	//6, 7, 9 Ϊ����
	gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, LEVEL_HIGH);
	gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, LEVEL_HIGH);
	gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, LEVEL_HIGH);
	//8, 10 Ϊ�Ƶ�
	
	gpio_set_value(GPIO_GRP0, 8, PIN_OUTPUT, LEVEL_LOW);
	gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, LEVEL_LOW);
	
	
	//speaker
	gpio_init(&(LPC_IOCON->PIO1_8));
	gpio_set_value(GPIO_GRP1, 8, PIN_OUTPUT, LEVEL_LOW);
}
//////status_deal ///////end

///////key_ctrl start/////////
void key_init(void)
{
	gpio_init(&(LPC_IOCON->PIO3_4));
	gpio_set_value(GPIO_GRP3, 4, PIN_INPUT, LEVEL_HIGH);
	IO_int_enable(GPIO_GRP3, 4, 0);
	
	gpio_init(&(LPC_IOCON->SWDIO_PIO1_3));
	gpio_set_value(GPIO_GRP1, 3, PIN_INPUT, LEVEL_HIGH);
	IO_int_enable(GPIO_GRP1, 3, 0);
}

void snd_poweroff(void)
{
	LPC11xx_print("poweroff", 0, 1);
}

void delay_poweroff(void)
{
	static uint8_t power_cnt = 0;
	
	rcv_data2level();
	if(run_state == S_SYS_OFF_DONE){
		if(power_cnt++ < 7){
			LPC11xx_print("power_cnt = ", power_cnt, 1);
			return;
		}
		power_cnt = 0;
		task_unregister(delay_poweroff);
		gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, LEVEL_HIGH);
		gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, LEVEL_HIGH);
		gpio_set_value(GPIO_GRP0, 8, PIN_OUTPUT, LEVEL_HIGH);
		gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, LEVEL_HIGH);
		gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, LEVEL_HIGH);
		LPC11xx_print("poweroff done", 0, 1);
		power_12v_off();
		rcv_data_reset();
		//IO_int_enable(GPIO_GRP3, 4, 0);
		run_state = S_12V_OFF_DONE;
	}
}

uint8_t run_state = S_ON_ING;
// �������жϷ������
void PIOINT3_IRQHandler(void)
{
	IO_int_disable(GPIO_GRP3, 4);
	
	//delay_ms(100);
	LPC11xx_print("key int 3_4 catch", 0, 1);
	task_register(long_press_chk, 1, 50);
}

// 6820��λ�������жϷ������
void PIOINT1_IRQHandler(void)
{
	IO_int_disable(GPIO_GRP1, 3);
	
	LPC11xx_print("key int 1_3 catch", gpio_get_value(GPIO_GRP1, 3), 1);
	// task_register(long_press_chk, 1, 20);  //��������
	task_unregister(snd_poweroff);
	rcv_data_reset();
	run_state = S_ON_ING;
	gpio_set_value(GPIO_GRP1, 9, PIN_OUTPUT, LEVEL_LOW);
	task_register(speaker_ctrl, 1, 300);
	IO_int_enable(GPIO_GRP1, 3, 0);
}

void long_press_chk(void)
{
	static uint8_t cnt_key = 0;
	//LPC11xx_print("----press----", gpio_get_value(GPIO_GRP1, 3), 1);
	uint8_t i;
	/* // ��λ���ķ������, ����λ��Ӳ���������, ���ֻ�Ǽ��, ����������˷���Ҳû��
	if(!gpio_get_value(GPIO_GRP1, 3)){
		if(cnt_key++ > 3){
			LPC11xx_print("reset cmd get", 0, 1);
			cnt_key = 0;
			task_unregister(long_press_chk);
			task_unregister(snd_poweroff);
			rcv_data_reset();
			run_state = S_ON_ING;
			gpio_set_value(GPIO_GRP1, 9, PIN_OUTPUT, LEVEL_LOW);
			task_register(speaker_ctrl, 1, 300);
			IO_int_enable(GPIO_GRP1, 3, 0);
		}
	}else
	*/ 
	if(!gpio_get_value(GPIO_GRP3, 4)){
		if(cnt_key++ > 10){
			LPC11xx_print("power cmd get", 0, 1);
			cnt_key = 0;
			task_unregister(long_press_chk);
			if(gpio_get_value(GPIO_GRP1, 9)){  //����
				power_12v_on();
				run_state = S_ON_ING;
				task_register(speaker_ctrl, 1, 300); //����ʱ��������
				//IO_int_enable(GPIO_GRP3, 4, 0);
			}else{  //�ػ�
				task_register(snd_poweroff, 1, 500);
				run_state = S_OFF_ING;
				for(i = 0; i < TASK_MAX_NUM; ++i)
						LPC11xx_print("i = ", (uint32_t)task[i].p_calbak, 1);
			}
		}
	}else{
		cnt_key = 0;
		task_unregister(long_press_chk);
		IO_int_enable(GPIO_GRP3, 4, 0);
		//IO_int_enable(GPIO_GRP1, 3, 0);
	}
	
}
///////key_ctrl end//////////

////// pwm gpio /////////
void gpio_pwm_init(void)
{
#ifdef PWM_FAN
	gpio_init(&(LPC_IOCON->PIO0_3));
	gpio_set_value(GPIO_GRP0, 3, PIN_OUTPUT, LEVEL_LOW);
#else
	gpio_init(&(LPC_IOCON->R_PIO1_0));
	gpio_init(&(LPC_IOCON->R_PIO1_2));
	gpio_set_value(GPIO_GRP1, 0, PIN_OUTPUT, LEVEL_HIGH);
	gpio_set_value(GPIO_GRP1, 2, PIN_OUTPUT, LEVEL_HIGH);
#endif
}
////// pwm gpio ////////

////// MOTOR SPEED GET START////////
void speed_gpio_init(void)
{
#ifdef PWM_FAN
	gpio_init(&(LPC_IOCON->PIO0_2));
	gpio_set_value(GPIO_GRP0, 2, PIN_INPUT, LEVEL_HIGH);
	IO_int_enable(GPIO_GRP0, 2, 0);
#endif
}

void PIOINT0_IRQHandler(void)
{
	IO_int_disable(GPIO_GRP0, 2);
	++rotation;
	IO_int_enable(GPIO_GRP0, 2, 0);
}

////// MOTOR SPEED GET END////////

/*
	��ͬϵͳ����״̬�Ĵ���.
*/

void status_deal(void)
{
	static uint8_t last_run_state = 0xff;
	static uint8_t flash = 1;
	uint8_t level;

	//rcv_data2level();
	if(last_run_state != run_state)
		LPC11xx_print("run_state", run_state, 1);

	if(run_state == S_NORMAL || run_state == S_INIT_ING || run_state == S_RESET_ING){
		++uart_status;
		if(uart_status >= 5){  // ���ڽ��յ���Ϣʱ���uart_status��0, ����ϵͳ��������ʱuart_status������>3
			LPC11xx_print("over time", 0, 1);
			run_state = S_ABNORMAL;
		}
	}
	
	//��/�ػ�, ��ʼ��, �ָ��������ù����в����յ�Դ�����ж�, ����
	if(run_state == S_ON_ING || run_state == S_OFF_ING || run_state == S_INIT_ING 
		|| run_state == S_RESET_ING ||run_state == S_SYS_OFF_DONE){
		flash = 1;
		if(IO_int_is_on(GPIO_GRP3, 4))
			IO_int_disable(GPIO_GRP3, 4);
	} else {    //�������� �ػ���� �쳣��� ���հ����ж�, �Ʋ���
		flash = 0;
		if(!IO_int_is_on(GPIO_GRP3, 4))
			IO_int_enable(GPIO_GRP3, 4, 0);
	}

	if(run_state == S_SYS_OFF_DONE && last_run_state != run_state){
		task_unregister(snd_poweroff);
		task_register(delay_poweroff, 1, 500);
	}

	if(run_state != last_run_state || flash){
		if(run_state == S_ON_ING || run_state == S_12V_OFF_DONE || run_state == S_SYS_OFF_DONE){ //��/�ػ��׶� ������
			level = gpio_get_value(GPIO_GRP0, 6);
			//6, 7, 10Ϊ����
			gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, !level);
			gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, !level);
			gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, !level);
			
			//8, 9 Ϊ�Ƶ�
			gpio_set_value(GPIO_GRP0, 8, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, LEVEL_LOW);
		} else if(run_state == S_NORMAL) {  //�������н׶� ���Ƴ���
			//6, 7, 10Ϊ����
			gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, LEVEL_HIGH);
			gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, LEVEL_HIGH);
			gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, LEVEL_HIGH);
			
			//8, 9 Ϊ�Ƶ�
			gpio_set_value(GPIO_GRP0, 8, PIN_OUTPUT, 0);
			gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, 0);
			
		} else if(run_state == S_INIT_ING) {  //��ʼ���׶�  ���Ƶƽ�����
			level = gpio_get_value(GPIO_GRP0, 6); 
			//6, 7, 10Ϊ����
			gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, !level);
			gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, !level);
			gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, !level);
			
			//8, 9 Ϊ�Ƶ�
			gpio_set_value(GPIO_GRP0, 8, PIN_OUTPUT, level);
			gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, level);
			
		} else if(run_state == S_12V_OFF_DONE) {  //�ػ����  ����
			//6, 7, 10Ϊ����
			gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, LEVEL_LOW);
			
			//8, 9 Ϊ�Ƶ�
			gpio_set_value(GPIO_GRP0, 8, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, LEVEL_LOW);
		} else if(run_state == S_ABNORMAL) {  //�����쳣   �ƵƳ���
			//6, 7, 10Ϊ����
			gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, LEVEL_LOW);
			
			//8, 9 Ϊ�Ƶ�
			gpio_set_value(GPIO_GRP0, 8, PIN_OUTPUT, LEVEL_HIGH);
			gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, LEVEL_HIGH);
		} else if(run_state == S_RESET_ING) { //�ָ���������, �Ƶ�����
			level = gpio_get_value(GPIO_GRP0, 8);
			//6, 7, 10Ϊ����
			gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, LEVEL_LOW);
			
			//8, 9 Ϊ�Ƶ�
			gpio_set_value(GPIO_GRP0, 8, PIN_OUTPUT, !level);
			gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, !level);
		}
	}
	last_run_state = run_state;
}

void speaker_ctrl(void)
{

	if(gpio_get_value(GPIO_GRP1, 8)){
		gpio_set_value(GPIO_GRP1, 8, PIN_OUTPUT, LEVEL_LOW);
		task_unregister(speaker_ctrl);
	}else{
		gpio_set_value(GPIO_GRP1, 8, PIN_OUTPUT, LEVEL_HIGH);
	}
}


