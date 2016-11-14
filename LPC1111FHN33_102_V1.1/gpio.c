#include "LPC11xx.h"
#include "sys_cfg.h"
#include "gpio.h"
#include "uart.h"
#include "task.h"
#include "timer.h"

uint32_t rotation = 0;

/*
	gpio_init();
	功能: gpio口初始化
	参数: 
	返回值:
*/

void gpio_init(volatile uint32_t* reg_addr)
{
	uint32_t io_addr = (uint32_t)reg_addr;
	if((uint32_t)reg_addr < (uint32_t)(&(LPC_IOCON->PIO2_6)) || 
		(uint32_t)reg_addr > (uint32_t)(&(LPC_IOCON->RXD_LOC)))
	{
		//输入的io口地址不合法
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
功能: 设置gpio的方向和电平
参数:
	uint8_t PIOx:gpio的组号 (见LPC11xx_gpio.h中宏定义)
	uint32_t gpio_num:gpiogpio组内序号 (0~11) 
	uint8_t direction:1输入/输出 (见LPC11xx_gpio.h中宏定义)
	uint8_t level: 高/低电平, 设为输入时,电平设置不生效 (见LPC11xx_gpio.h中宏定义)
	返回值: 无
*/
void gpio_set_value(uint8_t PIOx, uint32_t gpio_num, uint8_t direction, uint8_t level)
{
	LPC_GPIO_TypeDef* port[4] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3};
	
	//方向设置
	if(direction)
		port[PIOx]->DIR |= (1<<gpio_num);
	else 
	{
		port[PIOx]->DIR &= ~(1<<gpio_num);
		return; //输入时直接返回
	}
	
	//电平设置
	if(level)
		port[PIOx]->DATA |= (1<<gpio_num);
	else
		port[PIOx]->DATA &= ~(1<<gpio_num);
}

/*
	gpio_get_value()
	功能: 获取gpio的电平
	参数: 
		uint8_t PIOx: gpio的组号
		uint32_t gpio_num: gpio组内序号
*/
uint8_t gpio_get_value(uint8_t PIOx, uint32_t gpio_num)
{
	LPC_GPIO_TypeDef* port[4] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3};
	
	return (((port[PIOx]->DATA) >> gpio_num) & 1);
}

/*
	IO_int_enable()
	功能: 初始化gpio的中断, 以开启指定管脚的中断
	参数: 
		uint8_t PIOx: gpio的组号
		uint32_t gpio_num: gpio组内序号
		uint8_t trigger_type: 中断的触发方式, 0 为边沿触发, 1为电平触发
*/
static void IO_int_enable(uint8_t PIOx, uint32_t gpio_num, uint8_t trigger_type)
{
	LPC_GPIO_TypeDef* port[4] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3};
	
	if(trigger_type)
	{
		port[PIOx]->IS |= 1 << gpio_num; //电平触发
		port[PIOx]->IEV &= ~(1<<gpio_num);  //高电平触发
	}
	else
	{
		port[PIOx]->IS &= ~(1 << gpio_num);//设为边沿触发
		port[PIOx]->IEV &= ~(1 << gpio_num);//设为上升沿触发
	}
	
	port[PIOx]->IE &= ~(1 << gpio_num);// 屏蔽中断
	port[PIOx]->IC |= 1 << gpio_num;   // 清除中断
	while(port[PIOx]->MIS & (1 << gpio_num));
	port[PIOx]->IE |= 1 << gpio_num;//取消中断屏蔽
	
	NVIC_EnableIRQ(EINT3_IRQn);//EINT0_IRQn~EINT3_IRQn为枚举中连续的项
	NVIC_EnableIRQ(EINT1_IRQn);
	NVIC_EnableIRQ(EINT0_IRQn);
}

static void IO_int_disable(uint8_t PIOx, uint32_t gpio_num)
{
	LPC_GPIO_TypeDef* port[4] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3};
	port[PIOx]->IE &= ~(1 << gpio_num);// 屏蔽中断
	port[PIOx]->IC |= 1 << gpio_num;   // 清除中断
	while(port[PIOx]->MIS & (1 << gpio_num));
}

uint8_t IO_int_is_on(uint8_t PIOx, uint32_t gpio_num)
{
	LPC_GPIO_TypeDef* port[4] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3};
	return (port[PIOx]->IE & (1 << gpio_num));
}

/////////////////////////////////以上为通用函数, 以下为具体项目中调用/////////////////////////////

//打开12v供电
void power_12v_on(void)
{
	//LPC11xx_print("on", 0, 1);
	gpio_init(&(LPC_IOCON->PIO1_9));//设为gpio
	gpio_set_value(GPIO_GRP1, 9, PIN_OUTPUT, LEVEL_LOW);
}

// 12电源开关
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
	gpio_init(&(LPC_IOCON->PIO0_8));
	gpio_init(&(LPC_IOCON->PIO0_9));
	gpio_init(&(LPC_IOCON->SWCLK_PIO0_10));
	
	//6, 7, 9 为蓝灯
	gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, LEVEL_HIGH);
	gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, LEVEL_HIGH);
	gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, LEVEL_HIGH);
	//8, 10 为黄灯
	
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
		
		LPC11xx_print("poweroff done", 0, 1);
		power_12v_off();
		rcv_data_reset();
		//IO_int_enable(GPIO_GRP3, 4, 0);
		run_state = S_12V_OFF_DONE;
	}
}

uint8_t run_state = S_ON_ING;
// 按键的中断服务程序
void PIOINT3_IRQHandler(void)
{
	IO_int_disable(GPIO_GRP3, 4);
	
	//delay_ms(100);
	LPC11xx_print("key int 3_4 get", 0, 1);
	task_register(long_press_chk, 1, 50);
}

// 6820复位按键的中断服务程序
void PIOINT1_IRQHandler(void)
{
	IO_int_disable(GPIO_GRP1, 3);
	
	LPC11xx_print("key int 1_3 get", 0, 1);
	task_unregister(snd_poweroff);
	rcv_data_reset();
	run_state = S_ON_ING;
	gpio_set_value(GPIO_GRP1, 9, PIN_OUTPUT, LEVEL_LOW);
	IO_int_enable(GPIO_GRP1, 3, 0);
}

void long_press_chk(void)
{
	static uint8_t cnt_key = 0;
	//LPC11xx_print("----press----", gpio_get_value(GPIO_GRP3, 4), 1);
	uint8_t i;
	
	if(!gpio_get_value(GPIO_GRP3, 4)){
		if(cnt_key++ > 10){
			cnt_key = 0;
			task_unregister(long_press_chk);
			if(gpio_get_value(GPIO_GRP1, 9)){  //开机
				power_12v_on();
				run_state = S_ON_ING;
				//IO_int_enable(GPIO_GRP3, 4, 0);
			}else{  //关机
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

////// MOTOR SPEED GET START////////
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

////// MOTOR SPEED GET END////////

/*
	不同系统运行状态的处理.
*/

void status_deal(void)
{
	static uint8_t last_run_state = 0xff;
	static uint8_t flash = 1;
	uint8_t level;
	
	//rcv_data2level();
	if(last_run_state != run_state)
		LPC11xx_print("run_state", run_state, 1);
	
	//开/关机, 初始化, 恢复出厂设置过程中不接收电源按键中断, 灯闪
	if(run_state == S_ON_ING || run_state == S_OFF_ING || run_state == S_INIT_ING 
		|| run_state == S_RESET_ING ||run_state == S_SYS_OFF_DONE){
		flash = 1;
		if(IO_int_is_on(GPIO_GRP3, 4))
			IO_int_disable(GPIO_GRP3, 4);
	} else {    //正常运行 关机完成 异常情况 接收按键中断, 灯不闪
		flash = 0;
		if(!IO_int_is_on(GPIO_GRP3, 4))
			IO_int_enable(GPIO_GRP3, 4, 0);
	}

	if(run_state == S_SYS_OFF_DONE && last_run_state != run_state){
		task_unregister(snd_poweroff);
		task_register(delay_poweroff, 1, 500);
	}

	if(run_state != last_run_state || flash){
		if(run_state == S_ON_ING || run_state == S_12V_OFF_DONE || run_state == S_SYS_OFF_DONE){ //开/关机阶段 蓝灯闪
			level = gpio_get_value(GPIO_GRP0, 6);
			//6, 7, 10为蓝灯
			gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, !level);
			gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, !level);
			gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, !level);
			
			//8, 9 为黄灯
			gpio_set_value(GPIO_GRP0, 8, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, LEVEL_LOW);
		} else if(run_state == S_NORMAL) {  //正常运行阶段 蓝灯常亮
			//6, 7, 10为蓝灯
			gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, LEVEL_HIGH);
			gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, LEVEL_HIGH);			
			gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, LEVEL_HIGH);
			
			//8, 9 为黄灯
			gpio_set_value(GPIO_GRP0, 8, PIN_OUTPUT, 0);
			gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, 0);
			
		} else if(run_state == S_INIT_ING) {  //初始化阶段  蓝黄灯交替闪
			level = gpio_get_value(GPIO_GRP0, 6); 
			//6, 7, 10为蓝灯
			gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, !level);
			gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, !level);
			gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, !level);
			
			//8, 9 为黄灯
			gpio_set_value(GPIO_GRP0, 8, PIN_OUTPUT, level);
			gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, level);
			
		} else if(run_state == S_12V_OFF_DONE) {  //关机完成  灯灭
			//6, 7, 10为蓝灯
			gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, LEVEL_LOW);
			
			//8, 9 为黄灯
			gpio_set_value(GPIO_GRP0, 8, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, LEVEL_LOW);
		} else if(run_state == S_ABNORMAL) {  //运行异常   黄灯常亮
			//6, 7, 10为蓝灯
			gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, LEVEL_LOW);
			
			//8, 9 为黄灯
			gpio_set_value(GPIO_GRP0, 8, PIN_OUTPUT, LEVEL_HIGH);
			gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, LEVEL_HIGH);
		} else if(run_state == S_RESET_ING) { //恢复出厂设置, 黄灯闪亮
			level = gpio_get_value(GPIO_GRP0, 8);
			//6, 7, 10为蓝灯
			gpio_set_value(GPIO_GRP0, 6, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 7, PIN_OUTPUT, LEVEL_LOW);
			gpio_set_value(GPIO_GRP0, 10, PIN_OUTPUT, LEVEL_LOW);
			
			//8, 9 为黄灯
			gpio_set_value(GPIO_GRP0, 8, PIN_OUTPUT, !level);
			gpio_set_value(GPIO_GRP0, 9, PIN_OUTPUT, !level);
		}
	}
	last_run_state = run_state;
}

#if 0  //只用于定时器与pwm分开的做法
void pwm_init(void)
{
	gpio_pwm_init();
	timer_init(LPC_TMR32B0);
	timer_start(LPC_TMR32B0, 0, 1);
}
#endif

