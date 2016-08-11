#include "LPC11xx.h"
#include "uart.h"
#include "sys_cfg.h"
#include "timer.h"
#include "task.h"
#include "gpio.h"

static uint8_t level = 1;
//static uint32_t speed_arr[3] = {600, 360, 120};
/*
timer使用情况说明:
	timer16_0, 测量风扇转速
	timer16_1, 用于task时间片管理定时器
	timer32_0, 用于产生模拟产生pwm,频率20khz
	timer32_1, 用于精确的delay系列函数, 使用了MR3的匹配
*/

/*
	timer_init();
	功能: 硬件定时器初始化
	参数: 
		LPC_TMR_TypeDef* p_timer: 定时器寄存器地址
	返回值: 无	
*/

void timer_init(LPC_TMR_TypeDef* p_timer)
{
	//开启相应的定时器模块时钟, 设置预分频寄存器, 判断定时时间是否合法
	switch((uint32_t)p_timer)
	{
	case (uint32_t)LPC_TMR32B0:
		peripherals_clk_switch(AHBCLKCTRL_CT32B0, 1);
		p_timer->PR = 11;  // 1 us
		p_timer->IR |= 0x1f; //中断复位
		p_timer->MCR |= 3<<6; //MR2匹配时触发中断, 复位TC
		
		//使能计数器并保持复位
		p_timer->TCR &= ~3;
		p_timer->TCR |= 2;
		return ;
//		break;
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
	
	p_timer->IR |= 0x1f; //中断复位
	p_timer->MCR |= 3<<9; //MR3匹配时触发中断, 复位TC
	
	//使能计数器并保持复位
	p_timer->TCR &= ~3;
	p_timer->TCR |= 2;
}

/*
	timer_start();
	功能: 启动定时器
	参数: 
		LPC_TMR_TypeDef* timer: 定时器寄存器地址
		uint8_t loop: 循环/非循环定时, 1循环, 0非循环
		uint32_t interval: 超时时间, 位定时器时间单位分别为us/ms
	返回值: 无
*/
void timer_start(LPC_TMR_TypeDef* p_timer, uint8_t loop, uint32_t interval)
{
	if(loop > 1)
	{
		LPC11xx_print("loop invalid", 0, 1);
		return;
	}
	
	//禁能计数器, 并保持复位
	p_timer->TCR &= ~3;
	p_timer->TCR |= 2;
	
	if(loop) //一直循环执行
		p_timer->MCR &= ~(1<<11); 
	else //执行一次
		p_timer->MCR |= 1<<11;//3匹配后停止pc和tc, 禁能计数器

	//设置定时时间, 开启定时器中断服务函数
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
	p_timer->MR3 = interval; //单位:1ms/us
	//计数器使能, 停止复位动作开始计时
	p_timer->TCR &= ~3;
	p_timer->TCR |= 1;
}

/*
	中断服务函数
*/
void TIMER32_0_IRQHandler(void)
{
	/*
	LPC11xx_print("timer TIMER32_0_IRQ", 0, 1);
	uint8_t pwm_state = gpio_get_value(GPIO_GRP0, 3);
	
	LPC_TMR32B0->IR |= 1<<3; //中断复位
	
	gpio_set_value(GPIO_GRP0, 3, PIN_OUTPUT, !pwm_state);
	
	if(pwm_state){
		timer_start(LPC_TMR32B0, 0, speed_arr[level - 1]);
	} else {
		timer_start(LPC_TMR32B0, 0, period - speed_arr[level - 1]);
	}
	
	LPC_TMR32B0->IR |= 1<<3;
	*/
}

void TIMER32_1_IRQHandler(void)
{
	LPC_TMR32B1->IR |= 1<<3;//中断复位
	LPC11xx_print("32_1", 0, 1);
}


//用于控制模拟pwm控制风扇转速
void TIMER16_0_IRQHandler(void)
{
	LPC_TMR16B0->IR |= 1<<3;//中断复位
	LPC11xx_print("rotation = ", rotation, 1);
	rotation = 0;
}

//1ms产生一次中断
void TIMER16_1_IRQHandler(void)
{
	uint8_t i;
	LPC_TMR16B1->IR |= 1<<3;//中断复位
	
	for(i = 0; i < TASK_MAX_NUM; ++i)
	{
		if(task[i].p_calbak)
		{
			if((--task[i].rest_time) == 0) //当休眠时间为0是激活运行状态, 并重新设置间隔
			{
				task[i].rest_time = task[i].interval;
				task[i].run = 1;
			}
		}
	}
}

/*
	延时函数, 使用32位定时器1
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

#if 1		//用于pwm管脚方法控制pwm
/*
	timer_init()默认开启MR3匹配复位触发中断, pwm不需要中断可以关闭中断
*/

void pwm_init(LPC_TMR_TypeDef* p_timer, __IO uint32_t* addr)
{
	//配置pwm输出的io口
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
		//非法地址, 不可配置为pwm
		break;
	}
	peripherals_clk_switch(AHBCLKCTRL_IOCON, 0);
	
	//配置pwm的参数
	p_timer->MR2 = period;
	p_timer->MR3 = 4*period / 5;
	p_timer->CTCR &= ~3; //定时器模式工作
	p_timer->PWMC |= (1<<3);
	
	p_timer->MCR &= ~(1<<6); //关闭MR2匹配时的中断
	//NVIC_DisableIRQ(TIMER_16_0_IRQn);
}

//pwm_ctrl();
// #define DEBUG
#ifdef DEBUG
void pwm_ctrl(LPC_TMR_TypeDef* p_timer, uint8_t level)
{
	static uint8_t last_level = 0;
	
	level = 0; //将转速调到最低用于测试
	LPC11xx_print("speed level = ", level, 1);
	if(level != last_level)
	{
		p_timer->TCR &= ~3;  //10 计数器禁能, 保持复位
		p_timer->TCR |= 2;
		
		//p_timer->MR3 = 35 - 15 * (level -1); //设置转速 30 15 0
		
		p_timer->MR3 = 50 - level * 5;
		
		p_timer->TCR &= ~3;  //01 计数器使能, 解除复位
		p_timer->TCR |= 1;
		
		last_level = level;
	}
}
void speed_ctrl(void)
{
	uint8_t tmp = rcv_data2level();
	
	if(tmp == 0xff)
		return;
	
	pwm_ctrl(LPC_TMR32B0, tmp);
}
#else
void pwm_ctrl(LPC_TMR_TypeDef* p_timer, uint8_t level)
{
	static uint8_t last_level = 0;
	if(level < 1)
		level = 1;
	else if(level > 3)
		level = 3;
	//LPC11xx_print("speed level = ", level, 1);
	if(level != last_level)
	{
		p_timer->TCR &= ~3;  //10 计数器禁能, 保持复位
		p_timer->TCR |= 2;
		
		p_timer->MR3 = 35 - 15 * (level -1); //设置转速 30 15 0
		//p_timer->MR3 = 325 + 350 * (level - 1); //设置转速 325 675 975
		
		p_timer->TCR &= ~3;  //01 计数器使能, 解除复位
		p_timer->TCR |= 1;
		
		last_level = level;
	}
}

void speed_ctrl(void)
{
	uint8_t tmp = rcv_data2level();
	
	if(tmp == 0xff)
		return;
	else
		level = tmp;
	if(level > 3)
		level = 3;
	
	pwm_ctrl(LPC_TMR32B0, level);
}
#endif
#else  //定时器和普通pio模拟pwm, 控制风扇转速
void speed_ctrl(void)
{
	static uint8_t last_level = 0;
	uint8_t tmp = rcv_data2level();
	if(tmp == 0xff)
		return;
	else
		level = tmp;

	//LPC11xx_print("last_level = ", last_level, 1);

	if(last_level != level)
	{
		last_level = level;
		if(level >= 3){
			gpio_set_value(GPIO_GRP0, 3, PIN_OUTPUT, 1);
		return ;
	}
		gpio_set_value(GPIO_GRP0, 3, PIN_OUTPUT, 0);
		timer_start(LPC_TMR32B0, 0, speed_arr[level - 1]);
	}
}
#endif 

