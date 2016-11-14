#ifndef __GPIO_H
#define __GPIO_H
#include "LPC11xx.h"

//gpio group
#define GPIO_GRP0	0
#define GPIO_GRP1	1
#define GPIO_GRP2	2
#define GPIO_GRP3	3

//level
#define LEVEL_HIGH	1
#define LEVEL_LOW		0

//direction
#define PIN_OUTPUT	1
#define PIN_INPUT		0

//系统运行状态
//0 开机过程
//1 正常运行
//2 初始化过程
//3 关机过程 
//4 关机完成
//5 温度过高, 异常
//6 恢复出厂设置
#define S_ON_ING   			0			//私有云开机过程
#define S_NORMAL  			1			//私有正常运行
#define S_INIT_ING  		2			//私有云初始化过程
#define S_OFF_ING  			3			//lpc1111延迟关机过程
#define S_12V_OFF_DONE  4			//cpu 12V供电关闭
#define S_ABNORMAL  		5			//异常
#define S_RESET_ING 		6			//重置私有云过程中
#define S_SYS_OFF_DONE 	7    	//主系统关闭完成

extern uint32_t rotation;
extern uint8_t run_state;  //用于记录单片机系统的运行状态的全局变量

void gpio_set_value(uint8_t PIOx, uint32_t gpio_num, uint8_t direction, uint8_t level);

uint8_t gpio_get_value(uint8_t PIOx, uint32_t gpio_num);

void power_12v_on(void);

void power_12v_ctrl(void);

void indicator_init(void);

void key_init(void);

void gpio_pwm_init(void);

void speed_gpio_init(void);

void status_deal(void);

void long_press_chk(void);

void delay_poweroff(void);
//void pwm_init(void);
#endif
