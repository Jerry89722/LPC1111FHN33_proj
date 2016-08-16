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

extern uint32_t rotation;
extern uint8_t run_state;  //用于记录单片机系统的运行状态的全局变量

void gpio_set_value(uint8_t PIOx, uint32_t gpio_num, uint8_t direction, uint8_t level);

uint8_t gpio_get_value(uint8_t PIOx, uint32_t gpio_num);

void power_12v_on(void);

void power_12v_ctrl(void);

void indicator_init(void);

void indicator_ctrl(void);

void resetkey_init(void);

void gpio_pwm_init(void);

void speed_gpio_init(void);

void led_ctrl(void);

void long_press_chk(void);

//void pwm_init(void);
#endif
