#ifndef __TIMER_H
#define __TIMER_H

#include "LPC11xx.h"

#define PWM_FAN  //使用PWM控制风扇转速时打开, 使用GPIO控制风扇转速时注释此行

#define period		50 		//20khz

void timer_init(LPC_TMR_TypeDef* p_timer);
void timer_start(LPC_TMR_TypeDef* p_timer, uint8_t loop, uint32_t interval);
void TIMER32_0_IRQHandler(void);
void TIMER32_1_IRQHandler(void);
void TIMER16_0_IRQHandler(void);
void TIMER16_1_IRQHandler(void);

void delay_sec(uint32_t duration);

void delay_ms(uint32_t duration);

void delay_us(uint32_t duration);

void pwm_init(LPC_TMR_TypeDef* p_timer, __IO uint32_t* addr);

void pwm_ctrl(LPC_TMR_TypeDef* p_timer, uint8_t level);

void task_timer_init(void);

void speed_ctrl(void);

#endif
