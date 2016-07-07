#ifndef __SYS_CFG_H
#define __SYS_CFG_H
#include "LPC11xx.h"

#define AHBCLKCTRL_SYS	0
#define AHBCLKCTRL_ROM	1
#define AHBCLKCTRL_RAM	2
#define AHBCLKCTRL_FLASH1	3
#define AHBCLKCTRL_FLASH2	4
#define AHBCLKCTRL_I2C	5
#define AHBCLKCTRL_GPIO	6
#define AHBCLKCTRL_CT16B0	7
#define AHBCLKCTRL_CT16B1	8
#define AHBCLKCTRL_CT32B0	9
#define AHBCLKCTRL_CT32B1	10
#define AHBCLKCTRL_SSP0	11
#define AHBCLKCTRL_UART	12
#define AHBCLKCTRL_ADC	13
									//14 ±£¡Ù
#define AHBCLKCTRL_WDT	15
#define AHBCLKCTRL_IOCON	16
									//17 ±£¡Ù
#define AHBCLKCTRL_SSP1	18

extern uint32_t SystemAHBFrequency;

void peripherals_clk_switch(uint8_t peripherals, uint8_t on_off);

void SYSAHBCLK_init(void);

#endif
