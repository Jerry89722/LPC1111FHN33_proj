#include "LPC11xx.h"
#include "sys_cfg.h"

uint32_t SystemAHBFrequency = 12000000UL;
/*
	peripherals_clk_switch()
	功能: 设置ahb对外设时钟的设置
	参数: 
		uint8_t peripherals: 外设时钟的编号, 0号为SYS, 不允许写, 参数具体见宏定义
		uint8_t on_off: 外设时钟的开关, 1为on, 0为off
	返回值: 无
*/
void peripherals_clk_switch(uint8_t peripherals, uint8_t on_off)
{
	if(peripherals == AHBCLKCTRL_SYS)
		return;
	if(on_off)
		LPC_SYSCON->SYSAHBCLKCTRL |= 1<<peripherals;
	else
		LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<peripherals);
}

/*
	AHBCLK_init()
	功能: 打开运行过程中一直需要运行的时钟, rom, ram, flash1, flash2, gpio, UART, WDT
	参数: 无
	返回值: 无
*/
void SYSAHBCLK_init(void)
{
	peripherals_clk_switch(AHBCLKCTRL_ROM, 1);
	peripherals_clk_switch(AHBCLKCTRL_RAM, 1);
	peripherals_clk_switch(AHBCLKCTRL_FLASH1, 1);
	peripherals_clk_switch(AHBCLKCTRL_FLASH2, 1);
	peripherals_clk_switch(AHBCLKCTRL_GPIO, 1);
	peripherals_clk_switch(AHBCLKCTRL_UART, 1);
	peripherals_clk_switch(AHBCLKCTRL_WDT, 1);
}
