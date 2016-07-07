#include "LPC11xx.h"
#include "sys_cfg.h"

uint32_t SystemAHBFrequency = 12000000UL;
/*
	peripherals_clk_switch()
	����: ����ahb������ʱ�ӵ�����
	����: 
		uint8_t peripherals: ����ʱ�ӵı��, 0��ΪSYS, ������д, ����������궨��
		uint8_t on_off: ����ʱ�ӵĿ���, 1Ϊon, 0Ϊoff
	����ֵ: ��
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
	����: �����й�����һֱ��Ҫ���е�ʱ��, rom, ram, flash1, flash2, gpio, UART, WDT
	����: ��
	����ֵ: ��
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
