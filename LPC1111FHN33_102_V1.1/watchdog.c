#include "LPC11xx.h"
#include "watchdog.h"
#include "uart.h"

volatile uint32_t wdt_counter;

void watchdog_feed(void)
{
	LPC_WDT->FEED = 0xAA;
	LPC_WDT->FEED = 0x55;
}

//wdt_clk 62500Hz
void watchdog_init(void)
{
	//uint32_t i;
	LPC_SYSCON->PDRUNCFG &= ~(1<<6);//看门狗模块上电
	//LPC_SYSCON->WDTOSCCTRL = 1<<5;
	LPC_SYSCON->WDTCLKSEL = 2; //看门狗时钟源选择看门狗振荡器
	LPC_SYSCON->WDTCLKUEN = 1;
	LPC_SYSCON->WDTCLKUEN = 0;
	LPC_SYSCON->WDTCLKUEN = 1;
	while ( !(LPC_SYSCON->WDTCLKUEN & 1));
	
	LPC_SYSCON->WDTCLKDIV = 3;  //分频到约100KHz
	
	LPC_WDT->TC = 2500; //约1s
	
	//LPC_WDT->MOD &= ~(7); //清空低3位
	LPC_WDT->MOD |= 2; // 0 位为0 禁能看门狗
										 // 1位为1时看门狗超时后会引起复位
										 // 2位看门狗超时置位, 由软件清零
	
	LPC_WDT->MOD |= 1; //0位置1 使能看门狗
	
	watchdog_feed();//第一次喂狗, 开启看门狗
	NVIC_EnableIRQ(WDT_IRQn);
	//for (i = 0; i < 0x80000; i++);
}

void WDT_IRQHandler(void)
{
	LPC_WDT->MOD &= ~(1<<2);
	LPC11xx_print("restart", 0, 1);
	NVIC_SystemReset();
}

