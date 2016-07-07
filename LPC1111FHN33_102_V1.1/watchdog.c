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
	LPC_SYSCON->PDRUNCFG &= ~(1<<6);//���Ź�ģ���ϵ�
	//LPC_SYSCON->WDTOSCCTRL = 1<<5;
	LPC_SYSCON->WDTCLKSEL = 2; //���Ź�ʱ��Դѡ���Ź�����
	LPC_SYSCON->WDTCLKUEN = 1;
	LPC_SYSCON->WDTCLKUEN = 0;
	LPC_SYSCON->WDTCLKUEN = 1;
	while ( !(LPC_SYSCON->WDTCLKUEN & 1));
	
	LPC_SYSCON->WDTCLKDIV = 3;  //��Ƶ��Լ100KHz
	
	LPC_WDT->TC = 2500; //Լ1s
	
	//LPC_WDT->MOD &= ~(7); //��յ�3λ
	LPC_WDT->MOD |= 2; // 0 λΪ0 ���ܿ��Ź�
										 // 1λΪ1ʱ���Ź���ʱ�������λ
										 // 2λ���Ź���ʱ��λ, ���������
	
	LPC_WDT->MOD |= 1; //0λ��1 ʹ�ܿ��Ź�
	
	watchdog_feed();//��һ��ι��, �������Ź�
	NVIC_EnableIRQ(WDT_IRQn);
	//for (i = 0; i < 0x80000; i++);
}

void WDT_IRQHandler(void)
{
	LPC_WDT->MOD &= ~(1<<2);
	LPC11xx_print("restart", 0, 1);
	NVIC_SystemReset();
}

