#include "LPC11xx.h"
#include "sys_cfg.h"
#include "uart.h"
#include "adc.h"
uint32_t temper;
void adc_config(void)
{
	peripherals_clk_switch(AHBCLKCTRL_IOCON, 1);
	//adc管脚配置
	LPC_IOCON->R_PIO1_1 &= ~7;
	LPC_IOCON->R_PIO1_1 |= 2;
	LPC_IOCON->R_PIO1_1 &= ~(1<<7);
	peripherals_clk_switch(AHBCLKCTRL_IOCON, 0);
}
void adc_init(void)
{
	//NVIC_DisableIRQ(ADC_IRQn);
	LPC_SYSCON->PDRUNCFG &= ~(1<<4);  //adc模块上电
	adc_config(); //R_PIO1_1配置为模拟量的输入管脚
	
	peripherals_clk_switch(AHBCLKCTRL_ADC, 1); //开adc模块时钟
	
	LPC_ADC->INTEN |= 1<<2;
	LPC_ADC->INTEN |= 1<<8;
	LPC_ADC->INTEN &= ~(1<<8);
	LPC_ADC->CR = 1 << 2 //选adc2 
							| 3 << 8; //clkdiv = 3 , 分频频率为3mhz BIT8~15
										   // 软件控制模式 bit16
										   //11个时钟, 10位精度 bit17~19
							//| 1 << 24;		 //start, 立即启动转换
							//| 1 << 27; //下降沿启动转换, start开启时有效
	//LPC_ADC->DR[2] ;//此寄存器的6~15位为adc结果 31位为完成标识位
	NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_IRQHandler(void)
{
	uint16_t tmp;
	LPC_ADC->CR &= ~(1<<24);
	
	tmp = (LPC_ADC->DR[2] >> 6) & 0x3ff;
	temper = get_temp(tmp); //temper = 实际温度值*1000000
	LPC11xx_print("temperature = ", temper/1000000, 0);
	LPC11xx_print(".", (temper%1000000)/10000, 1);
}

void adc_start(void)
{
	LPC_ADC->CR |= 1<<24;//立即开始转换
	//LPC11xx_print("3", 0, 1);
}
