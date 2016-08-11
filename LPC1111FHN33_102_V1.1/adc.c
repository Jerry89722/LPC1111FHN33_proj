#include "LPC11xx.h"
#include "sys_cfg.h"
#include "uart.h"
#include "adc.h"

uint32_t temper; //���ڴ洢temp�������õ��� �¶�ֵ*1000000

void adc_config(void)
{
	peripherals_clk_switch(AHBCLKCTRL_IOCON, 1);
	//adc�ܽ�����
	LPC_IOCON->R_PIO1_1 &= ~7;
	LPC_IOCON->R_PIO1_1 |= 2;
	LPC_IOCON->R_PIO1_1 &= ~(1<<7);
	peripherals_clk_switch(AHBCLKCTRL_IOCON, 0);
}
void adc_init(void)
{
	//NVIC_DisableIRQ(ADC_IRQn);
	LPC_SYSCON->PDRUNCFG &= ~(1<<4);  //adcģ���ϵ�
	adc_config(); //R_PIO1_1����Ϊģ����������ܽ�
	
	peripherals_clk_switch(AHBCLKCTRL_ADC, 1); //��adcģ��ʱ��
	
	LPC_ADC->INTEN |= 1<<2;
	LPC_ADC->INTEN |= 1<<8;
	LPC_ADC->INTEN &= ~(1<<8);
	LPC_ADC->CR = 1 << 2 //ѡadc2 
							| 3 << 8; //clkdiv = 3 , ��ƵƵ��Ϊ3mhz BIT8~15
										   // �������ģʽ bit16
										   //11��ʱ��, 10λ���� bit17~19
							//| 1 << 24;		 //start, ��������ת��
							//| 1 << 27; //�½�������ת��, start����ʱ��Ч
	//LPC_ADC->DR[2] ; //�˼Ĵ�����6~15λΪadc��� 31λΪ��ɱ�ʶλ
	NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_IRQHandler(void)
{
	uint16_t tmp;
	LPC_ADC->CR &= ~(1<<24);
	
	tmp = (LPC_ADC->DR[2] >> 6) & 0x3ff;
	temper = get_temp(tmp); //temper = ʵ���¶�ֵ*1000000
	// LPC11xx_print("temperature = ", temper/1000000, 0);
	//LPC11xx_print(".", (temper%1000000)/10000, 1);
}

void adc_start(void)
{
	LPC_ADC->CR |= 1<<24;//������ʼת��
	//LPC11xx_print("3", 0, 1);
}
