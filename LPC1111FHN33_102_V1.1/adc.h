#ifndef __ADC_H
#define __ADC_H

#include "LPC11xx.h"

#define Vtemp(v)		((3300000/1023)*v)    //v 为adc寄存器数据位中读取到的值
#define get_temp(v)		(((Vtemp(v)) * 100) - 50000000)  //得到温度值*1000000

void adc_config(void);

void adc_init(void);

void ADC_IRQHandler(void);

void adc_start(void);

#endif
