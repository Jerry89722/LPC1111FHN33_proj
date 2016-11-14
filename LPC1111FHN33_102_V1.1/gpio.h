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

//ϵͳ����״̬
//0 ��������
//1 ��������
//2 ��ʼ������
//3 �ػ����� 
//4 �ػ����
//5 �¶ȹ���, �쳣
//6 �ָ���������
#define S_ON_ING   			0			//˽���ƿ�������
#define S_NORMAL  			1			//˽����������
#define S_INIT_ING  		2			//˽���Ƴ�ʼ������
#define S_OFF_ING  			3			//lpc1111�ӳٹػ�����
#define S_12V_OFF_DONE  4			//cpu 12V����ر�
#define S_ABNORMAL  		5			//�쳣
#define S_RESET_ING 		6			//����˽���ƹ�����
#define S_SYS_OFF_DONE 	7    	//��ϵͳ�ر����

extern uint32_t rotation;
extern uint8_t run_state;  //���ڼ�¼��Ƭ��ϵͳ������״̬��ȫ�ֱ���

void gpio_set_value(uint8_t PIOx, uint32_t gpio_num, uint8_t direction, uint8_t level);

uint8_t gpio_get_value(uint8_t PIOx, uint32_t gpio_num);

void power_12v_on(void);

void power_12v_ctrl(void);

void indicator_init(void);

void key_init(void);

void gpio_pwm_init(void);

void speed_gpio_init(void);

void status_deal(void);

void long_press_chk(void);

void delay_poweroff(void);
//void pwm_init(void);
#endif
