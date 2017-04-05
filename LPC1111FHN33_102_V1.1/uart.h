#ifndef __UART_H
#define __UART_H

#include "LPC11xx.h"
//��״̬�Ĵ���
#define LSR_RDR         (0x01<<0)
#define LSR_OE          (0x01<<1)
#define LSR_PE          (0x01<<2)
#define LSR_FE          (0x01<<3)
#define LSR_BI          (0x01<<4)
#define LSR_THRE        (0x01<<5)
#define LSR_TEMT        (0x01<<6)
#define LSR_RXFE        (0x01<<7)

//�жϿ���λ
#define IER_RBR         (0x01<<0)
#define IER_THRE        (0x01<<1)
#define IER_RLS         (0x01<<2)

extern uint16_t sig_mask;
extern uint8_t uart_status;

void uart_init(uint32_t baudrate); 
void LPC11xx_print(const char* p_str, int32_t value, uint8_t newline);
void UART_IRQHandler(void);

uint8_t rcv_data2level(void);
void rcv_data_reset(void);
void snd_poweroff(void);

#endif
