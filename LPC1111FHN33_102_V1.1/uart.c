#include "LPC11xx.h"
#include "string.h"
#include "sys_cfg.h"
#include "uart.h"

static uint8_t rcv_data = 0;//���ڴ�Ŵ�����оƬ���յ���ֵ, linux�ػ�ǰ����0xff

/*
	uart_init();
	����: uart���ܳ�ʼ��, ������Ϊ115200
	����: ��(Ŀǰֻʵ��115200�����ʵ�)
	����ֵ: ��
*/

void uart_init(uint32_t baudrate) 
{
  //uint32_t regVal;

	//UART I/O����ΪRX����
	peripherals_clk_switch(AHBCLKCTRL_IOCON, 1);
	LPC_IOCON->PIO1_6 &= ~7;   
  LPC_IOCON->PIO1_6 |= 1;
  
	//UART I/O����ΪTX����
  LPC_IOCON->PIO1_7 &= ~7;	
  LPC_IOCON->PIO1_7 |= 1;
  peripherals_clk_switch(AHBCLKCTRL_IOCON, 0);
	
	//UART��Ƶ��Ϊ1, uartʱ��Ƶ��Ϊ12MHz
	LPC_SYSCON->UARTCLKDIV &= ~(0xff);
	LPC_SYSCON->UARTCLKDIV |= 1;

  LPC_UART->LCR |= 1<<7; //ʹ�ܶ��������ķ���
	
	//���ò����ʵ�ֵ
	switch(baudrate)
	{
	case 2400:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 0xd0;
		
		//��DLL DLM���ʹ�ò���ָ���Ĳ�����
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 1;
		LPC_UART->FDR |= 2<<4;
		break;
	case 4800:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 0x68;
		
		//��DLL DLM���ʹ�ò���ָ���Ĳ�����
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 1;
		LPC_UART->FDR |= 2<<4;
		break;
		case 9600:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 0x34;
		
		//��DLL DLM���ʹ�ò���ָ���Ĳ�����
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 1;
		LPC_UART->FDR |= 2<<4;
		break;
	case 19200:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 0x1a;
		
		//��DLL DLM���ʹ�ò���ָ���Ĳ�����
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 1;
		LPC_UART->FDR |= 2<<4;
		break;
		case 38400:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 0xd;
		
		//��DLL DLM���ʹ�ò���ָ���Ĳ�����
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 1;
		LPC_UART->FDR |= 2<<4;
		break;
	case 56000:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 8;
		
		//��DLL DLM���ʹ�ò���ָ���Ĳ�����
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 2;
		LPC_UART->FDR |= 3<<4;
		break;
	case 57600:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 0x8;
		
		//��DLL DLM���ʹ�ò���ָ���Ĳ�����
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 5;
		LPC_UART->FDR |= 8<<4;
		break;
		case 115200:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 4;
		
		//��DLL DLM���ʹ�ò���ָ���Ĳ�����
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 5;
		LPC_UART->FDR |= 8<<4;
		break;
	case 128000:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 4;
		
		//��DLL DLM���ʹ�ò���ָ���Ĳ�����
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 6;
		LPC_UART->FDR |= 13<<4;
		break;		
	}
	
	LPC_UART->LCR &= ~(1<<7); //���ܶ��������ķ���
	
  LPC_UART->FCR |= 7;//ʹ��FIFO, ������
	LPC_UART->FCR &= ~(3<<6); //FIFO�����жϵ�����Ϊ1�ֽڴ���
	
	//��������λΪ8, ֹͣλ1, ��У��
	LPC_UART->LCR &= ~0xf;	
	LPC_UART->LCR |= 3;

  //regVal = LPC_UART->LSR; //��LSR�������״̬�Ĵ���
	LPC_UART->LSR;  //��LSR�������״̬�Ĵ���, -------------
	//��շ�����صļĴ���
  while (( LPC_UART->LSR & (LSR_THRE|LSR_TEMT)) != (LSR_THRE|LSR_TEMT) );
  //��ս�����صļĴ���
	while ( LPC_UART->LSR & LSR_RDR )
  {
		LPC_UART->RBR;	// ��ȡRBR�����RX��FIFO, -------------
		//regVal = LPC_UART->RBR;
  }
  LPC_UART->IER = IER_RBR | IER_RLS;	//�����÷����ж�
  NVIC_EnableIRQ(UART_IRQn); //����UART���ж�
}


/*
	���������ݰ�ʮ����λ����ɵ������ִ�ŵ�������, ���������ݵ�λ��
*/
static uint8_t num[10]= {0};
static uint8_t get_num_bit(uint32_t value)
{
	uint8_t i = 0;

	while(1)
	{
		num[i++] = value % 10;
		value = value / 10;
		if(value == 0)
			break;
	}
	return i;
}

/*
	LPC11xx_print()
	����: ��ӡ�ַ���, ���ͱ���, ����
	����:
		const char* p_str: Ҫ��ӡ�ַ����ĵ�ַ.
		int32_t value: Ҫ��ӡ�ı���ֵ
		uint8_t newline: Ϊ1����, Ϊ0������
*/
void LPC11xx_print(const char* p_str, int32_t value, uint8_t newline)
{
	uint8_t i = 0;

	if(p_str == NULL)
		return;
	while(p_str[i++] != '\0')
		LPC_UART->THR = p_str[i-1];
	if(value < 0) 
	{
		value = 0 - value;
		LPC_UART->THR = '-' + '0';
	}
	if(value == 0) 
	{
		goto newl;
	}
	
	i = get_num_bit(value);
	for( ; i > 0; --i)
	{
		LPC_UART->THR = num[i-1] + '0';
	}
	
newl:
	if(newline)
	{
		LPC_UART->THR = '\r';
		LPC_UART->THR = '\n';
	}
	while(!(LPC_UART->LSR & LSR_TEMT) || !(LPC_UART->LSR & LSR_TEMT));
}

/*
	UART_IRQHandler()
	����:uart�жϽ���uart�յ���ֵ, ֻ�������һ���ֽ�
	����: ��
	����ֵ: ��
*/
void UART_IRQHandler(void)
{
	if(((LPC_UART->IIR>>1) & 7) == 2)
	{
		while(LPC_UART->LSR & LSR_RDR)
			rcv_data = LPC_UART->RBR;
	}
	//LPC11xx_print("receive ", rcv_data, 1);
	return;
}

uint8_t rcv_data_translate(void)
{
	uint8_t res;
	if(rcv_data < 50)
		res = 1;
	else if(rcv_data >= 50 && rcv_data < 80)
		res = 2;
	else if(rcv_data >= 80 && rcv_data < 100)
		res = 3;
	else if(rcv_data >= 100 && rcv_data < 254)
		res = 4;
	else if(rcv_data == 0xff){
		//rcv_data = 0;
		res = 0xff;
	}
	return res;
}

void rvc_data_reset(void)
{
	rcv_data = 0;
}
