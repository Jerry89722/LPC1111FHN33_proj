#include "LPC11xx.h"
#include "string.h"
#include "sys_cfg.h"
#include "uart.h"
#include "gpio.h"
#include "task.h"

#include "debug.h"

static uint8_t rcv_data = 50; //���ڴ�Ŵ�����оƬ���յ���ֵ
uint8_t uart_status = 0;   //����ָʾ ��Ƭ����ϵͳ����ͨѶ״̬, 0����ͨѶ, >3δ����ͨѶ

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

static const uint8_t dataframe[4] = {0x0, 0xef, 0, 0xee}; // ����֡�ṹdataframe[2]Ϊ����
void UART_IRQHandler(void)
{
	static uint8_t buf[4] = {0};
	static uint8_t i = 0;
	uart_status = 0;  // �����յ��ж���0, ��ʾͨѶ����
	//LPC11xx_print("i: ", i, 1);
	
	
	if(((LPC_UART->IIR>>1) & 7) == 2)
	{
		while(LPC_UART->LSR & LSR_RDR){
			buf[i] = LPC_UART->RBR;
			
			//LPC11xx_print("buf ", buf[i], 1);
			//���һ����Э���ֹ��������ʱ�ĸ����ź�, ����֡ 0x0 0xef data 0xee
			if(i == 2){
				if(buf[i-1] > 130 && buf[i-1] < 0xe0)
					i = 0;
				++i;
				return;
			}
			if(buf[i] != dataframe[i]){
				i = 0;
				return;
			}
			if(++i >= 4){
				i = 0;
				rcv_data = buf[2];
			}else{
				return ;
			}
		}
		LPC11xx_print("receive ", rcv_data, 1);
		if(rcv_data == 0)
			return ;
		else if(rcv_data > 0 && rcv_data <120)
			run_state = S_NORMAL;
		else if(rcv_data >= 120 && rcv_data < 0xe0)
			run_state = S_ABNORMAL;
		else if(rcv_data == 0xff)
			run_state = S_SYS_OFF_DONE;
		
		else if(rcv_data == 0xfe)
			run_state = S_INIT_ING;
		else if(rcv_data == 0xfd)
			run_state = S_RESET_ING;
		else
			run_state = S_ABNORMAL;
	}
	
	return;
}

#ifdef DEBUG 
uint8_t rcv_data2level(void)
{
	if(rcv_data <= 10)
		return rcv_data;
	else
		return 0xff;
}

#else
//run_state
//0 ��������
//1 ��������
//2 ��ʼ������
//3 �ػ����� 
//4 �ػ����
//5 �¶ȹ���, �쳣
//6 �ָ���������
uint8_t rcv_data2level(void)
{
	uint8_t level = 0;  //���ڷ��ط���level
	
	if(rcv_data < 50 && rcv_data > 0)
		level = 1;
	else if(rcv_data >= 50 && rcv_data < 80)
		level = 2;
	else if(rcv_data >= 80 && rcv_data < 100)
		level = 3;
	else
		level = 4;
	
	return level;
}
#endif
void rcv_data_reset(void)
{
	rcv_data = 0;
}

