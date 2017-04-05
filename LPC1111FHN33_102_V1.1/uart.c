#include "LPC11xx.h"
#include "string.h"
#include "sys_cfg.h"
#include "uart.h"
#include "gpio.h"
#include "task.h"

#include "debug.h"

static uint8_t rcv_data = 50; //用于存放从主控芯片接收到的值
uint8_t uart_status = 0;   //用于指示 单片机与系统串口通讯状态, 0正常通讯, >3未正常通讯

/*
	uart_init();
	功能: uart功能初始化, 波特率为115200
	参数: 无(目前只实现115200波特率的)
	返回值: 无
*/

void uart_init(uint32_t baudrate) 
{
  //uint32_t regVal;

	//UART I/O配置为RX功能
	peripherals_clk_switch(AHBCLKCTRL_IOCON, 1);
	LPC_IOCON->PIO1_6 &= ~7;   
  LPC_IOCON->PIO1_6 |= 1;
  
	//UART I/O配置为TX功能
  LPC_IOCON->PIO1_7 &= ~7;
  LPC_IOCON->PIO1_7 |= 1;
  peripherals_clk_switch(AHBCLKCTRL_IOCON, 0);
	
	//UART分频设为1, uart时钟频率为12MHz
	LPC_SYSCON->UARTCLKDIV &= ~(0xff);
	LPC_SYSCON->UARTCLKDIV |= 1;

  LPC_UART->LCR |= 1<<7; //使能对锁存器的访问
	
	//设置波特率的值
	switch(baudrate)
	{
	case 2400:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 0xd0;
		
		//与DLL DLM配合使用产生指定的波特率
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 1;
		LPC_UART->FDR |= 2<<4;
		break;
	case 4800:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 0x68;
		
		//与DLL DLM配合使用产生指定的波特率
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 1;
		LPC_UART->FDR |= 2<<4;
		break;
		case 9600:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 0x34;
		
		//与DLL DLM配合使用产生指定的波特率
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 1;
		LPC_UART->FDR |= 2<<4;
		break;
	case 19200:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 0x1a;
		
		//与DLL DLM配合使用产生指定的波特率
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 1;
		LPC_UART->FDR |= 2<<4;
		break;
	case 38400:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 0xd;
		
		//与DLL DLM配合使用产生指定的波特率
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 1;
		LPC_UART->FDR |= 2<<4;
		break;
	case 56000:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 8;
		
		//与DLL DLM配合使用产生指定的波特率
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 2;
		LPC_UART->FDR |= 3<<4;
		break;
	case 57600:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 0x8;
		
		//与DLL DLM配合使用产生指定的波特率
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 5;
		LPC_UART->FDR |= 8<<4;
		break;
	case 115200:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 4;
		
		//与DLL DLM配合使用产生指定的波特率
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 5;
		LPC_UART->FDR |= 8<<4;
		break;
	case 128000:
		LPC_UART->DLM &= ~0xff;
		LPC_UART->DLL &= ~0xff;
		LPC_UART->DLL |= 4;
		
		//与DLL DLM配合使用产生指定的波特率
		LPC_UART->FDR &= ~0xff;
		LPC_UART->FDR |= 6;
		LPC_UART->FDR |= 13<<4;
		break;		
	}
	
	LPC_UART->LCR &= ~(1<<7); //禁能对锁存器的访问
	
  LPC_UART->FCR |= 7;//使能FIFO, 并清零
	LPC_UART->FCR &= ~(3<<6); //FIFO激活中断的条件为1字节触发
	
	//设置数据位为8, 停止位1, 无校验
	LPC_UART->LCR &= ~0xf;	
	LPC_UART->LCR |= 3;

  //regVal = LPC_UART->LSR; //读LSR以清空线状态寄存器
	LPC_UART->LSR;  //读LSR以清空线状态寄存器, -------------
	
	//清空发送相关的寄存器
  while (( LPC_UART->LSR & (LSR_THRE|LSR_TEMT)) != (LSR_THRE|LSR_TEMT) );
  //清空接收相关的寄存器
	while ( LPC_UART->LSR & LSR_RDR )
  {
		LPC_UART->RBR;	// 读取RBR以清空RX的FIFO, -------------
		//regVal = LPC_UART->RBR;
  }
  LPC_UART->IER = IER_RBR | IER_RLS;	//不设置发送中断
  NVIC_EnableIRQ(UART_IRQn); //开启UART的中断
}


/*
	将整型数据按十进制位分离成单个数字存放到数组中, 并返回数据的位数
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
	功能: 打印字符串, 整型变量, 或换行
	参数:
		const char* p_str: 要打印字符串的地址.
		int32_t value: 要打印的变量值
		uint8_t newline: 为1换行, 为0不换行
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
	功能:uart中断接收uart收到的值, 只保存最后一个字节
	参数: 无
	返回值: 无
*/

static const uint8_t dataframe[4] = {0x0, 0xef, 0, 0xee}; // 数据帧结构dataframe[2]为数据
void UART_IRQHandler(void)
{
	static uint8_t buf[4] = {0};
	static uint8_t i = 0;
	uart_status = 0;  // 串口收到中断置0, 表示通讯正常
	//LPC11xx_print("i: ", i, 1);
	
	
	if(((LPC_UART->IIR>>1) & 7) == 2)
	{
		while(LPC_UART->LSR & LSR_RDR){
			buf[i] = LPC_UART->RBR;
			
			//LPC11xx_print("buf ", buf[i], 1);
			//添加一个简单协议防止接收数据时的干扰信号, 数据帧 0x0 0xef data 0xee
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
//0 开机过程
//1 正常运行
//2 初始化过程
//3 关机过程 
//4 关机完成
//5 温度过高, 异常
//6 恢复出厂设置
uint8_t rcv_data2level(void)
{
	uint8_t level = 0;  //用于返回风速level
	
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

