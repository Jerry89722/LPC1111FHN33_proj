#include "LPC11xx.h"
#include "iap.h"
#include "uart.h"
#include "watchdog.h"
//void iap_entry(unsigned int [] , unsigned int []);
IAP iap_entry = (IAP)IAP_LOCATION;

void get_devid(void)
{
	unsigned int cmd[5], res[5];
	cmd[0] = 53;
	cmd[1] = 0;
	cmd[2] = 0;
	iap_entry(cmd, res);
	LPC11xx_print("res = ", res[0], 1);
	LPC11xx_print("res = ", res[1], 1);
	LPC11xx_print("res = ", res[2], 1);
	cmd[0] = 53;
	cmd[1] = 1;
	cmd[2] = 1;
	iap_entry(cmd, res);
	LPC11xx_print("res = ", res[0], 1);
	LPC11xx_print("res = ", res[1], 1);
	LPC11xx_print("res = ", res[2], 1);
}
//268436192



