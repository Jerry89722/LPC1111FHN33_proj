/*
	
*/


#ifndef __IAP_H
#define __IAP_H

//#define IAP_LOCATION 0x1fff 1ff0
#define IAP_LOCATION 0x1FFF1FF1

typedef void (*IAP) (unsigned int [] , unsigned int []);

//void iap_init(void);
void get_devid(void);

#endif //__IAP_H
