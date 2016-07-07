#ifndef __WATCHDOG_H 
#define __WATCHDOG_H

extern void WDT_IRQHandler(void);
extern void watchdog_init(void);
extern void watchdog_feed(void);

#endif
