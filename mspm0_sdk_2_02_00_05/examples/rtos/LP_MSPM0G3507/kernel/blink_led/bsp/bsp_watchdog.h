#ifndef BSP_WATCHDOG_H
#define BSP_WATCHDOG_H

#include <stdbool.h>

/* WWDT0 is configured and started by SYSCFG_DL_init(). */
void bsp_watchdog_init(void);
void bsp_watchdog_feed(void);
bool bsp_watchdog_is_running(void);

#endif
