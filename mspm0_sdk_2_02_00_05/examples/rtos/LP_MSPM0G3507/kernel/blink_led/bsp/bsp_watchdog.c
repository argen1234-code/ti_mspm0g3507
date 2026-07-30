#include "bsp_watchdog.h"

#include "ti_msp_dl_config.h"

void bsp_watchdog_init(void)
{
    /* Keep source-level debugging usable while the CPU is halted. */
    DL_WWDT_setCoreHaltBehavior(WWDT0_INST, DL_WWDT_CORE_HALT_STOP);

    /* Restart the 4-second period after board initialization completes. */
    DL_WWDT_restart(WWDT0_INST);
}

void bsp_watchdog_feed(void)
{
    DL_WWDT_restart(WWDT0_INST);
}

bool bsp_watchdog_is_running(void)
{
    return DL_WWDT_isRunning(WWDT0_INST);
}
