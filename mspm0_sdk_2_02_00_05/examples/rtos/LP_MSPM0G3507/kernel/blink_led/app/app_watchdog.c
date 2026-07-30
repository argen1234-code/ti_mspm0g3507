#include "app_watchdog.h"

#include <stddef.h>

#include "bsp_watchdog.h"

void app_watchdog_init(app_watchdog_t *watchdog)
{
    if (watchdog == NULL) {
        return;
    }

    watchdog->chassis_heartbeat = 0U;
    watchdog->oled_heartbeat = 0U;
    watchdog->monitor_heartbeat = 0U;
    watchdog->feed_count = 0U;
    watchdog->fault_mask = APP_WATCHDOG_FAULT_CHASSIS |
                           APP_WATCHDOG_FAULT_OLED |
                           APP_WATCHDOG_FAULT_ZDT;
    watchdog->all_tasks_healthy = false;
    watchdog->last_chassis_heartbeat = 0U;
    watchdog->last_oled_heartbeat = 0U;
    watchdog->last_zdt_heartbeat = 0U;

    bsp_watchdog_init();
    watchdog->hardware_running = bsp_watchdog_is_running();
}

void app_watchdog_task_heartbeat(app_watchdog_t *watchdog,
                                 app_watchdog_task_t task)
{
    if (watchdog == NULL) {
        return;
    }

    switch (task) {
    case APP_WATCHDOG_TASK_CHASSIS:
        watchdog->chassis_heartbeat++;
        break;
    case APP_WATCHDOG_TASK_OLED:
        watchdog->oled_heartbeat++;
        break;
    default:
        break;
    }
}

void app_watchdog_monitor(app_watchdog_t *watchdog,
                          uint32_t zdt_heartbeat)
{
    uint32_t fault_mask = 0U;
    uint32_t chassis_heartbeat;
    uint32_t oled_heartbeat;

    if (watchdog == NULL) {
        return;
    }

    chassis_heartbeat = watchdog->chassis_heartbeat;
    oled_heartbeat = watchdog->oled_heartbeat;
    watchdog->monitor_heartbeat++;

    if (chassis_heartbeat == watchdog->last_chassis_heartbeat) {
        fault_mask |= APP_WATCHDOG_FAULT_CHASSIS;
    }
    if (oled_heartbeat == watchdog->last_oled_heartbeat) {
        fault_mask |= APP_WATCHDOG_FAULT_OLED;
    }
    if (zdt_heartbeat == watchdog->last_zdt_heartbeat) {
        fault_mask |= APP_WATCHDOG_FAULT_ZDT;
    }

    watchdog->last_chassis_heartbeat = chassis_heartbeat;
    watchdog->last_oled_heartbeat = oled_heartbeat;
    watchdog->last_zdt_heartbeat = zdt_heartbeat;
    watchdog->fault_mask = fault_mask;
    watchdog->hardware_running = bsp_watchdog_is_running();

    if ((fault_mask == 0U) && watchdog->hardware_running) {
        bsp_watchdog_feed();
        watchdog->feed_count++;
        watchdog->all_tasks_healthy = true;
    } else {
        /* Do not feed; WWDT0 will issue SYSRST after its 4-second period. */
        watchdog->all_tasks_healthy = false;
    }
}
