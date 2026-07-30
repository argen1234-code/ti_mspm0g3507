#include "app_task_state_machine.h"
#include <limits.h>
#include <stddef.h>

void app_task_state_machine_init(app_task_state_machine_t *machine)
{
    if (machine == NULL) return;

    machine->state = APP_TASK_STATE_IDLE;
    machine->elapsed_time_ms = 0U;
    machine->final_time_ms = 0U;
    machine->start_count = 0U;
    machine->tracking_active = false;
    machine->timer_running = false;
    machine->start_key_previous = false;
    machine->final_time_within_requirement = false;
}

bool app_task_state_machine_update(app_task_state_machine_t *machine,
                                   bool task1_start_key_pressed,
                                   uint32_t elapsed_ms)
{
    bool start_pressed_edge;
    bool started = false;

    if (machine == NULL) return false;

    start_pressed_edge = task1_start_key_pressed &&
                         !machine->start_key_previous;
    machine->start_key_previous = task1_start_key_pressed;

    if (start_pressed_edge &&
        (machine->state != APP_TASK_STATE_TASK1_TRACKING)) {
        machine->state = APP_TASK_STATE_TASK1_TRACKING;
        machine->elapsed_time_ms = 0U;
        machine->final_time_ms = 0U;
        machine->tracking_active = true;
        machine->timer_running = true;
        machine->final_time_within_requirement = false;
        machine->start_count++;
        started = true;
    }

    if (machine->timer_running && !started) {
        if (machine->elapsed_time_ms <= (UINT32_MAX - elapsed_ms)) {
            machine->elapsed_time_ms += elapsed_ms;
        } else {
            machine->elapsed_time_ms = UINT32_MAX;
        }
    }

    return started;
}

bool app_task_state_machine_finish_task1(app_task_state_machine_t *machine)
{
    if ((machine == NULL) ||
        (machine->state != APP_TASK_STATE_TASK1_TRACKING)) {
        return false;
    }

    machine->tracking_active = false;
    machine->timer_running = false;
    machine->final_time_ms = machine->elapsed_time_ms;
    machine->final_time_within_requirement =
        machine->final_time_ms <= APP_TASK1_TIME_REQUIREMENT_MS;
    machine->state = APP_TASK_STATE_TASK1_FINISHED;
    return true;
}
