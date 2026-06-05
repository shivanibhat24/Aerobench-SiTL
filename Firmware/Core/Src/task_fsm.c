#include "cmsis_os.h"
#include "flight_fsm.h"
#include "sensor_data.h"
#include "blackbox.h"
#include "main.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Externals from main.c                                               */
/* ------------------------------------------------------------------ */

extern osMessageQueueId_t g_sensor_queue;
extern osMutexId_t        g_state_mutex;
extern osEventFlagsId_t   g_fault_flags;
extern blackbox_t         g_blackbox;

/* ------------------------------------------------------------------ */
/*  Shared flight state — protected by g_state_mutex                   */
/* ------------------------------------------------------------------ */

static fsm_ctx_t      s_fsm;
static sensor_data_t  s_last_sensors;

/* Read-only accessors for other tasks (caller must NOT hold mutex) */
flight_state_t fsm_task_get_state(void)
{
    osMutexAcquire(g_state_mutex, osWaitForever);
    flight_state_t s = s_fsm.current_state;
    osMutexRelease(g_state_mutex);
    return s;
}

void fsm_task_get_sensors(sensor_data_t *out)
{
    osMutexAcquire(g_state_mutex, osWaitForever);
    memcpy(out, &s_last_sensors, sizeof(*out));
    osMutexRelease(g_state_mutex);
}

uint8_t fsm_task_get_fault_flags(void)
{
    return (uint8_t)osEventFlagsGet(g_fault_flags);
}

/* ------------------------------------------------------------------ */
/*  Internal: evaluate sensor data for fault conditions                 */
/* ------------------------------------------------------------------ */

static uint32_t evaluate_faults(const sensor_data_t *s, uint32_t now_ms)
{
    uint32_t flags = 0;

    /* GPS loss */
    if (s->gps_fix_quality == 0 || s->gps_satellites < GPS_MIN_SATELLITES)
        flags |= FAULT_FLAG_GPS_LOSS;

    /* Battery low */
    if (s->battery_voltage_v < BATT_LOW_VOLTAGE_V)
        flags |= FAULT_FLAG_BATT_LOW;

    /* Sensor freeze: all accel values exactly zero is suspicious */
    if (s->accel_x == 0.0f && s->accel_y == 0.0f && s->accel_z == 0.0f &&
        s->gyro_x  == 0.0f && s->gyro_y  == 0.0f && s->gyro_z  == 0.0f)
        flags |= FAULT_FLAG_SENSOR_FREEZE;

    (void)now_ms;
    return flags;
}

/* ------------------------------------------------------------------ */
/*  Task entry point                                                    */
/* ------------------------------------------------------------------ */

#define FSM_TASK_PERIOD_MS  (20u)   /* 50 Hz FSM update rate */

void task_fsm(void *argument)
{
    (void)argument;

    fsm_init(&s_fsm);
    memset(&s_last_sensors, 0, sizeof(s_last_sensors));

    uint32_t last_wake = osKernelGetTickCount();

    for (;;) {
        osDelayUntil(last_wake + FSM_TASK_PERIOD_MS);
        last_wake = osKernelGetTickCount();

        /* Pull latest sensor data from queue */
        sensor_data_t fresh;
        bool got_sensors = (osMessageQueueGet(g_sensor_queue,
                                              &fresh, NULL, 0) == osOK);

        osMutexAcquire(g_state_mutex, osWaitForever);

        if (got_sensors)
            memcpy(&s_last_sensors, &fresh, sizeof(fresh));

        /* Evaluate fault conditions from sensor data */
        uint32_t now_ms    = osKernelGetTickCount();
        uint32_t new_faults = evaluate_faults(&s_last_sensors, now_ms);

        /* Sync fault flags EventGroup */
        uint32_t prev_faults = osEventFlagsGet(g_fault_flags);

        /* Set newly raised faults */
        if (new_faults & ~prev_faults)
            osEventFlagsSet(g_fault_flags, new_faults & ~prev_faults);

        /* Clear faults that are no longer active */
        if (prev_faults & ~new_faults)
            osEventFlagsClear(g_fault_flags, prev_faults & ~new_faults);

        /* Run FSM tick */
        bool transitioned = fsm_tick(&s_fsm, &s_last_sensors,
                                      new_faults, now_ms);

        osMutexRelease(g_state_mutex);

        /* Log fault state changes */
        if (new_faults != prev_faults) {
            const char *msg = (new_faults > prev_faults)
                            ? "fault raised"
                            : "fault cleared";
            float accel = s_last_sensors.accel_z;
            blackbox_log(&g_blackbox,
                         new_faults > prev_faults ? BB_EVT_FAULT_SET
                                                   : BB_EVT_FAULT_CLEAR,
                         s_fsm.current_state,
                         (uint8_t)new_faults,
                         s_last_sensors.sensor_status,
                         s_last_sensors.battery_voltage_v,
                         s_last_sensors.altitude_m,
                         accel, msg);
        }

        (void)transitioned; /* transition already logged inside fsm_tick */
    }
}
