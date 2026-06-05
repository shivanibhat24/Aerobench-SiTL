#ifndef HEALTH_MONITOR_H
#define HEALTH_MONITOR_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------ */
/*  Task handle registry — filled in by main.c after task creation     */
/* ------------------------------------------------------------------ */

typedef struct {
    TaskHandle_t sensor_task;
    TaskHandle_t fsm_task;
    TaskHandle_t telemetry_task;
    TaskHandle_t health_task;
} task_registry_t;

/* ------------------------------------------------------------------ */
/*  Health snapshot (populated each health monitor tick)               */
/* ------------------------------------------------------------------ */

typedef struct {
    uint32_t timestamp_ms;
    uint8_t  cpu_load_pct;
    uint16_t hwm_sensor;       /* stack high-water mark, words */
    uint16_t hwm_fsm;
    uint16_t hwm_telemetry;
    uint16_t hwm_health;
    uint32_t free_heap_bytes;
    uint8_t  fault_flags;
} health_snapshot_t;

/* ------------------------------------------------------------------ */
/*  Thresholds                                                          */
/* ------------------------------------------------------------------ */

#define HEALTH_STACK_MIN_HWM    (32u)   /* words — raise fault if below */
#define HEALTH_HEAP_MIN_BYTES   (512u)
#define HEALTH_REPORT_PERIOD_MS (1000u)
#define HEALTH_WATCHDOG_FEED_MS (500u)

/* ------------------------------------------------------------------ */
/*  API                                                                 */
/* ------------------------------------------------------------------ */

void health_init(task_registry_t *registry);
void health_get_snapshot(health_snapshot_t *out);

/* Called from task_health — one tick of monitoring logic */
void health_tick(uint32_t now_ms);

#endif /* HEALTH_MONITOR_H */
