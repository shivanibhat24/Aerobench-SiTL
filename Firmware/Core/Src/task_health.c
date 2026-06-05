#include "cmsis_os.h"
#include "health_monitor.h"
#include "protocol.h"
#include "blackbox.h"
#include "main.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Externals                                                           */
/* ------------------------------------------------------------------ */

extern UART_HandleTypeDef  huart2;
extern IWDG_HandleTypeDef  hiwdg;
extern blackbox_t          g_blackbox;
extern osEventFlagsId_t    g_fault_flags;

/* Forward from task_sensor */
uint32_t sensor_task_get_last_rx_ms(void);
uint8_t  fsm_task_get_fault_flags(void);
flight_state_t fsm_task_get_state(void);

/* ------------------------------------------------------------------ */
/*  Module state                                                        */
/* ------------------------------------------------------------------ */

static task_registry_t   s_registry;
static health_snapshot_t s_snapshot;

void health_init(task_registry_t *reg)
{
    memcpy(&s_registry, reg, sizeof(s_registry));
    memset(&s_snapshot, 0, sizeof(s_snapshot));
}

void health_get_snapshot(health_snapshot_t *out)
{
    memcpy(out, &s_snapshot, sizeof(*out));
}

/* ------------------------------------------------------------------ */
/*  Task entry point                                                    */
/* ------------------------------------------------------------------ */

#define HEALTH_PERIOD_MS    (200u)   /* 5 Hz — fast enough for IWDG   */

void task_health(void *argument)
{
    (void)argument;

    static uint8_t tx_buf[PROTO_MAX_FRAME];

    uint32_t last_wake      = osKernelGetTickCount();
    uint32_t last_report_ms = 0;
    uint32_t last_comms_check_ms = 0;

    for (;;) {
        osDelayUntil(last_wake + HEALTH_PERIOD_MS);
        last_wake = osKernelGetTickCount();
        uint32_t now = last_wake;

        /* -------- Feed hardware watchdog -------- */
        HAL_IWDG_Refresh(&hiwdg);

        /* -------- Stack high-water marks -------- */
        uint16_t hwm_sensor = 0, hwm_fsm = 0, hwm_telem = 0, hwm_health = 0;

        if (s_registry.sensor_task)
            hwm_sensor = (uint16_t)uxTaskGetStackHighWaterMark(s_registry.sensor_task);
        if (s_registry.fsm_task)
            hwm_fsm    = (uint16_t)uxTaskGetStackHighWaterMark(s_registry.fsm_task);
        if (s_registry.telemetry_task)
            hwm_telem  = (uint16_t)uxTaskGetStackHighWaterMark(s_registry.telemetry_task);
        if (s_registry.health_task)
            hwm_health = (uint16_t)uxTaskGetStackHighWaterMark(s_registry.health_task);

        /* -------- Heap -------- */
        uint32_t free_heap = (uint32_t)xPortGetFreeHeapSize();

        /* -------- Raise stack overflow fault if any HWM critical ---- */
        bool stack_critical = (hwm_sensor  < HEALTH_STACK_MIN_HWM ||
                               hwm_fsm     < HEALTH_STACK_MIN_HWM ||
                               hwm_telem   < HEALTH_STACK_MIN_HWM ||
                               hwm_health  < HEALTH_STACK_MIN_HWM);

        if (stack_critical)
            osEventFlagsSet(g_fault_flags, FAULT_FLAG_STACK_OVERFLOW);
        else
            osEventFlagsClear(g_fault_flags, FAULT_FLAG_STACK_OVERFLOW);

        /* -------- COMMS timeout fault check -------- */
        uint32_t last_rx = sensor_task_get_last_rx_ms();
        if (last_rx > 0 && (now - last_rx) > COMMS_TIMEOUT_MS) {
            osEventFlagsSet(g_fault_flags, FAULT_FLAG_COMMS_TIMEOUT);
        } else {
            osEventFlagsClear(g_fault_flags, FAULT_FLAG_COMMS_TIMEOUT);
        }

        /* -------- Update snapshot -------- */
        s_snapshot.timestamp_ms    = now;
        s_snapshot.hwm_sensor      = hwm_sensor;
        s_snapshot.hwm_fsm         = hwm_fsm;
        s_snapshot.hwm_telemetry   = hwm_telem;
        s_snapshot.hwm_health      = hwm_health;
        s_snapshot.free_heap_bytes = free_heap;
        s_snapshot.fault_flags     = fsm_task_get_fault_flags();

        /* -------- Periodic health report to GCS -------- */
        if ((now - last_report_ms) >= HEALTH_REPORT_PERIOD_MS) {
            last_report_ms = now;

            pkt_health_report_t rpt;
            rpt.timestamp_ms    = now;
            rpt.cpu_load_pct    = 0;   /* CPU load requires timer — placeholder */
            rpt.hwm_sensor      = hwm_sensor;
            rpt.hwm_fsm         = hwm_fsm;
            rpt.hwm_telemetry   = hwm_telem;
            rpt.hwm_health      = hwm_health;
            rpt.free_heap_bytes = free_heap;

            size_t n = proto_build_health(&rpt, tx_buf, sizeof(tx_buf));
            if (n > 0)
                HAL_UART_Transmit(&huart2, tx_buf, (uint16_t)n, 20);

            /* Log health snapshot to black-box every 5 seconds */
            static uint32_t last_bb_ms = 0;
            if ((now - last_bb_ms) >= 5000u) {
                last_bb_ms = now;
                blackbox_log(&g_blackbox,
                             BB_EVT_HEALTH_SNAPSHOT,
                             fsm_task_get_state(),
                             s_snapshot.fault_flags,
                             0,
                             0.0f, 0.0f, 0.0f,
                             "health ok");
            }
        }
    }
}
