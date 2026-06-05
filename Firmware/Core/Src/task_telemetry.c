#include "cmsis_os.h"
#include "protocol.h"
#include "sensor_data.h"
#include "blackbox.h"
#include "main.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Externals                                                           */
/* ------------------------------------------------------------------ */

extern UART_HandleTypeDef huart2;
extern blackbox_t         g_blackbox;

/* Forward declarations from other tasks */
flight_state_t fsm_task_get_state(void);
void           fsm_task_get_sensors(sensor_data_t *out);
uint8_t        fsm_task_get_fault_flags(void);

/* ------------------------------------------------------------------ */
/*  Task configuration                                                  */
/* ------------------------------------------------------------------ */

#define TELEM_PERIOD_MS         (100u)   /* 10 Hz                        */
#define TELEM_TX_TIMEOUT_MS     (20u)    /* UART TX blocking timeout     */
#define TELEM_BLACKBOX_BURST    (4u)     /* blackbox records per tick    */

/* ------------------------------------------------------------------ */
/*  Transmit a wire frame over UART2                                    */
/* ------------------------------------------------------------------ */

static void uart_send(const uint8_t *buf, size_t len)
{
    if (len == 0) return;
    HAL_UART_Transmit(&huart2, (uint8_t *)buf,
                      (uint16_t)len, TELEM_TX_TIMEOUT_MS);
}

/* ------------------------------------------------------------------ */
/*  Task entry point                                                    */
/* ------------------------------------------------------------------ */

void task_telemetry(void *argument)
{
    (void)argument;

    static uint8_t    tx_buf[PROTO_MAX_FRAME];
    static sensor_data_t snap;

    uint32_t last_wake = osKernelGetTickCount();

    for (;;) {
        osDelayUntil(last_wake + TELEM_PERIOD_MS);
        last_wake = osKernelGetTickCount();

        /* --- Primary telemetry frame --- */
        fsm_task_get_sensors(&snap);
        flight_state_t state = fsm_task_get_state();
        uint8_t faults       = fsm_task_get_fault_flags();

        size_t n = proto_build_telemetry(&snap, state, faults,
                                          tx_buf, sizeof(tx_buf));
        uart_send(tx_buf, n);

        /* --- Drain a few black-box records per tick --- */
        bb_record_t rec;
        for (uint8_t i = 0; i < TELEM_BLACKBOX_BURST; i++) {
            if (!blackbox_read(&g_blackbox, &rec)) break;

            /* Build a PKT_TYPE_BLACKBOX_ENTRY wire frame */
            pkt_blackbox_entry_t bb_pkt;
            memset(&bb_pkt, 0, sizeof(bb_pkt));
            bb_pkt.timestamp_ms      = rec.timestamp_ms;
            bb_pkt.event_type        = rec.event_type;
            bb_pkt.flight_state      = rec.flight_state;
            bb_pkt.fault_flags       = rec.fault_flags;
            bb_pkt.sensor_status     = rec.sensor_status;
            bb_pkt.battery_voltage_v = rec.battery_voltage_v;
            bb_pkt.altitude_m        = rec.altitude_m;
            memcpy(bb_pkt.message, rec.message, sizeof(bb_pkt.message));

            /* Inline frame build for blackbox type */
            uint8_t bb_buf[PROTO_HEADER_SIZE + sizeof(bb_pkt) + PROTO_CRC_SIZE];
            bb_buf[0] = PROTO_SOF;
            bb_buf[1] = PKT_TYPE_BLACKBOX_ENTRY;
            bb_buf[2] = (uint8_t)(sizeof(bb_pkt) & 0xFF);
            bb_buf[3] = (uint8_t)(sizeof(bb_pkt) >> 8);
            memcpy(bb_buf + PROTO_HEADER_SIZE, &bb_pkt, sizeof(bb_pkt));
            uint16_t crc = proto_crc16(bb_buf + 1,
                                        3u + sizeof(bb_pkt));
            size_t total = PROTO_HEADER_SIZE + sizeof(bb_pkt) + PROTO_CRC_SIZE;
            bb_buf[total - 2] = (uint8_t)(crc & 0xFF);
            bb_buf[total - 1] = (uint8_t)(crc >> 8);
            uart_send(bb_buf, total);
        }
    }
}
