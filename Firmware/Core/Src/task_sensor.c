#include "cmsis_os.h"
#include "protocol.h"
#include "sensor_data.h"
#include "blackbox.h"
#include "main.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Externals from main.c                                               */
/* ------------------------------------------------------------------ */

extern UART_HandleTypeDef huart2;        /* UART connected to QEMU/PC  */
extern osMessageQueueId_t g_sensor_queue;
extern blackbox_t         g_blackbox;

/* ------------------------------------------------------------------ */
/*  Task configuration                                                  */
/* ------------------------------------------------------------------ */

#define SENSOR_TASK_PERIOD_MS   (10u)    /* 100 Hz poll rate            */
#define UART_RX_BUF_SIZE        (256u)

/* ------------------------------------------------------------------ */
/*  Module-local state                                                  */
/* ------------------------------------------------------------------ */

static proto_rx_ctx_t  s_rx_ctx;
static uint8_t         s_uart_rx_byte;  /* single-byte IT receive target */
static sensor_data_t   s_latest;
static uint32_t        s_last_packet_ms  = 0;
static uint32_t        s_packet_count    = 0;
static uint32_t        s_crc_error_count = 0;
static volatile bool   s_byte_ready      = false;

/* ------------------------------------------------------------------ */
/*  UART RX complete callback (called from ISR)                         */
/* ------------------------------------------------------------------ */

/*
 * Override HAL_UART_RxCpltCallback in this file.
 * STM32CubeIDE generates a weak version; this replaces it.
 * We feed the received byte into the protocol state machine.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != huart2.Instance) return;

    /* Feed byte to RX state machine */
    if (proto_rx_feed(&s_rx_ctx, s_uart_rx_byte)) {
        /* Complete valid frame — copy frame type for task to handle */
        s_byte_ready = true;
    }

    /* Re-arm single-byte interrupt */
    HAL_UART_Receive_IT(&huart2, &s_uart_rx_byte, 1);
}

/* ------------------------------------------------------------------ */
/*  Handle a fully received, CRC-validated frame                        */
/* ------------------------------------------------------------------ */

static void handle_frame(const proto_frame_t *frame)
{
    switch (frame->type) {

    case PKT_TYPE_SENSOR_FRAME: {
        sensor_data_t parsed;
        if (proto_unpack_sensor(frame->payload, frame->len, &parsed)) {
            /* Copy to module-local latest */
            memcpy(&s_latest, &parsed, sizeof(s_latest));
            s_last_packet_ms = osKernelGetTickCount();
            s_packet_count++;

            /* Push to queue — drop oldest if full (non-blocking) */
            if (osMessageQueueGetCount(g_sensor_queue) > 0) {
                sensor_data_t dummy;
                osMessageQueueGet(g_sensor_queue, &dummy, NULL, 0);
            }
            osMessageQueuePut(g_sensor_queue, &parsed, 0, 0);
        } else {
            s_crc_error_count++;
        }
        break;
    }

    case PKT_TYPE_HEARTBEAT_REQ: {
        /* Build and send ACK immediately */
        uint8_t ack_buf[PROTO_HEADER_SIZE + PROTO_CRC_SIZE + 1];
        uint8_t payload = 0xACu;
        size_t n = /* minimal frame */ PROTO_HEADER_SIZE + 1 + PROTO_CRC_SIZE;
        ack_buf[0] = PROTO_SOF;
        ack_buf[1] = PKT_TYPE_HEARTBEAT_ACK;
        ack_buf[2] = 1;
        ack_buf[3] = 0;
        ack_buf[4] = payload;
        uint16_t crc = proto_crc16(ack_buf + 1, 4);
        ack_buf[5] = (uint8_t)(crc & 0xFF);
        ack_buf[6] = (uint8_t)(crc >> 8);
        HAL_UART_Transmit(&huart2, ack_buf, (uint16_t)n, 10);
        break;
    }

    default:
        /* Unknown type — ignore */
        break;
    }
}

/* ------------------------------------------------------------------ */
/*  Task entry point                                                    */
/* ------------------------------------------------------------------ */

void task_sensor(void *argument)
{
    (void)argument;

    /* Initialise RX state machine */
    proto_rx_reset(&s_rx_ctx);
    memset(&s_latest, 0, sizeof(s_latest));

    /* Arm first UART RX interrupt */
    HAL_UART_Receive_IT(&huart2, &s_uart_rx_byte, 1);

    /* Log boot event */
    blackbox_log(&g_blackbox, BB_EVT_BOOT,
                 FLIGHT_STATE_INIT, 0, 0,
                 0.0f, 0.0f, 0.0f,
                 "sensor_task started");

    uint32_t last_wake = osKernelGetTickCount();
    uint32_t comms_warn_logged = 0;

    for (;;) {
        osDelayUntil(last_wake + SENSOR_TASK_PERIOD_MS);
        last_wake = osKernelGetTickCount();

        /* Process any frame that arrived during the period */
        if (s_byte_ready) {
            s_byte_ready = false;
            handle_frame(&s_rx_ctx.frame);
        }

        /* Comms watchdog — log if no packet within timeout */
        uint32_t now = osKernelGetTickCount();
        if (s_last_packet_ms > 0 &&
            (now - s_last_packet_ms) > COMMS_TIMEOUT_MS &&
            (now - comms_warn_logged) > 2000u)
        {
            blackbox_log(&g_blackbox, BB_EVT_COMMS_TIMEOUT,
                         FLIGHT_STATE_INIT, FAULT_FLAG_COMMS_TIMEOUT, 0,
                         0.0f, 0.0f, 0.0f,
                         "UART RX timeout");
            comms_warn_logged = now;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Diagnostics accessors (called from health task / telemetry)        */
/* ------------------------------------------------------------------ */

uint32_t sensor_task_get_packet_count(void)   { return s_packet_count; }
uint32_t sensor_task_get_crc_errors(void)     { return s_crc_error_count; }
uint32_t sensor_task_get_last_rx_ms(void)     { return s_last_packet_ms; }
