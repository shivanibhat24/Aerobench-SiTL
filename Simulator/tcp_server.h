#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "sim_types.h"
#include "fault_injector.h"

/* ------------------------------------------------------------------ */
/*  Simple line-based ASCII command server                              */
/*  GCS connects on TCP port 5760 and sends commands like:            */
/*    FAULT GPS_LOSS 5000          inject GPS loss for 5 seconds      */
/*    FAULT BATT_SAG 9.5 0         latch battery sag to 9.5V          */
/*    FAULT SENSOR_FREEZE 3000     freeze IMU for 3 seconds            */
/*    FAULT CLEAR                  clear all faults                    */
/*    STATUS                       returns active fault flags          */
/*    QUIT                         close connection                    */
/* ------------------------------------------------------------------ */

typedef struct {
    int               listen_fd;
    int               client_fd;
    int               port;
    fault_injector_t *faults;
    uint32_t         *sim_time_ms;   /* pointer to simulator wall clock */
} tcp_server_t;

int  tcp_server_init(tcp_server_t *srv, int port,
                     fault_injector_t *faults,
                     uint32_t *sim_time_ms);

/* Non-blocking — call from main loop each tick */
void tcp_server_tick(tcp_server_t *srv);

void tcp_server_close(tcp_server_t *srv);

#endif /* TCP_SERVER_H */
