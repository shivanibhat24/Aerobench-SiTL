#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H

#include "sim_types.h"

typedef struct {
    int   fd;              /* open file descriptor             */
    char  device[64];      /* path: /dev/ttyS0 or /dev/pts/N  */
    int   baud;
    bool  use_pty;         /* true if we opened a PTY pair     */
    char  pty_slave[64];   /* slave end path when use_pty=true */

    /* Stats */
    uint32_t tx_frames;
    uint32_t rx_acks;
    uint32_t tx_errors;
} uart_bridge_t;

/* Open the UART device (or create a PTY pair if device == "pty") */
int  uart_open(uart_bridge_t *b, const char *device, int baud);

/* Write a fully-formed wire frame. Returns bytes written or -1. */
int  uart_write_frame(uart_bridge_t *b,
                      const uint8_t *frame, size_t len);

/* Non-blocking read — fills buf up to buf_size bytes. Returns bytes read. */
int  uart_read(uart_bridge_t *b, uint8_t *buf, size_t buf_size);

void uart_close(uart_bridge_t *b);

#endif /* UART_BRIDGE_H */
