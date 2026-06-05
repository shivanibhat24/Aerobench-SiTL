#include "uart_bridge.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <pty.h>

/* ------------------------------------------------------------------ */
/*  Baud rate constant lookup                                           */
/* ------------------------------------------------------------------ */

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default:
        fprintf(stderr, "[UART] Unknown baud %d, defaulting to 115200\n", baud);
        return B115200;
    }
}

/* ------------------------------------------------------------------ */
/*  Configure a file descriptor as a raw serial port                   */
/* ------------------------------------------------------------------ */

static int configure_tty(int fd, int baud)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("[UART] tcgetattr");
        return -1;
    }

    speed_t speed = baud_to_speed(baud);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    /* 8N1, no flow control, raw mode */
    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tty.c_cflag |=  (CS8 | CREAD | CLOCAL);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT |
                     PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    /* Non-blocking reads */
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("[UART] tcsetattr");
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

int uart_open(uart_bridge_t *b, const char *device, int baud)
{
    memset(b, 0, sizeof(*b));
    b->baud = baud;

    if (strcmp(device, "pty") == 0) {
        /* Create a PTY pair for loopback/virtual testing */
        int master_fd, slave_fd;
        char slave_name[64];

        if (openpty(&master_fd, &slave_fd, slave_name, NULL, NULL) != 0) {
            perror("[UART] openpty");
            return -1;
        }
        close(slave_fd);   /* Slave will be opened by whoever connects */

        configure_tty(master_fd, baud);

        b->fd       = master_fd;
        b->use_pty  = true;
        strncpy(b->device,     "pty master", sizeof(b->device) - 1);
        strncpy(b->pty_slave,  slave_name,   sizeof(b->pty_slave) - 1);
        printf("[UART] PTY pair created. Connect to: %s\n", slave_name);
    } else {
        /* Open a real or existing serial device */
        int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            fprintf(stderr, "[UART] Cannot open %s: %s\n",
                    device, strerror(errno));
            return -1;
        }
        if (configure_tty(fd, baud) != 0) {
            close(fd);
            return -1;
        }
        b->fd = fd;
        strncpy(b->device, device, sizeof(b->device) - 1);
        printf("[UART] Opened %s at %d baud\n", device, baud);
    }

    return 0;
}

int uart_write_frame(uart_bridge_t *b, const uint8_t *frame, size_t len)
{
    if (b->fd < 0 || len == 0) return -1;

    ssize_t written = write(b->fd, frame, len);
    if (written < 0) {
        if (errno != EAGAIN) {
            perror("[UART] write");
            b->tx_errors++;
        }
        return -1;
    }

    b->tx_frames++;
    return (int)written;
}

int uart_read(uart_bridge_t *b, uint8_t *buf, size_t buf_size)
{
    if (b->fd < 0) return -1;
    ssize_t n = read(b->fd, buf, buf_size);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        perror("[UART] read");
        return -1;
    }
    if (n > 0) b->rx_acks += (uint32_t)n;
    return (int)n;
}

void uart_close(uart_bridge_t *b)
{
    if (b->fd >= 0) {
        close(b->fd);
        b->fd = -1;
    }
    printf("[UART] Closed. TX frames: %u  RX bytes: %u  Errors: %u\n",
           b->tx_frames, b->rx_acks, b->tx_errors);
}
