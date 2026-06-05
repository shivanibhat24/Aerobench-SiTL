#include "tcp_server.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

static void set_nonblocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

int tcp_server_init(tcp_server_t *srv, int port,
                    fault_injector_t *faults,
                    uint32_t *sim_time_ms)
{
    memset(srv, 0, sizeof(*srv));
    srv->port         = port;
    srv->faults       = faults;
    srv->sim_time_ms  = sim_time_ms;
    srv->client_fd    = -1;

    srv->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (srv->listen_fd < 0) { perror("[TCP] socket"); return -1; }

    int opt = 1;
    setsockopt(srv->listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    set_nonblocking(srv->listen_fd);

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port        = htons((uint16_t)port),
    };

    if (bind(srv->listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[TCP] bind"); close(srv->listen_fd); return -1;
    }
    if (listen(srv->listen_fd, 1) < 0) {
        perror("[TCP] listen"); close(srv->listen_fd); return -1;
    }

    printf("[TCP] Listening on port %d for GCS commands\n", port);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Command parser                                                      */
/* ------------------------------------------------------------------ */

static void handle_command(tcp_server_t *srv, const char *line)
{
    char cmd[32] = {0};
    sscanf(line, "%31s", cmd);

    if (strcmp(cmd, "FAULT") == 0) {
        char sub[32] = {0};
        sscanf(line + 6, "%31s", sub);

        if (strcmp(sub, "CLEAR") == 0) {
            fault_clear_all(srv->faults);
            dprintf(srv->client_fd, "OK cleared all faults\n");
            return;
        }

        fault_type_t type  = FAULT_TYPE_NONE;
        float        param = 0.0f;
        uint32_t     dur   = 0;

        if (strcmp(sub, "GPS_LOSS") == 0) {
            type = FAULT_TYPE_GPS_LOSS;
            sscanf(line + 6 + strlen(sub) + 1, "%u", &dur);
        } else if (strcmp(sub, "SENSOR_FREEZE") == 0) {
            type = FAULT_TYPE_SENSOR_FREEZE;
            sscanf(line + 6 + strlen(sub) + 1, "%u", &dur);
        } else if (strcmp(sub, "BATT_SAG") == 0) {
            type = FAULT_TYPE_BATTERY_SAG;
            sscanf(line + 6 + strlen(sub) + 1, "%f %u", &param, &dur);
            if (param == 0.0f) param = 10.0f; /* default sag target */
        } else if (strcmp(sub, "COMMS_DROP") == 0) {
            type = FAULT_TYPE_COMMS_DROP;
            sscanf(line + 6 + strlen(sub) + 1, "%u", &dur);
        } else if (strcmp(sub, "PKT_CORRUPT") == 0) {
            type = FAULT_TYPE_PACKET_CORRUPT;
            sscanf(line + 6 + strlen(sub) + 1, "%u", &dur);
        } else {
            dprintf(srv->client_fd, "ERR unknown fault type: %s\n", sub);
            return;
        }

        int slot = fault_inject(srv->faults, type, param, dur,
                                *srv->sim_time_ms);
        if (slot >= 0)
            dprintf(srv->client_fd, "OK injected %s slot=%d dur=%u\n",
                    sub, slot, dur);
        else
            dprintf(srv->client_fd, "ERR no free fault slots\n");

    } else if (strcmp(cmd, "STATUS") == 0) {
        uint8_t flags = fault_get_flags(srv->faults);
        dprintf(srv->client_fd,
                "STATUS flags=0x%02X time=%u\n",
                flags, *srv->sim_time_ms);

    } else if (strcmp(cmd, "QUIT") == 0) {
        dprintf(srv->client_fd, "BYE\n");
        close(srv->client_fd);
        srv->client_fd = -1;
        printf("[TCP] Client disconnected\n");

    } else {
        dprintf(srv->client_fd, "ERR unknown command: %s\n", cmd);
    }
}

/* ------------------------------------------------------------------ */
/*  Main loop tick — non-blocking                                       */
/* ------------------------------------------------------------------ */

void tcp_server_tick(tcp_server_t *srv)
{
    /* Accept new connection if none active */
    if (srv->client_fd < 0) {
        struct sockaddr_in peer;
        socklen_t plen = sizeof(peer);
        int fd = accept(srv->listen_fd,
                        (struct sockaddr *)&peer, &plen);
        if (fd >= 0) {
            set_nonblocking(fd);
            srv->client_fd = fd;
            printf("[TCP] GCS connected from %s\n",
                   inet_ntoa(peer.sin_addr));
            dprintf(fd, "AEROBENCH-SIM ready. Commands: FAULT|STATUS|QUIT\n");
        }
        return;
    }

    /* Read available data from connected client */
    static char rx_buf[256];
    static int  rx_pos = 0;

    ssize_t n = read(srv->client_fd,
                     rx_buf + rx_pos,
                     sizeof(rx_buf) - 1 - rx_pos);

    if (n < 0 && errno != EAGAIN) {
        printf("[TCP] Client read error — closing\n");
        close(srv->client_fd);
        srv->client_fd = -1;
        rx_pos = 0;
        return;
    }
    if (n == 0) {   /* EOF */
        printf("[TCP] Client closed connection\n");
        close(srv->client_fd);
        srv->client_fd = -1;
        rx_pos = 0;
        return;
    }
    if (n < 0) return;  /* EAGAIN — nothing yet */

    rx_pos += (int)n;
    rx_buf[rx_pos] = '\0';

    /* Process complete lines */
    char *line_start = rx_buf;
    char *newline;
    while ((newline = strchr(line_start, '\n')) != NULL) {
        *newline = '\0';
        /* Strip carriage return */
        size_t len = strlen(line_start);
        if (len > 0 && line_start[len-1] == '\r')
            line_start[len-1] = '\0';

        if (strlen(line_start) > 0)
            handle_command(srv, line_start);

        line_start = newline + 1;
    }

    /* Shift remaining partial line to front */
    int remaining = (int)(rx_buf + rx_pos - line_start);
    if (remaining > 0 && line_start != rx_buf)
        memmove(rx_buf, line_start, (size_t)remaining);
    else if (remaining == 0)
        remaining = 0;
    rx_pos = remaining;
}

void tcp_server_close(tcp_server_t *srv)
{
    if (srv->client_fd >= 0) { close(srv->client_fd); srv->client_fd = -1; }
    if (srv->listen_fd  >= 0) { close(srv->listen_fd); srv->listen_fd  = -1; }
}
