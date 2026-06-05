#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>

#include "sim_types.h"
#include "sensor_imu.h"
#include "sensor_gps.h"
#include "sensor_battery.h"
#include "sensor_baro.h"
#include "fault_injector.h"
#include "packet_serialiser.h"
#include "scenario_engine.h"
#include "uart_bridge.h"
#include "tcp_server.h"

/* ------------------------------------------------------------------ */
/*  Globals                                                             */
/* ------------------------------------------------------------------ */

static volatile int g_running = 1;

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
    printf("\n[SIM] Caught signal — shutting down\n");
}

/* ------------------------------------------------------------------ */
/*  Monotonic millisecond clock                                         */
/* ------------------------------------------------------------------ */

static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void sleep_ms(uint32_t ms)
{
    struct timespec ts = {
        .tv_sec  = ms / 1000,
        .tv_nsec = (ms % 1000) * 1000000L,
    };
    nanosleep(&ts, NULL);
}

/* ------------------------------------------------------------------ */
/*  Usage                                                               */
/* ------------------------------------------------------------------ */

static void print_usage(const char *prog)
{
    printf("Usage: %s [OPTIONS]\n", prog);
    printf("  -d DEVICE    UART device (default: pty)\n");
    printf("  -b BAUD      Baud rate   (default: 115200)\n");
    printf("  -s SCENARIO  Scenario JSON file (default: built-in hover)\n");
    printf("  -p PORT      TCP command port (default: 5760)\n");
    printf("  -v           Verbose output\n");
    printf("  -h           This help\n");
    printf("\nExample:\n");
    printf("  %s -d /dev/ttyS0 -s scenarios/gps_loss_test.json -v\n", prog);
    printf("  %s -d pty        (creates PTY pair, prints slave path)\n", prog);
}

/* ------------------------------------------------------------------ */
/*  Main                                                                */
/* ------------------------------------------------------------------ */

int main(int argc, char *argv[])
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    /* Defaults */
    sim_config_t cfg = {
        .uart_device   = "pty",
        .uart_baud     = 115200,
        .scenario_file = "",
        .tcp_enabled   = true,
        .tcp_port      = SIM_TCP_PORT,
        .verbose       = false,
    };

    /* Argument parsing */
    int opt;
    while ((opt = getopt(argc, argv, "d:b:s:p:vh")) != -1) {
        switch (opt) {
        case 'd': strncpy(cfg.uart_device,   optarg, sizeof(cfg.uart_device)   - 1); break;
        case 'b': cfg.uart_baud = atoi(optarg);                                       break;
        case 's': strncpy(cfg.scenario_file, optarg, sizeof(cfg.scenario_file) - 1); break;
        case 'p': cfg.tcp_port = atoi(optarg);                                        break;
        case 'v': cfg.verbose = true;                                                  break;
        case 'h': print_usage(argv[0]); return 0;
        default:  print_usage(argv[0]); return 1;
        }
    }

    printf("=== AeroBench-SITL Sensor Simulator ===\n");

    /* ---- Init sensor models ---- */
    imu_model_t     imu;
    gps_model_t     gps;
    battery_model_t battery;
    baro_model_t    baro;

    imu_init(&imu);
    gps_init(&gps, 37.7749, -122.4194);   /* San Francisco */
    battery_init(&battery, 12.6f, 2200.0f);
    baro_init(&baro);

    /* ---- Init fault injector ---- */
    fault_injector_t faults;
    fault_init(&faults, &imu, &gps, &battery);

    /* ---- Load scenario ---- */
    scenario_t scenario;
    if (strlen(cfg.scenario_file) > 0)
        scenario_load(&scenario, cfg.scenario_file);
    else
        scenario_load_default(&scenario);

    /* ---- Open UART ---- */
    uart_bridge_t uart;
    if (uart_open(&uart, cfg.uart_device, cfg.uart_baud) != 0) {
        fprintf(stderr, "[SIM] Failed to open UART — aborting\n");
        return 1;
    }

    /* ---- Start TCP command server ---- */
    tcp_server_t tcp;
    uint32_t sim_wall_ms = 0;
    if (cfg.tcp_enabled)
        tcp_server_init(&tcp, cfg.tcp_port, &faults, &sim_wall_ms);

    /* ---- Timing ---- */
    const uint32_t TICK_MS     = SIM_TICK_MS;               /* 10ms   */
    const uint32_t TX_EVERY_N  = SIM_TICK_RATE_HZ / SIM_UART_TX_RATE_HZ; /* 2 ticks */
    uint32_t tick_count        = 0;
    uint32_t start_real_ms     = now_ms();
    uint32_t last_stats_ms     = 0;

    printf("[SIM] Running at %u Hz, TX at %u Hz. CTRL+C to stop.\n",
           SIM_TICK_RATE_HZ, SIM_UART_TX_RATE_HZ);

    /* ================================================================
     * Main simulation loop
     * ================================================================ */
    while (g_running) {
        uint32_t loop_start = now_ms();
        sim_wall_ms = loop_start - start_real_ms;

        /* 1. Tick scenario engine → drives sensor model true values */
        scenario_tick(&scenario, &imu, &gps, &battery, &baro,
                      &faults, sim_wall_ms);

        /* 2. Tick fault injector → applies/expires faults to models */
        fault_tick(&faults, sim_wall_ms);

        /* 3. Sample all sensor models */
        sim_sensor_data_t sensors = {0};
        float dt = (float)TICK_MS / 1000.0f;

        imu_tick(&imu, dt,
                 &sensors.accel_x, &sensors.accel_y, &sensors.accel_z,
                 &sensors.gyro_x,  &sensors.gyro_y,  &sensors.gyro_z);

        gps_tick(&gps,
                 &sensors.latitude, &sensors.longitude,
                 &sensors.altitude_m, &sensors.ground_speed_mps,
                 &sensors.gps_fix_quality, &sensors.gps_satellites);

        battery_tick(&battery, dt,
                     &sensors.battery_voltage_v,
                     &sensors.battery_current_a,
                     &sensors.battery_pct);

        baro_tick(&baro,
                  &sensors.baro_pressure_hpa,
                  &sensors.temperature_c);

        sensors.timestamp_ms  = sim_wall_ms;
        sensors.sensor_status = SIM_SENSOR_ALL_OK;
        if (sensors.gps_fix_quality == 0)
            sensors.sensor_status &= ~SIM_SENSOR_GPS_OK;

        /* 4. Transmit sensor frame at reduced rate */
        if ((tick_count % TX_EVERY_N) == 0 && !faults.comms_drop_active) {
            static uint8_t tx_frame[PROTO_HEADER_SIZE +
                                    128 + PROTO_CRC_SIZE];
            size_t n = pkt_build_sensor_frame(&sensors, tx_frame,
                                              sizeof(tx_frame));
            if (n > 0) {
                if (faults.packet_corrupt_active)
                    pkt_corrupt_crc(tx_frame, n);
                uart_write_frame(&uart, tx_frame, n);
            }
        }

        /* 5. Service TCP command server */
        if (cfg.tcp_enabled)
            tcp_server_tick(&tcp);

        /* 6. Read any incoming UART (ACKs from STM32) */
        {
            uint8_t rx[64];
            int n = uart_read(&uart, rx, sizeof(rx));
            if (n > 0 && cfg.verbose)
                printf("[SIM] RX %d bytes from STM32\n", n);
        }

        /* 7. Periodic stats */
        if ((sim_wall_ms - last_stats_ms) >= 5000u) {
            last_stats_ms = sim_wall_ms;
            printf("[SIM] t=%u ms  TX=%u frames  alt=%.1f m  "
                   "batt=%.2fV  faults=0x%02X\n",
                   sim_wall_ms, uart.tx_frames,
                   sensors.altitude_m, sensors.battery_voltage_v,
                   fault_get_flags(&faults));
        }

        tick_count++;

        /* 8. Rate limiting: sleep for the remainder of the tick */
        uint32_t elapsed = now_ms() - loop_start;
        if (elapsed < TICK_MS)
            sleep_ms(TICK_MS - elapsed);
    }

    /* ---- Shutdown ---- */
    printf("[SIM] Stopping after %u ticks (%.1f s)\n",
           tick_count, (float)sim_wall_ms / 1000.0f);

    if (cfg.tcp_enabled) tcp_server_close(&tcp);
    uart_close(&uart);
    return 0;
}
