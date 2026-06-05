#ifndef FAULT_INJECTOR_H
#define FAULT_INJECTOR_H

#include "sim_types.h"
#include "sensor_imu.h"
#include "sensor_gps.h"
#include "sensor_battery.h"

/* ------------------------------------------------------------------ */
/*  One active fault entry                                              */
/* ------------------------------------------------------------------ */

typedef enum {
    FAULT_TYPE_NONE = 0,
    FAULT_TYPE_GPS_LOSS,
    FAULT_TYPE_SENSOR_FREEZE,
    FAULT_TYPE_BATTERY_SAG,
    FAULT_TYPE_COMMS_DROP,      /* drops outbound UART packets     */
    FAULT_TYPE_PACKET_CORRUPT,  /* flips CRC bytes                 */
} fault_type_t;

typedef struct {
    fault_type_t type;
    bool         active;
    uint32_t     start_ms;
    uint32_t     duration_ms;  /* 0 = latched until cleared       */
    float        param;        /* type-specific: sag target V etc */
} fault_entry_t;

#define FAULT_SLOTS (8u)

typedef struct {
    fault_entry_t slots[FAULT_SLOTS];

    /* Pointers to sensor models for direct control */
    imu_model_t     *imu;
    gps_model_t     *gps;
    battery_model_t *battery;

    /* Derived output flags */
    bool comms_drop_active;
    bool packet_corrupt_active;
} fault_injector_t;

/* ------------------------------------------------------------------ */
/*  API                                                                 */
/* ------------------------------------------------------------------ */

void fault_init(fault_injector_t *f,
                imu_model_t     *imu,
                gps_model_t     *gps,
                battery_model_t *battery);

/* Inject a fault. duration_ms=0 → latched. Returns slot index or -1. */
int  fault_inject(fault_injector_t *f, fault_type_t type,
                  float param, uint32_t duration_ms, uint32_t now_ms);

/* Clear a specific fault slot */
void fault_clear(fault_injector_t *f, int slot);

/* Clear all active faults */
void fault_clear_all(fault_injector_t *f);

/* Called every sim tick — expires timed faults, applies to models */
void fault_tick(fault_injector_t *f, uint32_t now_ms);

/* Query active fault bitmask (mirrors FAULT_FLAG_* for the STM32) */
uint8_t fault_get_flags(const fault_injector_t *f);

#endif /* FAULT_INJECTOR_H */
