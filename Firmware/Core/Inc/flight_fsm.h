#ifndef FLIGHT_FSM_H
#define FLIGHT_FSM_H

#include "sensor_data.h"
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/*  Transition guard thresholds                                         */
/* ------------------------------------------------------------------ */

#define FSM_PREFLIGHT_BATT_MIN_V    (11.0f)   /* must have before ARMED   */
#define FSM_PREFLIGHT_GPS_MIN_SATS  (4u)
#define FSM_PREFLIGHT_SENSOR_MASK   (SENSOR_FLAG_ALL_OK)

#define FSM_ARMED_ACCEL_THRESH_G    (0.15f)   /* liftoff detect (≥ 0.15G) */
#define FSM_LANDING_ALT_THRESH_M    (0.5f)    /* considered grounded       */
#define FSM_LANDING_SPEED_THRESH    (0.3f)    /* m/s — considered stopped  */

/* ------------------------------------------------------------------ */
/*  Transition event type (written to black-box on every transition)   */
/* ------------------------------------------------------------------ */

typedef enum {
    FSM_EVT_SENSOR_OK       = 0x01,
    FSM_EVT_GPS_ACQUIRED    = 0x02,
    FSM_EVT_ARMED_CMD       = 0x03,
    FSM_EVT_LIFTOFF         = 0x04,
    FSM_EVT_LAND_DETECT     = 0x05,
    FSM_EVT_FAULT_DETECTED  = 0x06,
    FSM_EVT_FAULT_CLEARED   = 0x07,
    FSM_EVT_DISARM_CMD      = 0x08,
} fsm_event_t;

/* ------------------------------------------------------------------ */
/*  FSM context — owned by task_fsm, read (under mutex) by others      */
/* ------------------------------------------------------------------ */

typedef struct {
    flight_state_t  current_state;
    flight_state_t  previous_state;
    fsm_event_t     last_event;
    uint32_t        state_entry_ms;   /* tick when current state entered */
    uint32_t        fault_flags;      /* mirror of EventGroup bits        */
    uint32_t        transition_count;
} fsm_ctx_t;

/* ------------------------------------------------------------------ */
/*  API (called from task_fsm.c)                                        */
/* ------------------------------------------------------------------ */

/* Initialise FSM context — call once before scheduler starts */
void fsm_init(fsm_ctx_t *ctx);

/* Run one FSM tick given fresh sensor data and current fault flags.
 * Returns true if a state transition occurred. */
bool fsm_tick(fsm_ctx_t *ctx,
              const sensor_data_t *sensors,
              uint32_t fault_flags,
              uint32_t now_ms);

/* Force a state transition (used for arm/disarm commands from GCS).
 * Logs to black-box. Returns false if transition is illegal. */
bool fsm_command(fsm_ctx_t *ctx, fsm_event_t event, uint32_t now_ms);

/* Convenience: get current state safely (no mutex — caller must hold) */
static inline flight_state_t fsm_state(const fsm_ctx_t *ctx)
{
    return ctx->current_state;
}

#endif /* FLIGHT_FSM_H */
