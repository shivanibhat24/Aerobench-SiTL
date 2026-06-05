#include "flight_fsm.h"
#include "blackbox.h"
#include "cmsis_os.h"   /* osKernelGetTickCount */
#include <math.h>
#include <string.h>
#include <stdio.h>

/* Forward declared — defined in main.c as a global */
extern blackbox_t g_blackbox;

/* ------------------------------------------------------------------ */
/*  Transition table                                                    */
/*  Legal transitions only — anything not listed is forbidden          */
/* ------------------------------------------------------------------ */

typedef struct {
    flight_state_t from;
    flight_state_t to;
    fsm_event_t    event;
} fsm_transition_t;

static const fsm_transition_t TRANSITIONS[] = {
    { FLIGHT_STATE_INIT,      FLIGHT_STATE_PREFLIGHT, FSM_EVT_SENSOR_OK      },
    { FLIGHT_STATE_PREFLIGHT, FLIGHT_STATE_ARMED,     FSM_EVT_ARMED_CMD      },
    { FLIGHT_STATE_ARMED,     FLIGHT_STATE_PREFLIGHT, FSM_EVT_DISARM_CMD     },
    { FLIGHT_STATE_ARMED,     FLIGHT_STATE_FLYING,    FSM_EVT_LIFTOFF        },
    { FLIGHT_STATE_FLYING,    FLIGHT_STATE_LANDING,   FSM_EVT_LAND_DETECT    },
    { FLIGHT_STATE_LANDING,   FLIGHT_STATE_PREFLIGHT, FSM_EVT_DISARM_CMD     },
    /* Any state → FAULT on fault detected */
    { FLIGHT_STATE_PREFLIGHT, FLIGHT_STATE_FAULT,     FSM_EVT_FAULT_DETECTED },
    { FLIGHT_STATE_ARMED,     FLIGHT_STATE_FAULT,     FSM_EVT_FAULT_DETECTED },
    { FLIGHT_STATE_FLYING,    FLIGHT_STATE_FAULT,     FSM_EVT_FAULT_DETECTED },
    { FLIGHT_STATE_LANDING,   FLIGHT_STATE_FAULT,     FSM_EVT_FAULT_DETECTED },
    /* Recovery from fault → preflight after fault cleared */
    { FLIGHT_STATE_FAULT,     FLIGHT_STATE_PREFLIGHT, FSM_EVT_FAULT_CLEARED  },
};
#define TRANSITION_COUNT (sizeof(TRANSITIONS) / sizeof(TRANSITIONS[0]))

/* ------------------------------------------------------------------ */
/*  Internal: do the state transition and log it                        */
/* ------------------------------------------------------------------ */

static void do_transition(fsm_ctx_t *ctx,
                           flight_state_t new_state,
                           fsm_event_t    event,
                           uint32_t       now_ms,
                           const sensor_data_t *sensors)
{
    ctx->previous_state = ctx->current_state;
    ctx->current_state  = new_state;
    ctx->last_event     = event;
    ctx->state_entry_ms = now_ms;
    ctx->transition_count++;

    /* Log to black-box */
    char msg[BLACKBOX_MSG_LEN];
    snprintf(msg, sizeof(msg), "%s->%s",
             FLIGHT_STATE_NAMES[ctx->previous_state],
             FLIGHT_STATE_NAMES[ctx->current_state]);

    float accel_mag = 0.0f;
    float voltage   = 0.0f;
    float altitude  = 0.0f;

    if (sensors) {
        accel_mag = sqrtf(sensors->accel_x * sensors->accel_x
                        + sensors->accel_y * sensors->accel_y
                        + sensors->accel_z * sensors->accel_z);
        voltage  = sensors->battery_voltage_v;
        altitude = sensors->altitude_m;
    }

    blackbox_log(&g_blackbox,
                 BB_EVT_STATE_CHANGE,
                 new_state,
                 (uint8_t)ctx->fault_flags,
                 sensors ? sensors->sensor_status : 0,
                 voltage,
                 altitude,
                 accel_mag,
                 msg);
}

/* ------------------------------------------------------------------ */
/*  Check whether a transition is legal per the table                  */
/* ------------------------------------------------------------------ */

static bool transition_legal(flight_state_t from,
                              flight_state_t to,
                              fsm_event_t    event)
{
    for (size_t i = 0; i < TRANSITION_COUNT; i++) {
        if (TRANSITIONS[i].from  == from &&
            TRANSITIONS[i].to    == to   &&
            TRANSITIONS[i].event == event)
            return true;
    }
    return false;
}

/* ------------------------------------------------------------------ */
/*  Guard evaluators per source state                                   */
/* ------------------------------------------------------------------ */

static bool guard_init_to_preflight(const sensor_data_t *s, uint32_t faults)
{
    (void)faults;
    /* All sensors reporting OK */
    return (s->sensor_status & FSM_PREFLIGHT_SENSOR_MASK) == FSM_PREFLIGHT_SENSOR_MASK;
}

static bool guard_preflight_to_armed(const sensor_data_t *s, uint32_t faults)
{
    if (faults != 0)                                          return false;
    if (s->battery_voltage_v < FSM_PREFLIGHT_BATT_MIN_V)     return false;
    if (s->gps_satellites    < FSM_PREFLIGHT_GPS_MIN_SATS)   return false;
    if (s->gps_fix_quality   == 0)                           return false;
    return true;
}

static bool guard_armed_to_flying(const sensor_data_t *s, uint32_t faults)
{
    (void)faults;
    /* Liftoff: vertical accel above threshold */
    float az = s->accel_z;
    return az > (9.81f * FSM_ARMED_ACCEL_THRESH_G);
}

static bool guard_flying_to_landing(const sensor_data_t *s, uint32_t faults)
{
    (void)faults;
    return (s->altitude_m      < FSM_LANDING_ALT_THRESH_M &&
            s->ground_speed_mps < FSM_LANDING_SPEED_THRESH);
}

static bool guard_any_to_fault(uint32_t faults)
{
    return faults != 0;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void fsm_init(fsm_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->current_state  = FLIGHT_STATE_INIT;
    ctx->previous_state = FLIGHT_STATE_INIT;
    ctx->state_entry_ms = 0;
}

bool fsm_tick(fsm_ctx_t *ctx,
              const sensor_data_t *sensors,
              uint32_t fault_flags,
              uint32_t now_ms)
{
    ctx->fault_flags = fault_flags;
    flight_state_t cur = ctx->current_state;
    bool transitioned  = false;

    /* Fault takes priority from any state except INIT */
    if (cur != FLIGHT_STATE_INIT && cur != FLIGHT_STATE_FAULT) {
        if (guard_any_to_fault(fault_flags)) {
            do_transition(ctx, FLIGHT_STATE_FAULT,
                          FSM_EVT_FAULT_DETECTED, now_ms, sensors);
            return true;
        }
    }

    switch (cur) {

    case FLIGHT_STATE_INIT:
        if (guard_init_to_preflight(sensors, fault_flags)) {
            do_transition(ctx, FLIGHT_STATE_PREFLIGHT,
                          FSM_EVT_SENSOR_OK, now_ms, sensors);
            transitioned = true;
        }
        break;

    case FLIGHT_STATE_PREFLIGHT:
        /* Auto-advance to ARMED only happens via fsm_command (explicit arm) */
        break;

    case FLIGHT_STATE_ARMED:
        if (guard_armed_to_flying(sensors, fault_flags)) {
            do_transition(ctx, FLIGHT_STATE_FLYING,
                          FSM_EVT_LIFTOFF, now_ms, sensors);
            transitioned = true;
        }
        break;

    case FLIGHT_STATE_FLYING:
        if (guard_flying_to_landing(sensors, fault_flags)) {
            do_transition(ctx, FLIGHT_STATE_LANDING,
                          FSM_EVT_LAND_DETECT, now_ms, sensors);
            transitioned = true;
        }
        break;

    case FLIGHT_STATE_LANDING:
        /* Auto-disarm once fully stopped handled by command from GCS */
        break;

    case FLIGHT_STATE_FAULT:
        /* Recover if all faults cleared */
        if (fault_flags == 0) {
            do_transition(ctx, FLIGHT_STATE_PREFLIGHT,
                          FSM_EVT_FAULT_CLEARED, now_ms, sensors);
            transitioned = true;
        }
        break;

    default:
        break;
    }

    return transitioned;
}

bool fsm_command(fsm_ctx_t *ctx, fsm_event_t event, uint32_t now_ms)
{
    flight_state_t target = FLIGHT_STATE_COUNT; /* invalid sentinel */

    switch (event) {
    case FSM_EVT_ARMED_CMD:   target = FLIGHT_STATE_ARMED;     break;
    case FSM_EVT_DISARM_CMD:  target = FLIGHT_STATE_PREFLIGHT; break;
    default: return false;
    }

    if (!transition_legal(ctx->current_state, target, event))
        return false;

    do_transition(ctx, target, event, now_ms, NULL);
    return true;
}
