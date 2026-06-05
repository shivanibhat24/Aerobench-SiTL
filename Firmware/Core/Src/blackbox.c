#include "blackbox.h"
#include "cmsis_os.h"
#include <string.h>
#include <math.h>

/* Verify record size at compile time */
_Static_assert(sizeof(bb_record_t) == 64, "bb_record_t must be 64 bytes");

/* FreeRTOS mutex for thread safety */
static osMutexId_t s_mutex;
static osMutexAttr_t s_mutex_attr = { "bb_mutex", 0, NULL, 0 };

void blackbox_init(blackbox_t *bb)
{
    memset(bb, 0, sizeof(*bb));
    s_mutex = osMutexNew(&s_mutex_attr);
}

void blackbox_log(blackbox_t   *bb,
                  bb_event_type_t type,
                  flight_state_t  state,
                  uint8_t         fault_flags,
                  uint8_t         sensor_status,
                  float           voltage,
                  float           altitude,
                  float           accel_mag,
                  const char     *msg)
{
    osMutexAcquire(s_mutex, osWaitForever);

    bb_record_t *rec = &bb->entries[bb->write_idx];
    memset(rec, 0, sizeof(*rec));

    rec->timestamp_ms      = osKernelGetTickCount();
    rec->event_type        = (uint8_t)type;
    rec->flight_state      = (uint8_t)state;
    rec->fault_flags       = fault_flags;
    rec->sensor_status     = sensor_status;
    rec->battery_voltage_v = voltage;
    rec->altitude_m        = altitude;
    rec->accel_magnitude   = accel_mag;
    rec->sequence          = bb->sequence++;

    if (msg)
        strncpy(rec->message, msg, BLACKBOX_MSG_LEN - 1);

    /* Advance write pointer */
    bb->write_idx = (bb->write_idx + 1) % BLACKBOX_CAPACITY;

    if (bb->count < BLACKBOX_CAPACITY) {
        bb->count++;
    } else {
        /* Buffer full — overwrite oldest, advance read to stay valid */
        bb->overflow = true;
        bb->read_idx = (bb->read_idx + 1) % BLACKBOX_CAPACITY;
    }

    osMutexRelease(s_mutex);
}

bool blackbox_read(blackbox_t *bb, bb_record_t *out)
{
    osMutexAcquire(s_mutex, osWaitForever);

    bool available = (bb->count > 0) &&
                     (bb->read_idx != bb->write_idx || bb->overflow);

    if (available) {
        memcpy(out, &bb->entries[bb->read_idx], sizeof(*out));
        bb->read_idx = (bb->read_idx + 1) % BLACKBOX_CAPACITY;
        if (bb->count > 0) bb->count--;
        bb->overflow = false;
    }

    osMutexRelease(s_mutex);
    return available;
}

bool blackbox_peek(const blackbox_t *bb, uint16_t idx, bb_record_t *out)
{
    if (idx >= BLACKBOX_CAPACITY) return false;
    memcpy(out, &bb->entries[idx], sizeof(*out));
    return true;
}

uint16_t blackbox_count(const blackbox_t *bb)
{
    return bb->count;
}

void blackbox_rewind(blackbox_t *bb)
{
    osMutexAcquire(s_mutex, osWaitForever);

    /* Find oldest entry — it's just past write_idx when full, else 0 */
    if (bb->sequence >= BLACKBOX_CAPACITY)
        bb->read_idx = bb->write_idx;  /* oldest is one past current write */
    else
        bb->read_idx = 0;

    osMutexRelease(s_mutex);
}
