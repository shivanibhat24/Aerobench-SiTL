#include "fault_injector.h"
#include <string.h>
#include <stdio.h>

void fault_init(fault_injector_t *f,
                imu_model_t     *imu,
                gps_model_t     *gps,
                battery_model_t *battery)
{
    memset(f, 0, sizeof(*f));
    f->imu     = imu;
    f->gps     = gps;
    f->battery = battery;
}

int fault_inject(fault_injector_t *f, fault_type_t type,
                 float param, uint32_t duration_ms, uint32_t now_ms)
{
    /* Find a free slot */
    for (int i = 0; i < (int)FAULT_SLOTS; i++) {
        if (!f->slots[i].active) {
            f->slots[i].type        = type;
            f->slots[i].active      = true;
            f->slots[i].start_ms    = now_ms;
            f->slots[i].duration_ms = duration_ms;
            f->slots[i].param       = param;
            printf("[FAULT] Injected type=%d param=%.2f dur=%u ms (slot %d)\n",
                   type, param, duration_ms, i);
            return i;
        }
    }
    fprintf(stderr, "[FAULT] No free fault slots!\n");
    return -1;
}

void fault_clear(fault_injector_t *f, int slot)
{
    if (slot < 0 || slot >= (int)FAULT_SLOTS) return;
    f->slots[slot].active = false;
    f->slots[slot].type   = FAULT_TYPE_NONE;
    printf("[FAULT] Cleared slot %d\n", slot);
}

void fault_clear_all(fault_injector_t *f)
{
    for (int i = 0; i < (int)FAULT_SLOTS; i++)
        f->slots[i].active = false;

    /* Reset all sensor models to healthy */
    if (f->imu)     imu_freeze(f->imu, false);
    if (f->gps)     gps_set_signal_lost(f->gps, false);
    if (f->battery) battery_set_sag(f->battery, false, 0.0f);

    f->comms_drop_active    = false;
    f->packet_corrupt_active = false;

    printf("[FAULT] All faults cleared\n");
}

void fault_tick(fault_injector_t *f, uint32_t now_ms)
{
    /* Reset derived flags — recompute from active slots */
    bool gps_lost    = false;
    bool imu_frozen  = false;
    bool batt_sag    = false;
    bool comms_drop  = false;
    bool pkt_corrupt = false;
    float sag_v      = 10.0f;

    for (int i = 0; i < (int)FAULT_SLOTS; i++) {
        fault_entry_t *e = &f->slots[i];
        if (!e->active) continue;

        /* Auto-expire timed faults */
        if (e->duration_ms > 0 &&
            (now_ms - e->start_ms) >= e->duration_ms) {
            printf("[FAULT] Auto-expired slot %d (type=%d)\n", i, e->type);
            e->active = false;
            continue;
        }

        /* Accumulate active fault types */
        switch (e->type) {
        case FAULT_TYPE_GPS_LOSS:        gps_lost    = true;          break;
        case FAULT_TYPE_SENSOR_FREEZE:   imu_frozen  = true;          break;
        case FAULT_TYPE_BATTERY_SAG:     batt_sag    = true;
                                          sag_v = e->param;           break;
        case FAULT_TYPE_COMMS_DROP:      comms_drop  = true;          break;
        case FAULT_TYPE_PACKET_CORRUPT:  pkt_corrupt = true;          break;
        default: break;
        }
    }

    /* Apply to sensor models */
    if (f->gps)     gps_set_signal_lost(f->gps, gps_lost);
    if (f->imu)     imu_freeze(f->imu, imu_frozen);
    if (f->battery) battery_set_sag(f->battery, batt_sag, sag_v);

    f->comms_drop_active     = comms_drop;
    f->packet_corrupt_active = pkt_corrupt;
}

uint8_t fault_get_flags(const fault_injector_t *f)
{
    uint8_t flags = 0;
    for (int i = 0; i < (int)FAULT_SLOTS; i++) {
        if (!f->slots[i].active) continue;
        switch (f->slots[i].type) {
        case FAULT_TYPE_GPS_LOSS:       flags |= FAULT_GPS_LOSS;      break;
        case FAULT_TYPE_SENSOR_FREEZE:  flags |= FAULT_SENSOR_FREEZE; break;
        case FAULT_TYPE_BATTERY_SAG:    flags |= FAULT_BATT_LOW;      break;
        case FAULT_TYPE_COMMS_DROP:     flags |= FAULT_COMMS_TIMEOUT; break;
        default: break;
        }
    }
    return flags;
}
