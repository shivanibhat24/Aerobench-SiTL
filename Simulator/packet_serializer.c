#include "packet_serialiser.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  CRC-16/CCITT — must be byte-for-byte identical to firmware         */
/* ------------------------------------------------------------------ */

uint16_t pkt_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000u)
                crc = (crc << 1) ^ 0x1021u;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ------------------------------------------------------------------ */
/*  Little-endian field serialisers (identical to firmware)            */
/* ------------------------------------------------------------------ */

static void put_f32(uint8_t *b, float v)
{
    uint32_t tmp;
    memcpy(&tmp, &v, 4);
    b[0] = (uint8_t)(tmp);
    b[1] = (uint8_t)(tmp >>  8);
    b[2] = (uint8_t)(tmp >> 16);
    b[3] = (uint8_t)(tmp >> 24);
}

static void put_f64(uint8_t *b, double v)
{
    uint64_t tmp;
    memcpy(&tmp, &v, 8);
    for (int i = 0; i < 8; i++) b[i] = (uint8_t)(tmp >> (8 * i));
}

static void put_u32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)(v);
    b[1] = (uint8_t)(v >>  8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}

/* ------------------------------------------------------------------ */
/*  Sensor frame layout (73 payload bytes — mirrors firmware exactly)  */
/* ------------------------------------------------------------------ */

#define SENSOR_PAYLOAD_SIZE (73u)
#define SENSOR_FRAME_SIZE   (PROTO_HEADER_SIZE + SENSOR_PAYLOAD_SIZE + PROTO_CRC_SIZE)

size_t pkt_build_sensor_frame(const sim_sensor_data_t *s,
                               uint8_t *buf, size_t buf_size)
{
    if (buf_size < SENSOR_FRAME_SIZE) return 0;

    /* --- Header --- */
    buf[0] = PROTO_SOF;
    buf[1] = PKT_TYPE_SENSOR_FRAME;
    buf[2] = (uint8_t)(SENSOR_PAYLOAD_SIZE & 0xFFu);
    buf[3] = (uint8_t)(SENSOR_PAYLOAD_SIZE >> 8);

    /* --- Payload (same field order as firmware proto_pack_sensor) --- */
    uint8_t *p = buf + PROTO_HEADER_SIZE;

    put_f32(p, s->accel_x);           p += 4;
    put_f32(p, s->accel_y);           p += 4;
    put_f32(p, s->accel_z);           p += 4;
    put_f32(p, s->gyro_x);            p += 4;
    put_f32(p, s->gyro_y);            p += 4;
    put_f32(p, s->gyro_z);            p += 4;
    put_f64(p, s->latitude);          p += 8;
    put_f64(p, s->longitude);         p += 8;
    put_f32(p, s->altitude_m);        p += 4;
    put_f32(p, s->ground_speed_mps);  p += 4;
    *p++ = s->gps_fix_quality;
    *p++ = s->gps_satellites;
    put_f32(p, s->battery_voltage_v); p += 4;
    put_f32(p, s->battery_current_a); p += 4;
    *p++ = s->battery_pct;
    put_f32(p, s->baro_pressure_hpa); p += 4;
    put_f32(p, s->temperature_c);     p += 4;
    put_u32(p, s->timestamp_ms);      p += 4;
    *p++ = s->sensor_status;

    /* Sanity check payload size */
    size_t written = (size_t)(p - (buf + PROTO_HEADER_SIZE));
    if (written != SENSOR_PAYLOAD_SIZE) return 0;

    /* --- CRC over TYPE(1) + LEN(2) + PAYLOAD --- */
    uint16_t crc = pkt_crc16(buf + 1, 3u + SENSOR_PAYLOAD_SIZE);
    buf[PROTO_HEADER_SIZE + SENSOR_PAYLOAD_SIZE]     = (uint8_t)(crc & 0xFFu);
    buf[PROTO_HEADER_SIZE + SENSOR_PAYLOAD_SIZE + 1] = (uint8_t)(crc >> 8);

    return SENSOR_FRAME_SIZE;
}

void pkt_corrupt_crc(uint8_t *frame, size_t frame_len)
{
    if (frame_len < 2) return;
    /* Flip the last two bytes (CRC position) */
    frame[frame_len - 2] ^= 0xFFu;
    frame[frame_len - 1] ^= 0xFFu;
}
