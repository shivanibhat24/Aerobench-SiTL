#include "protocol.h"
#include <string.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  CRC-16/CCITT  (poly 0x1021, init 0xFFFF, no reflect)              */
/* ------------------------------------------------------------------ */

uint16_t proto_crc16(const uint8_t *data, size_t len)
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
/*  Internal helper: write a full wire frame into buf                  */
/*  header + payload + CRC, returns total bytes written                */
/* ------------------------------------------------------------------ */

static size_t build_frame(uint8_t type,
                           const uint8_t *payload,
                           uint16_t payload_len,
                           uint8_t *out_buf,
                           size_t out_size)
{
    size_t total = PROTO_HEADER_SIZE + payload_len + PROTO_CRC_SIZE;
    if (total > out_size) return 0;

    /* Header */
    out_buf[0] = PROTO_SOF;
    out_buf[1] = type;
    out_buf[2] = (uint8_t)(payload_len & 0xFFu);
    out_buf[3] = (uint8_t)(payload_len >> 8);

    /* Payload */
    memcpy(out_buf + PROTO_HEADER_SIZE, payload, payload_len);

    /* CRC covers TYPE + LEN(2) + PAYLOAD */
    uint16_t crc = proto_crc16(out_buf + 1, 3u + payload_len);
    out_buf[PROTO_HEADER_SIZE + payload_len]     = (uint8_t)(crc & 0xFFu);
    out_buf[PROTO_HEADER_SIZE + payload_len + 1] = (uint8_t)(crc >> 8);

    return total;
}

/* ------------------------------------------------------------------ */
/*  Sensor pack / unpack                                               */
/*  We serialise manually (no memcpy of struct) for portability        */
/*  across host/target endianness differences.                         */
/* ------------------------------------------------------------------ */

/* Little-endian helpers */
static void put_f32(uint8_t *b, float v)
{
    uint32_t tmp;
    memcpy(&tmp, &v, 4);
    b[0] = (uint8_t)(tmp);
    b[1] = (uint8_t)(tmp >> 8);
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
    b[1] = (uint8_t)(v >> 8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}

static float get_f32(const uint8_t *b)
{
    uint32_t tmp = (uint32_t)b[0]
                 | ((uint32_t)b[1] << 8)
                 | ((uint32_t)b[2] << 16)
                 | ((uint32_t)b[3] << 24);
    float v;
    memcpy(&v, &tmp, 4);
    return v;
}

static double get_f64(const uint8_t *b)
{
    uint64_t tmp = 0;
    for (int i = 0; i < 8; i++) tmp |= ((uint64_t)b[i]) << (8 * i);
    double v;
    memcpy(&v, &tmp, 8);
    return v;
}

static uint32_t get_u32(const uint8_t *b)
{
    return (uint32_t)b[0]
         | ((uint32_t)b[1] << 8)
         | ((uint32_t)b[2] << 16)
         | ((uint32_t)b[3] << 24);
}

/* Layout of a packed sensor frame (total = 73 bytes):
 *   accel_x[4] accel_y[4] accel_z[4]
 *   gyro_x[4]  gyro_y[4]  gyro_z[4]
 *   lat[8]     lon[8]
 *   altitude[4] speed[4]
 *   fix[1] sats[1]
 *   voltage[4] current[4] batt_pct[1]
 *   pressure[4] temp[4]
 *   timestamp[4] status[1]
 */
#define SENSOR_PACKED_SIZE  (73u)

size_t proto_pack_sensor(const sensor_data_t *s, uint8_t *buf, size_t buf_size)
{
    if (buf_size < SENSOR_PACKED_SIZE) return 0;
    uint8_t *p = buf;

    put_f32(p, s->accel_x);    p += 4;
    put_f32(p, s->accel_y);    p += 4;
    put_f32(p, s->accel_z);    p += 4;
    put_f32(p, s->gyro_x);     p += 4;
    put_f32(p, s->gyro_y);     p += 4;
    put_f32(p, s->gyro_z);     p += 4;
    put_f64(p, s->latitude);   p += 8;
    put_f64(p, s->longitude);  p += 8;
    put_f32(p, s->altitude_m); p += 4;
    put_f32(p, s->ground_speed_mps); p += 4;
    *p++ = s->gps_fix_quality;
    *p++ = s->gps_satellites;
    put_f32(p, s->battery_voltage_v);  p += 4;
    put_f32(p, s->battery_current_a);  p += 4;
    *p++ = s->battery_pct;
    put_f32(p, s->baro_pressure_hpa);  p += 4;
    put_f32(p, s->temperature_c);      p += 4;
    put_u32(p, s->timestamp_ms);       p += 4;
    *p++ = s->sensor_status;

    return (size_t)(p - buf);  /* should be SENSOR_PACKED_SIZE */
}

bool proto_unpack_sensor(const uint8_t *buf, size_t len, sensor_data_t *out)
{
    if (len < SENSOR_PACKED_SIZE) return false;
    const uint8_t *p = buf;

    out->accel_x           = get_f32(p); p += 4;
    out->accel_y           = get_f32(p); p += 4;
    out->accel_z           = get_f32(p); p += 4;
    out->gyro_x            = get_f32(p); p += 4;
    out->gyro_y            = get_f32(p); p += 4;
    out->gyro_z            = get_f32(p); p += 4;
    out->latitude          = get_f64(p); p += 8;
    out->longitude         = get_f64(p); p += 8;
    out->altitude_m        = get_f32(p); p += 4;
    out->ground_speed_mps  = get_f32(p); p += 4;
    out->gps_fix_quality   = *p++;
    out->gps_satellites    = *p++;
    out->battery_voltage_v = get_f32(p); p += 4;
    out->battery_current_a = get_f32(p); p += 4;
    out->battery_pct       = *p++;
    out->baro_pressure_hpa = get_f32(p); p += 4;
    out->temperature_c     = get_f32(p); p += 4;
    out->timestamp_ms      = get_u32(p); p += 4;
    out->sensor_status     = *p++;

    return true;
}

/* ------------------------------------------------------------------ */
/*  Build outbound telemetry wire frame                                 */
/* ------------------------------------------------------------------ */

size_t proto_build_telemetry(const sensor_data_t *s,
                              flight_state_t state,
                              uint8_t fault_flags,
                              uint8_t *out_buf,
                              size_t   out_size)
{
    pkt_telemetry_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.timestamp_ms      = s->timestamp_ms;
    pkt.flight_state      = (uint8_t)state;
    pkt.fault_flags       = fault_flags;
    pkt.altitude_m        = s->altitude_m;
    pkt.ground_speed_mps  = s->ground_speed_mps;
    pkt.battery_voltage_v = s->battery_voltage_v;
    pkt.battery_pct       = s->battery_pct;
    pkt.accel_x           = s->accel_x;
    pkt.accel_y           = s->accel_y;
    pkt.accel_z           = s->accel_z;
    pkt.gyro_x            = s->gyro_x;
    pkt.gyro_y            = s->gyro_y;
    pkt.gyro_z            = s->gyro_z;
    pkt.latitude          = s->latitude;
    pkt.longitude         = s->longitude;
    pkt.gps_fix_quality   = s->gps_fix_quality;
    pkt.gps_satellites    = s->gps_satellites;
    pkt.temperature_c     = s->temperature_c;
    pkt.sensor_status     = s->sensor_status;

    return build_frame(PKT_TYPE_TELEMETRY,
                       (const uint8_t *)&pkt, sizeof(pkt),
                       out_buf, out_size);
}

/* ------------------------------------------------------------------ */
/*  Build outbound health report wire frame                             */
/* ------------------------------------------------------------------ */

size_t proto_build_health(const pkt_health_report_t *h,
                           uint8_t *out_buf,
                           size_t   out_size)
{
    return build_frame(PKT_TYPE_HEALTH_REPORT,
                       (const uint8_t *)h, sizeof(*h),
                       out_buf, out_size);
}

/* ------------------------------------------------------------------ */
/*  RX byte-at-a-time state machine                                     */
/* ------------------------------------------------------------------ */

void proto_rx_reset(proto_rx_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->state = RX_STATE_WAIT_SOF;
}

bool proto_rx_feed(proto_rx_ctx_t *ctx, uint8_t byte)
{
    switch (ctx->state) {

    case RX_STATE_WAIT_SOF:
        if (byte == PROTO_SOF)
            ctx->state = RX_STATE_WAIT_TYPE;
        break;

    case RX_STATE_WAIT_TYPE:
        ctx->frame.type = byte;
        ctx->state = RX_STATE_WAIT_LEN_LO;
        break;

    case RX_STATE_WAIT_LEN_LO:
        ctx->frame.len = byte;
        ctx->state = RX_STATE_WAIT_LEN_HI;
        break;

    case RX_STATE_WAIT_LEN_HI:
        ctx->frame.len |= (uint16_t)byte << 8;
        if (ctx->frame.len == 0) {
            ctx->state = RX_STATE_WAIT_CRC_LO;
        } else if (ctx->frame.len > PROTO_MAX_PAYLOAD) {
            /* Oversized — resync */
            proto_rx_reset(ctx);
        } else {
            ctx->payload_idx = 0;
            ctx->state = RX_STATE_WAIT_PAYLOAD;
        }
        break;

    case RX_STATE_WAIT_PAYLOAD:
        ctx->frame.payload[ctx->payload_idx++] = byte;
        if (ctx->payload_idx >= ctx->frame.len)
            ctx->state = RX_STATE_WAIT_CRC_LO;
        break;

    case RX_STATE_WAIT_CRC_LO:
        ctx->crc_lo = byte;
        ctx->state  = RX_STATE_WAIT_CRC_HI;
        break;

    case RX_STATE_WAIT_CRC_HI: {
        uint16_t received_crc = (uint16_t)ctx->crc_lo | ((uint16_t)byte << 8);

        /* Re-compute CRC over TYPE(1) + LEN(2) + PAYLOAD */
        uint8_t hdr[3] = {
            ctx->frame.type,
            (uint8_t)(ctx->frame.len & 0xFF),
            (uint8_t)(ctx->frame.len >> 8)
        };
        uint16_t computed = proto_crc16(hdr, 3);
        computed = /* chain */ (void)computed, proto_crc16(hdr, 3);

        /* Simpler: feed header bytes then payload in one pass */
        uint8_t crc_buf[3 + PROTO_MAX_PAYLOAD];
        crc_buf[0] = ctx->frame.type;
        crc_buf[1] = (uint8_t)(ctx->frame.len & 0xFF);
        crc_buf[2] = (uint8_t)(ctx->frame.len >> 8);
        memcpy(crc_buf + 3, ctx->frame.payload, ctx->frame.len);
        uint16_t expected_crc = proto_crc16(crc_buf, 3u + ctx->frame.len);

        bool valid = (received_crc == expected_crc);
        proto_rx_reset(ctx);
        return valid;
    }

    default:
        proto_rx_reset(ctx);
        break;
    }

    return false;
}
