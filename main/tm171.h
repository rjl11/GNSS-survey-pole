#pragma once
// TransducerM (TM171) EasyProtocol parser, Roll/Pitch/Yaw object only.
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define TM_CMD_RPY 35

typedef struct {
    uint8_t state, len, idx;
    uint8_t buf[64];            // length byte + payload + 2 CRC bytes
} tm_parser_t;

// Modbus-style CRC16, computed over the length byte + payload
static inline uint16_t tm_crc16(const uint8_t *d, int n)
{
    uint16_t c = 0xFFFF;
    for (int i = 0; i < n; i++) {
        c ^= d[i];
        for (int j = 0; j < 8; j++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1;
    }
    return c;
}

// Feed one byte. Returns true when a valid Roll/Pitch/Yaw package completed.
// Package: AA 55 | len | 4-byte info (cmd:7, res:3, from:11, to:11) | payload | CRC lo, hi
// RPY payload: u32 timestamp_us, float roll, pitch, yaw (degrees, little-endian)
static inline bool tm_feed(tm_parser_t *p, uint8_t b, float *roll, float *pitch,
                           float *yaw, uint32_t *ts_us, uint16_t *from_id)
{
    switch (p->state) {
    case 0: p->state = (b == 0xAA) ? 1 : 0; return false;
    case 1: p->state = (b == 0x55) ? 2 : (b == 0xAA ? 1 : 0); return false;
    case 2:
        if (b < 4 || b > sizeof(p->buf) - 3) { p->state = 0; return false; }
        p->len = b; p->buf[0] = b; p->idx = 1; p->state = 3; return false;
    default:
        p->buf[p->idx++] = b;
        if (p->idx < p->len + 3) return false;
        p->state = 0;
        uint16_t crc = p->buf[p->len + 1] | (p->buf[p->len + 2] << 8);
        if (crc != tm_crc16(p->buf, p->len + 1)) return false;
        uint32_t info; memcpy(&info, &p->buf[1], 4);
        if ((info & 0x7F) != TM_CMD_RPY || ((info >> 7) & 7) != 0 || p->len != 20) return false;
        if (from_id) *from_id = (info >> 10) & 0x7FF;
        memcpy(ts_us, &p->buf[5],  4);
        memcpy(roll,  &p->buf[9],  4);
        memcpy(pitch, &p->buf[13], 4);
        memcpy(yaw,   &p->buf[17], 4);
        return true;
    }
}
