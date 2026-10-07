#include "param_journal.h"
#include <string.h>
static uint16_t r16(const uint8_t *p) { return (uint16_t)(p[0] | (uint16_t)p[1] << 8); }
static uint32_t r32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static void w16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}
static void w32(uint8_t *p, uint32_t v) {
    for (unsigned i = 0; i < 4; i++)
        p[i] = (uint8_t)(v >> (8 * i));
}
static uint32_t crc(const uint8_t *p, size_t n) {
    uint32_t c = ~0u;
    while (n--) {
        c ^= *p++;
        for (unsigned i = 0; i < 8; i++)
            c = (c >> 1) ^ ((0u - (c & 1u)) & 0xedb88320u);
    }
    return ~c;
}
static int valid_io(const pj_io_t *io) { return io && io->read && io->erase && io->program; }
static int read_slot(const pj_io_t *io, unsigned slot, uint16_t kind, uint16_t schema,
                     pj_value_t *v) {
    uint8_t b[PJ_RECORD_BYTES];
    if (io->read(io->context, slot, 0, b, sizeof(b)))
        return PJ_IO;
    if (r32(b) != 0x31504a53u || r16(b + 4) != kind || r16(b + 6) != schema ||
        r16(b + 8) > PJ_PAYLOAD_MAX || r16(b + 10) != 0 || r32(b + 148) != 0x434f4d54u ||
        crc(b, 144) != r32(b + 144))
        return PJ_EMPTY;
    v->kind = kind;
    v->schema = schema;
    v->length = r16(b + 8);
    v->sequence = r32(b + 12);
    memcpy(v->payload, b + 16, PJ_PAYLOAD_MAX);
    return PJ_OK;
}
static int select_slot(const pj_io_t *io, uint16_t kind, uint16_t schema, pj_value_t *v,
                       unsigned *slot) {
    pj_value_t a = {0}, b = {0};
    int ra = read_slot(io, 0, kind, schema, &a), rb = read_slot(io, 1, kind, schema, &b);
    /* Never erase on an unreadable slot: its record could be the latest one. */
    if (ra == PJ_IO || rb == PJ_IO)
        return PJ_IO;
    if (ra != PJ_OK && rb != PJ_OK)
        return PJ_EMPTY;
    uint32_t delta = b.sequence - a.sequence;
    if (rb == PJ_OK && (ra != PJ_OK || (delta != 0 && delta < 0x80000000u))) {
        *v = b;
        *slot = 1;
    } else {
        *v = a;
        *slot = 0;
    }
    return PJ_OK;
}
int pj_load(const pj_io_t *io, uint16_t kind, uint16_t schema, pj_value_t *v) {
    unsigned slot;
    if (!valid_io(io) || !v)
        return PJ_RANGE;
    return select_slot(io, kind, schema, v, &slot);
}
int pj_save(const pj_io_t *io, uint16_t kind, uint16_t schema, const uint8_t *p, size_t n,
            uint32_t *sequence) {
    uint8_t b[PJ_RECORD_BYTES];
    pj_value_t old, verify;
    unsigned slot = 1;
    if (!valid_io(io) || !p || n > PJ_PAYLOAD_MAX)
        return PJ_RANGE;
    int result = select_slot(io, kind, schema, &old, &slot);
    if (result == PJ_IO)
        return PJ_IO;
    if (result == PJ_OK && old.length == n && !memcmp(old.payload, p, n)) {
        if (sequence)
            *sequence = old.sequence;
        return PJ_OK;
    }
    uint32_t seq = result == PJ_OK ? old.sequence + 1u : 1u;
    slot ^= 1u;
    memset(b, 0, sizeof(b));
    w32(b, 0x31504a53u);
    w16(b + 4, kind);
    w16(b + 6, schema);
    w16(b + 8, (uint16_t)n);
    w32(b + 12, seq);
    memcpy(b + 16, p, n);
    w32(b + 144, crc(b, 144));
    w32(b + 148, 0x434f4d54u);
    if (io->erase(io->context, slot) || io->program(io->context, slot, 0, b, 148) ||
        io->program(io->context, slot, 148, b + 148, 4))
        return PJ_IO;
    if (read_slot(io, slot, kind, schema, &verify) != PJ_OK || verify.sequence != seq ||
        verify.length != n || memcmp(verify.payload, p, n))
        return PJ_IO;
    if (sequence)
        *sequence = seq;
    return PJ_OK;
}
