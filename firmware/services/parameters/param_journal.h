#ifndef PARAM_JOURNAL_H
#define PARAM_JOURNAL_H
#include <stddef.h>
#include <stdint.h>
#define PJ_PAYLOAD_MAX 128u
#define PJ_RECORD_BYTES (24u + PJ_PAYLOAD_MAX)
enum { PJ_OK = 0, PJ_EMPTY = 1, PJ_IO = -1, PJ_RANGE = -2 };
typedef struct {
    void *context;
    int (*read)(void *, unsigned slot, unsigned offset, uint8_t *, size_t);
    int (*erase)(void *, unsigned slot);
    int (*program)(void *, unsigned slot, unsigned offset, const uint8_t *, size_t);
} pj_io_t;
typedef struct {
    uint16_t kind, schema, length;
    uint32_t sequence;
    uint8_t payload[PJ_PAYLOAD_MAX];
} pj_value_t;
/* One storage owner. No RTOS, heap or native structure serialization. Two separate
 * erase units are required. A new slot is sealed last; the previous slot survives
 * every failed erase/program. CRC32 covers identity, version, length and payload.
 * PJ_EMPTY means neither slot is a compatible, committed record. */
int pj_load(const pj_io_t *io, uint16_t kind, uint16_t schema, pj_value_t *value);
int pj_save(const pj_io_t *io, uint16_t kind, uint16_t schema, const uint8_t *payload,
            size_t length, uint32_t *sequence);
#endif
