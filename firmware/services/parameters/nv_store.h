#ifndef UAV_NV_STORE_H
#define UAV_NV_STORE_H
#include <stddef.h>
#include <stdint.h>
#define NV_KEYS 16u
#define NV_PAYLOAD_MAX 128u
#define NV_HEADER_BYTES 32u
#define NV_RECORD_BYTES 160u
enum { NV_OK = 0, NV_EMPTY = 1, NV_IO = -1, NV_RANGE = -2 };
typedef struct {
    void *context;
    uint32_t bank_bytes;
    int (*read)(void *, unsigned, uint32_t, uint8_t *, size_t);
    int (*erase)(void *, unsigned);
    int (*program)(void *, unsigned, uint32_t, const uint8_t *, size_t);
} nv_io_t;
typedef struct {
    uint16_t schema, length;
    uint32_t sequence;
    uint32_t offset;
    uint8_t bank;
    uint8_t present, payload[NV_PAYLOAD_MAX];
} nv_value_t;
typedef struct {
    nv_io_t io;
    nv_value_t values[NV_KEYS];
    uint32_t generation, sequence, next_offset;
    int8_t active;
    uint8_t ready;
} nv_store_t;
/* One owner. Init only reads. Records and bank activation are committed last.
 * A transfer copies all latest keys before activating the alternate bank.
 * Failed writes require reinitialization; a torn slot is never overwritten. */
int nv_store_init(nv_store_t *store, const nv_io_t *io);
int nv_store_get(const nv_store_t *store, unsigned key, unsigned schema,
                 uint8_t *payload, size_t length, uint32_t *sequence);
int nv_store_put(nv_store_t *store, unsigned key, unsigned schema,
                 const uint8_t *payload, size_t length, uint32_t *sequence);
#endif
