#include "param_journal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t memory[2][PJ_RECORD_BYTES], baseline[2][PJ_RECORD_BYTES];
static int budget = -1, read_fail;
static unsigned erases;
static int take(void) {
    if (budget == 0)
        return -1;
    if (budget > 0)
        budget--;
    return 0;
}
static int rd(void *ctx, unsigned slot, unsigned off, uint8_t *b, size_t n) {
    (void)ctx;
    if (read_fail)
        return -1;
    memcpy(b, memory[slot] + off, n);
    return 0;
}
static int erase(void *ctx, unsigned slot) {
    (void)ctx;
    erases++;
    for (unsigned i = 0; i < PJ_RECORD_BYTES; i++) {
        if (take())
            return -1;
        memory[slot][i] = 255;
    }
    return 0;
}
static int prog(void *ctx, unsigned slot, unsigned off, const uint8_t *b, size_t n) {
    (void)ctx;
    for (size_t i = 0; i < n; i++) {
        if (take())
            return -1;
        memory[slot][off + i] &= b[i];
    }
    return 0;
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
static void w32(uint8_t *p, uint32_t v) {
    for (unsigned i = 0; i < 4; i++)
        p[i] = (uint8_t)(v >> (8 * i));
}
int main(void) {
    pj_io_t io = {0, rd, erase, prog};
    pj_value_t value;
    uint32_t sequence;
    uint8_t a[128], b[128];
    memset(memory, 255, sizeof(memory));
    memset(a, 0x19, sizeof(a));
    memset(b, 0x75, sizeof(b));
    assert(pj_load(&io, 1, 1, &value) == PJ_EMPTY);
    assert(pj_save(&io, 1, 1, a, sizeof(a), &sequence) == PJ_OK && sequence == 1);
    memcpy(baseline, memory, sizeof(memory));
    unsigned before = erases;
    assert(pj_save(&io, 1, 1, a, sizeof(a), &sequence) == PJ_OK && erases == before);
    /* Reset after every byte of erase, body program, and commit-marker program. */
    for (int cut = 0; cut <= (int)(PJ_RECORD_BYTES * 2); cut++) {
        memcpy(memory, baseline, sizeof(memory));
        budget = cut;
        int result = pj_save(&io, 1, 1, b, sizeof(b), &sequence);
        budget = -1;
        assert(pj_load(&io, 1, 1, &value) == PJ_OK);
        assert(!memcmp(value.payload, result == PJ_OK ? b : a, sizeof(a)));
    }
    assert(pj_load(&io, 1, 2, &value) == PJ_EMPTY && pj_load(&io, 2, 1, &value) == PJ_EMPTY);
    assert(pj_save(&io, 1, 1, b, 129, &sequence) == PJ_RANGE);
    assert(pj_load(NULL, 1, 1, &value) == PJ_RANGE);
    read_fail = 1;
    before = erases;
    assert(pj_save(&io, 1, 1, b, 1, &sequence) == PJ_IO && erases == before);
    read_fail = 0;
    /* Sequence wrap and corruption fallback must choose the valid latest slot. */
    memcpy(memory, baseline, sizeof(memory));
    w32(memory[0] + 12, UINT32_MAX);
    w32(memory[0] + 144, crc(memory[0], 144));
    assert(pj_save(&io, 1, 1, b, 128, &sequence) == PJ_OK && sequence == 0);
    assert(pj_load(&io, 1, 1, &value) == PJ_OK && value.sequence == 0);
    memory[1][20] ^= 1;
    assert(pj_load(&io, 1, 1, &value) == PJ_OK && value.sequence == UINT32_MAX);
    puts("PASS journal: all 305 interrupted-write cut points, CRC fallback, version/kind, wrap, "
         "unchanged-write and read-failure guards");
    return 0;
}
