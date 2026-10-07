#include "nv_store.h"
#include "star_protocol.h"
#include <string.h>
#define BANK_MAGIC 0x3153564eu
#define RECORD_MAGIC 0x31524150u
#define COMMIT 0x54494d43u
static uint32_t crc(const uint8_t *b, size_t n) {
    uint32_t v=0xffffffffu;
    while (n--) {
        v^=*b++;
        for (unsigned i=0;i<8;i++) v=(v>>1)^(0xedb88320u & (0u-(v&1u)));
    }
    return ~v;
}
static int newer(uint32_t a, uint32_t b) { return (int32_t)(a-b)>0; }
static int blank(const uint8_t *b, size_t n) {
    while (n--) if (*b++!=0xff) return 0;
    return 1;
}
static int record_valid(const uint8_t *b) {
    unsigned key=star_read_u16(b+4), length=star_read_u16(b+8);
    return star_read_u32(b)==RECORD_MAGIC && key>=1 && key<=NV_KEYS &&
        length<=NV_PAYLOAD_MAX && star_read_u16(b+10)==0 &&
        star_read_u32(b+144)==crc(b,144) && star_read_u32(b+148)==COMMIT;
}
int nv_store_init(nv_store_t *s, const nv_io_t *io) {
    if (!s || !io || !io->read || !io->erase || !io->program ||
        io->bank_bytes < NV_HEADER_BYTES+(NV_KEYS+1)*NV_RECORD_BYTES) return NV_RANGE;
    memset(s,0,sizeof(*s)); s->io=*io; s->active=-1; s->next_offset=NV_HEADER_BYTES;
    uint32_t generation[2]={0}; int valid[2]={0}; uint8_t b[NV_RECORD_BYTES];
    uint8_t have_sequence=0;
    for (unsigned bank=0;bank<2;bank++) {
        if (io->read(io->context,bank,0,b,NV_HEADER_BYTES)) return NV_IO;
        generation[bank]=star_read_u32(b+8);
        valid[bank]=star_read_u32(b)==BANK_MAGIC && star_read_u32(b+4)==1 &&
            star_read_u32(b+12)==crc(b,12) && star_read_u32(b+16)==COMMIT;
    }
    if (valid[0] || valid[1]) {
        s->active=(int8_t)(valid[1] && (!valid[0] || newer(generation[1],generation[0])) ? 1:0);
        s->generation=generation[(unsigned)s->active];
        /* Read the older bank as a per-key fallback, then the active bank. */
        for (unsigned pass=0;pass<2;pass++) {
            unsigned bank=pass ? (unsigned)s->active:1u-(unsigned)s->active;
            if (!valid[bank]) continue;
            for (uint32_t off=NV_HEADER_BYTES;off+NV_RECORD_BYTES<=io->bank_bytes;off+=NV_RECORD_BYTES) {
                if (io->read(io->context,bank,off,b,sizeof(b))) return NV_IO;
                if ((int)bank==s->active && !blank(b,sizeof(b))) s->next_offset=off+NV_RECORD_BYTES;
                if (!record_valid(b)) continue;
                uint32_t seq=star_read_u32(b+12);
                nv_value_t *v=&s->values[star_read_u16(b+4)-1u];
                if (!v->present || newer(seq,v->sequence)) {
                    v->present=1; v->sequence=seq; v->schema=star_read_u16(b+6);
                    v->bank=(uint8_t)bank; v->offset=off;
                    v->length=star_read_u16(b+8); memcpy(v->payload,b+16,v->length);
                }
                if (!have_sequence || newer(seq,s->sequence)) { s->sequence=seq; have_sequence=1; }
            }
        }
    }
    s->ready=1;
    return NV_OK;
}
int nv_store_get(const nv_store_t *s, unsigned key, unsigned schema,
                 uint8_t *b, size_t n, uint32_t *seq) {
    if (!s || !s->ready) return NV_IO;
    if (key<1 || key>NV_KEYS || !b || n>NV_PAYLOAD_MAX) return NV_RANGE;
    const nv_value_t *v=&s->values[key-1];
    if (!v->present || v->schema!=schema || v->length!=n) return NV_EMPTY;
    memcpy(b,v->payload,n); if (seq) *seq=v->sequence;
    return NV_OK;
}
static int write_record(nv_store_t *s, unsigned bank, uint32_t off, unsigned key,
                        unsigned schema, const uint8_t *payload, size_t n, uint32_t seq) {
    uint8_t b[NV_RECORD_BYTES], check[NV_RECORD_BYTES]; memset(b,0xff,sizeof(b));
    star_write_u32(b,RECORD_MAGIC); star_write_u16(b+4,(uint16_t)key);
    star_write_u16(b+6,(uint16_t)schema); star_write_u16(b+8,(uint16_t)n);
    star_write_u16(b+10,0); star_write_u32(b+12,seq); memcpy(b+16,payload,n);
    star_write_u32(b+144,crc(b,144)); star_write_u32(b+148,COMMIT);
    if (s->io.program(s->io.context,bank,off,b,148) ||
        s->io.program(s->io.context,bank,off+148,b+148,4) ||
        s->io.read(s->io.context,bank,off,check,sizeof(check)) ||
        memcmp(b,check,sizeof(b))) return NV_IO;
    return NV_OK;
}
int nv_store_put(nv_store_t *s, unsigned key, unsigned schema,
                 const uint8_t *payload, size_t n, uint32_t *sequence) {
    if (!s || !s->ready) return NV_IO;
    if (key<1 || key>NV_KEYS || schema>65535 || !payload || n>NV_PAYLOAD_MAX) return NV_RANGE;
    nv_value_t *v=&s->values[key-1];
    if (v->present && v->schema==schema && v->length==n && !memcmp(v->payload,payload,n)) {
        uint8_t check[NV_RECORD_BYTES];
        if (s->io.read(s->io.context,v->bank,v->offset,check,sizeof(check)) || !record_valid(check) ||
            star_read_u16(check+4)!=key || star_read_u16(check+6)!=schema || star_read_u16(check+8)!=n ||
            star_read_u32(check+12)!=v->sequence || memcmp(check+16,payload,n)) goto failed;
        if (sequence) *sequence=v->sequence;
        return NV_OK;
    }
    uint32_t seq=s->sequence+1u, record_offset;
    if (s->active>=0 && s->next_offset+NV_RECORD_BYTES<=s->io.bank_bytes) {
        uint32_t off=s->next_offset; s->next_offset+=NV_RECORD_BYTES;
        record_offset=off;
        if (write_record(s,(unsigned)s->active,off,key,schema,payload,n,seq)) goto failed;
    } else {
        unsigned bank=s->active<0 ? 0:1u-(unsigned)s->active;
        uint32_t gen=s->generation+1u, off=NV_HEADER_BYTES;
        uint8_t h[NV_HEADER_BYTES], check[NV_HEADER_BYTES]; memset(h,0xff,sizeof(h));
        star_write_u32(h,BANK_MAGIC); star_write_u32(h+4,1); star_write_u32(h+8,gen);
        star_write_u32(h+12,crc(h,12)); star_write_u32(h+16,COMMIT);
        if (s->io.erase(s->io.context,bank) || s->io.program(s->io.context,bank,0,h,16)) goto failed;
        for (unsigned i=0;i<NV_KEYS;i++) {
            const nv_value_t *old=&s->values[i];
            if (!old->present || i+1==key) continue;
            if (write_record(s,bank,off,i+1,old->schema,old->payload,old->length,old->sequence)) goto failed;
            off+=NV_RECORD_BYTES;
        }
        record_offset=off;
        if (write_record(s,bank,off,key,schema,payload,n,seq) ||
            s->io.program(s->io.context,bank,16,h+16,4) ||
            s->io.read(s->io.context,bank,0,check,sizeof(check)) || memcmp(h,check,sizeof(h))) goto failed;
        s->active=(int8_t)bank; s->generation=gen; s->next_offset=off+NV_RECORD_BYTES;
        off=NV_HEADER_BYTES;
        for (unsigned i=0;i<NV_KEYS;i++) {
            nv_value_t *old=&s->values[i];
            if (!old->present || i+1==key) continue;
            old->bank=(uint8_t)bank; old->offset=off; off+=NV_RECORD_BYTES;
        }
    }
    v->present=1; v->schema=(uint16_t)schema; v->length=(uint16_t)n; v->sequence=seq;
    v->bank=(uint8_t)s->active; v->offset=record_offset;
    memcpy(v->payload,payload,n); s->sequence=seq;
    if (sequence) *sequence=seq;
    return NV_OK;
failed:
    s->ready=0;
    return NV_IO;
}
