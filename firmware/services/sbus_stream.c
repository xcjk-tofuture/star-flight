#include "sbus_stream.h"
#include <string.h>
static int valid(const uint8_t *b) {
    uint8_t end=b[24];
    return b[0]==0x0f && !(b[23]&0xf0) &&
        (end==0 || end==0x04 || end==0x14 || end==0x24 || end==0x34);
}
void uav_sbus_stream_feed(uav_sbus_stream_t *s, const uint8_t *b, size_t n,
                         uint32_t ms, uav_sbus_frame_fn frame, void *ctx) {
    if (s->used && (uint32_t)(ms-s->last_ms)>20u) { s->used=0; s->gaps++; }
    s->last_ms=ms;
    for (size_t i=0;i<n;i++) {
        if (!s->used && b[i]!=0x0f) { s->discarded++; continue; }
        s->bytes[s->used++]=b[i];
        if (s->used<UAV_SBUS_FRAME_BYTES) continue;
        if (valid(s->bytes)) {
            s->frames++; frame(ctx,s->bytes,ms); s->used=0;
        } else {
            s->rejected++;
            unsigned start=1; while (start<UAV_SBUS_FRAME_BYTES && s->bytes[start]!=0x0f) start++;
            s->used=(uint8_t)(UAV_SBUS_FRAME_BYTES-start);
            memmove(s->bytes,s->bytes+start,s->used);
        }
    }
}
