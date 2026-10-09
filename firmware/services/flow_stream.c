#include "flow_stream.h"
#include <string.h>
static uint16_t word(const uint8_t *p) { return (uint16_t)(p[0]|((uint16_t)p[1]<<8)); }
static int16_t signed_word(const uint8_t *p) {
    uint16_t u=word(p); return (int16_t)(u>=32768u ? (int32_t)u-65536:(int32_t)u);
}
static int valid(const uint8_t *b) {
    uint8_t check=0; for (unsigned i=2;i<12;i++) check^=b[i];
    return b[0]==0xfe && b[1]==0x0a && b[12]==check && b[13]==0x55;
}
void uav_flow_stream_feed(uav_flow_stream_t *s, const uint8_t *b, size_t n,
                          uint32_t ms, uav_flow_frame_fn callback, void *ctx) {
    uav_flow_stream_feed_timed(s,b,n,ms,0,callback,ctx);
}
void uav_flow_stream_feed_timed(uav_flow_stream_t *s, const uint8_t *b, size_t n,
                                uint32_t ms, uint32_t us, uav_flow_frame_fn callback, void *ctx) {
    if (!s || !b || !callback) return;
    if (s->used && (uint32_t)(ms-s->last_ms)>50u) { s->used=0; s->gaps++; }
    s->last_ms=ms;
    for (size_t i=0;i<n;i++) {
        if (!s->used && b[i]!=0xfe) { s->discarded++; continue; }
        s->bytes[s->used++]=b[i];
        if (s->used<14) continue;
        if (valid(s->bytes)) {
            uint32_t trailing=(uint32_t)((n-i-1)*10000000u/115200u);
            uav_flow_frame_t f={.received_ms=us ? ms-trailing/1000u:ms,
                .integral_x=signed_word(s->bytes+2),.integral_y=signed_word(s->bytes+4),
                .integration_us=word(s->bytes+6),.range_mm=word(s->bytes+8),
                .byte10=s->bytes[10],.byte11=s->bytes[11],.received_us=us ? us-trailing:0};
            s->frames++; s->used=0; callback(ctx,&f);
        } else {
            s->rejected++;
            unsigned start=1; while (start<14 && s->bytes[start]!=0xfe) start++;
            s->used=(uint8_t)(14-start); memmove(s->bytes,s->bytes+start,s->used);
        }
    }
}
