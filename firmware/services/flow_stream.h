#ifndef UAV_FLOW_STREAM_H
#define UAV_FLOW_STREAM_H
#include <stddef.h>
#include <stdint.h>
typedef struct {
    uint32_t received_ms;
    int16_t integral_x, integral_y;
    uint16_t integration_us, range_mm;
    uint8_t byte10, byte11;
} uav_flow_frame_t;
typedef struct {
    uint8_t bytes[14], used;
    uint32_t last_ms, frames, rejected, discarded, gaps;
} uav_flow_stream_t;
typedef void (*uav_flow_frame_fn)(void *, const uav_flow_frame_t *);
/* FE 0A + 10 payload bytes + payload XOR + 55. One owner; no HAL/heap. */
void uav_flow_stream_feed(uav_flow_stream_t *stream, const uint8_t *bytes, size_t length,
                          uint32_t received_ms, uav_flow_frame_fn frame, void *context);
#endif
