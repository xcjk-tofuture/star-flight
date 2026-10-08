#ifndef UAV_SBUS_STREAM_H
#define UAV_SBUS_STREAM_H
#include <stddef.h>
#include <stdint.h>
#define UAV_SBUS_FRAME_BYTES 25u
typedef struct {
    uint8_t bytes[UAV_SBUS_FRAME_BYTES], used;
    uint32_t last_ms, frames, rejected, discarded, gaps;
} uav_sbus_stream_t;
typedef void (*uav_sbus_frame_fn)(void *context, const uint8_t frame[UAV_SBUS_FRAME_BYTES], uint32_t received_ms);
void uav_sbus_stream_feed(uav_sbus_stream_t *stream, const uint8_t *bytes, size_t count,
                         uint32_t received_ms, uav_sbus_frame_fn frame, void *context);
#endif
