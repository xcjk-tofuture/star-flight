#ifndef STAR_PROTOCOL_H
#define STAR_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>
#define STAR_VERSION 1u
#define STAR_PAYLOAD_MAX 128u
#define STAR_FRAME_MAX (STAR_PAYLOAD_MAX + 12u)
#define STAR_RX_TIMEOUT_MS 100u
enum { STAR_REQUEST = 0, STAR_RESPONSE = 1, STAR_EVENT = 2 };
enum {
    STAR_OK = 0,
    STAR_BAD_LENGTH = 1,
    STAR_UNSUPPORTED = 2,
    STAR_RANGE = 3,
    STAR_STATE = 4,
    STAR_BUSY = 5,
    STAR_INTERNAL = 6
};
enum {
    STAR_CMD_VERSION = 1,
    STAR_CMD_IDENTIFY = 2,
    STAR_CMD_CAPABILITIES = 3,
    STAR_CMD_STATUS = 4,
    STAR_CMD_PARAM_READ = 5,
    STAR_CMD_PARAM_WRITE = 6,
    STAR_CMD_CHASSIS_VELOCITY = 0x1000,
    STAR_CMD_CHASSIS_SAVE = 0x1001,
    STAR_CMD_CHASSIS_DIAGNOSTICS = 0x1002,
    STAR_CMD_UAV_ATTITUDE = 0x2000,
    STAR_CMD_UAV_DIAGNOSTICS = 0x2001
};
enum {
    STAR_CAP_STATUS = 1,
    STAR_CAP_TELEMETRY_PERIOD = 2,
    STAR_CAP_CHASSIS = 4,
    STAR_CAP_PARAMETER_STORE = 8,
    STAR_CAP_DIAGNOSTICS = 16
};
typedef struct {
    uint8_t flags;
    uint16_t sequence, command, length;
    uint8_t payload[STAR_PAYLOAD_MAX];
} star_frame_t;
typedef struct {
    uint8_t bytes[STAR_FRAME_MAX];
    uint16_t used;
    uint32_t last_ms, rejected, timed_out;
} star_parser_t;
/* All codec APIs are nonblocking, allocation-free and caller-owned.
 * One task owns each parser. Callback frame is valid only during callback.
 * now_ms must be a monotonic uint32 millisecond clock (wrap is supported). */
typedef void (*star_frame_fn)(void *context, const star_frame_t *frame);
void star_parser_init(star_parser_t *parser);
void star_parser_feed(star_parser_t *parser, const uint8_t *bytes, size_t length, uint32_t now_ms,
                      star_frame_fn callback, void *context);
void star_parser_expire(star_parser_t *parser, uint32_t now_ms);
/* Returns bytes written, or zero for invalid input / insufficient capacity. */
size_t star_encode(const star_frame_t *frame, uint8_t *output, size_t capacity);
uint16_t star_crc16(const uint8_t *bytes, size_t length);
uint16_t star_read_u16(const uint8_t *bytes);
uint32_t star_read_u32(const uint8_t *bytes);
void star_write_u16(uint8_t *bytes, uint16_t value);
void star_write_u32(uint8_t *bytes, uint32_t value);
float star_read_f32(const uint8_t *bytes);
void star_write_f32(uint8_t *bytes, float value);
#endif
