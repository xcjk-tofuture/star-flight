#ifndef UAV_LOG_SERVICE_H
#define UAV_LOG_SERVICE_H
#include <stdint.h>
#include <stddef.h>
#define UAV_LOG_BOOT_BYTES 8192u
#define UAV_LOG_QUEUE_BYTES 1024u
/* Task-context API, also usable before the scheduler starts. Emit never blocks.
 * Boot records retain their generation timestamps until the PC task sends them.
 * Runtime records use a bounded queue; full queues drop whole write chunks. */
int uav_log_init(void);
void uav_log_byte(uint8_t byte);
void uav_log_write(const uint8_t *bytes, size_t length);
void uav_logf(const char *level, const char *module, const char *format, ...);
uint32_t uav_log_dropped(void);
size_t uav_log_receive(uint8_t *bytes, size_t capacity, uint32_t timeout_ms);
#endif
