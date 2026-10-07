#ifndef STAR_SERIAL_PORT_H
#define STAR_SERIAL_PORT_H
#include <stdint.h>
#include <stddef.h>
typedef struct {
    uint32_t frames_ok, start_errors, timeouts, dma_errors, last_hal_status;
} serial_port_stats_t;
/* One communication task owns UART TX. DMA owns a static copy until completion;
 * the task sleeps while waiting, with a 50ms completion timeout. */
int serial_port_send(const uint8_t *bytes, size_t length);
void serial_port_get_stats(serial_port_stats_t *out);
#endif
