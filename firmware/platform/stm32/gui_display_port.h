#ifndef STAR_GUI_DISPLAY_PORT_H
#define STAR_GUI_DISPLAY_PORT_H
#include <stdint.h>
typedef struct {
    uint32_t initializations, clock_hz, dma_errors, timeouts, last_hal_status;
    uint8_t configured, enabled;
} gui_display_port_stats_t;
int gui_display_port_init(void);
int gui_display_port_enable(void);
int gui_display_port_reassert(void);
void gui_display_port_get_stats(gui_display_port_stats_t *out);
int gui_display_port_write(void *context, uint8_t page, uint8_t x,
                            const uint8_t *bytes, uint8_t count);
#endif
