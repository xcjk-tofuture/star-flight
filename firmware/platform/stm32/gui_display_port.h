#ifndef STAR_GUI_DISPLAY_PORT_H
#define STAR_GUI_DISPLAY_PORT_H
#include <stdint.h>
void gui_display_port_init(void);
int gui_display_port_write(void *context, uint8_t page, uint8_t x,
                            const uint8_t *bytes, uint8_t count);
#endif
