#ifndef UAV_DISPLAY_SERVICE_H
#define UAV_DISPLAY_SERVICE_H
#include <stdint.h>
typedef struct {
    uint32_t frames, spans, bytes, errors;
    uint16_t last_bytes, render_ms;
} uav_display_stats_t;
typedef enum { UAV_DISPLAY_NEXT, UAV_DISPLAY_CONFIRM, UAV_DISPLAY_BACK,
               UAV_DISPLAY_CAL_MENU } uav_display_input_t;
/* Task context; page requests coalesce, input events keep their order.
 * Only the display task owns menu/page state and dispatches GUI commands. */
void uav_display_request_page(uint8_t page);
void uav_display_next_page(void);
void uav_display_next_view(void);
void uav_display_get_stats(uav_display_stats_t *out);
void uav_display_input(uav_display_input_t input);
#endif
