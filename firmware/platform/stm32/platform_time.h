#ifndef STAR_PLATFORM_TIME_H
#define STAR_PLATFORM_TIME_H
#include <stdint.h>
uint32_t platform_millis(void);
/* Sensor-owner monotonic microseconds; wrap-safe deltas. Call at least once
 * every cycle-counter wrap (25s at 168MHz). No timer or pin is repurposed. */
uint32_t platform_micros(void);
#endif
