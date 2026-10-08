#ifndef UAV_BEEPER_H
#define UAV_BEEPER_H
#include <stdint.h>
enum { UAV_BEEP_CLICK=0, UAV_BEEP_BACK, UAV_BEEP_ACCEPT, UAV_BEEP_START,
       UAV_BEEP_DONE, UAV_BEEP_LINK_OK, UAV_BEEP_ARM, UAV_BEEP_LOCK, UAV_BEEP_FAILURE };
void uav_beeper_init(void);
void uav_beeper_request(unsigned event);
void uav_beeper_alarm(uint8_t active);
/* Nonblocking, called by Key task. No task/heap/timer allocation. */
void uav_beeper_tick(uint32_t now_ms);
#endif
