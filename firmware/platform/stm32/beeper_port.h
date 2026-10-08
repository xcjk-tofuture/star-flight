#ifndef UAV_BEEPER_PORT_H
#define UAV_BEEPER_PORT_H
#include <stdint.h>
int uav_beeper_port_init(void);
void uav_beeper_port_tone(uint16_t hz);
#endif
