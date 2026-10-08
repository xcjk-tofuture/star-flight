#ifndef UAV_APP_SETTINGS_H
#define UAV_APP_SETTINGS_H
#include "settings_record.h"
enum { UAV_SETTINGS_IDLE=0, UAV_SETTINGS_SAVING, UAV_SETTINGS_SAVED, UAV_SETTINGS_FAILED };
typedef struct { uav_settings_values_t values; uint8_t dirty, save_state; } uav_settings_snapshot_t;
/* Init before tasks. Display task owns subsequent updates and commands. */
void uav_settings_init(void);
void uav_settings_poll(void);
void uav_settings_snapshot(uav_settings_snapshot_t *out);
void uav_settings_set_sound(uint8_t mode);
int uav_settings_set_value(unsigned field, unsigned value);
void uav_settings_restore_defaults(void);
void uav_settings_restore_heater_defaults(void);
int uav_settings_save(void);
#endif
