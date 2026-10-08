#ifndef UAV_SETTINGS_RECORD_H
#define UAV_SETTINGS_RECORD_H
#include <stddef.h>
#include <stdint.h>
enum { UAV_SOUND_NORMAL=0, UAV_SOUND_QUIET, UAV_SOUND_MUTED };
enum { UAV_SETTING_SOUND=0, UAV_SETTING_BOOT_PAGE, UAV_SETTING_HEATER_ENABLED,
       UAV_SETTING_HEATER_LIMIT, UAV_SETTING_HEATER_TARGET, UAV_SETTING_HEATER_KP,
       UAV_SETTING_HEATER_KI, UAV_SETTING_HEATER_KD, UAV_SETTING_COUNT };
#define UAV_SETTINGS_BYTES 24u
typedef struct { uint16_t value[UAV_SETTING_COUNT]; } uav_settings_values_t;
/* Schema 1: first four fields u8, next four u16 LE, twelve reserved zero bytes. */
int uav_settings_record_valid(const uint8_t *bytes, size_t length);
void uav_settings_defaults(uav_settings_values_t *settings);
void uav_settings_encode(uint8_t bytes[UAV_SETTINGS_BYTES], const uav_settings_values_t *settings);
int uav_settings_decode(const uint8_t bytes[UAV_SETTINGS_BYTES], uav_settings_values_t *settings);
unsigned uav_settings_min(unsigned field);
unsigned uav_settings_max(unsigned field);
unsigned uav_settings_step(unsigned field);
#endif
