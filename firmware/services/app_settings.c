#include "app_settings.h"
#include "flash_proc.h"
#include "beeper.h"
#include "flight_snapshot.h"
#include "log_service.h"
#include <string.h>
static uav_settings_values_t current, saved, saving;
static uint8_t saved_known, save_state;
static uint32_t save_ticket;
void uav_settings_init(void) {
    uint8_t bytes[UAV_SETTINGS_BYTES];
    uav_settings_defaults(&current);
    saved_known=(uint8_t)(UAV_Read_Param_Settings(bytes) && uav_settings_decode(bytes,&current));
    saved=current;
    uav_beeper_set_mode((uint8_t)current.value[UAV_SETTING_SOUND]);
    uav_logf("INFO","SETTINGS","sound_mode=%u loaded=%u key=15 schema=1 boot_page=%u heater_enabled=%u target_deci_C=%u limit_pct=%u Kp_centi=%u Ki_milli=%u Kd_centi=%u",
        current.value[0],saved_known,current.value[1],current.value[2],current.value[4],current.value[3],current.value[5],current.value[6],current.value[7]);
}
void uav_settings_snapshot(uav_settings_snapshot_t *out) {
    taskENTER_CRITICAL();
    *out=(uav_settings_snapshot_t){current,(uint8_t)(!saved_known || memcmp(&current,&saved,sizeof(current))!=0),save_state};
    taskEXIT_CRITICAL();
}
int uav_settings_set_value(unsigned field, unsigned value) {
    if (field>=UAV_SETTING_COUNT || value<uav_settings_min(field) || value>uav_settings_max(field)) return -1;
    if (field>=UAV_SETTING_HEATER_ENABLED) {
        flight_snapshot_t flight; flight_snapshot_read(&flight);
        if (flight.state!=0 && !(field==UAV_SETTING_HEATER_ENABLED && value==0)) return -1;
    }
    if (current.value[field]==value) return 0;
    taskENTER_CRITICAL(); current.value[field]=(uint16_t)value;
    if (!save_ticket) save_state=UAV_SETTINGS_IDLE;
    taskEXIT_CRITICAL();
    if (field==UAV_SETTING_SOUND) uav_beeper_set_mode((uint8_t)value);
    uav_logf("INFO","SETTINGS","field=%u value=%u runtime=1",field,value);
    return 0;
}
void uav_settings_set_sound(uint8_t mode) { (void)uav_settings_set_value(UAV_SETTING_SOUND,mode); }
void uav_settings_restore_defaults(void) {
    flight_snapshot_t flight; flight_snapshot_read(&flight);
    if (flight.state!=0) return;
    uav_settings_values_t defaults; uav_settings_defaults(&defaults);
    for (unsigned i=0;i<UAV_SETTING_COUNT;i++) (void)uav_settings_set_value(i,defaults.value[i]);
}
int uav_settings_save(void) {
    if (save_ticket) return 0;
    if (saved_known && memcmp(&current,&saved,sizeof(current))==0) { save_state=UAV_SETTINGS_SAVED; return 0; }
    if (uav_storage_busy() || sensor_imu_calibrating() || sensors_mag_calibration_active() ||
        sensors_accel_calibration_active() || sbus_calibration_active()) {
        save_state=UAV_SETTINGS_FAILED; return -1;
    }
    uint8_t bytes[UAV_SETTINGS_BYTES]; uav_settings_encode(bytes,&current);
    if (UAV_Write_Param_Settings(bytes,&save_ticket)!=0) { save_state=UAV_SETTINGS_FAILED; return -1; }
    saving=current; save_state=UAV_SETTINGS_SAVING;
    return 0;
}
void uav_settings_restore_heater_defaults(void) {
    flight_snapshot_t flight; flight_snapshot_read(&flight);
    if (flight.state!=0) return;
    uav_settings_values_t defaults; uav_settings_defaults(&defaults);
    for (unsigned i=UAV_SETTING_HEATER_ENABLED;i<UAV_SETTING_COUNT;i++)
        (void)uav_settings_set_value(i,defaults.value[i]);
}
void uav_settings_poll(void) {
    if (!save_ticket) return;
    int result=uav_storage_result(save_ticket);
    if (!result) return;
    save_ticket=0;
    taskENTER_CRITICAL();
    if (result>0) { saved=saving; saved_known=1; save_state=UAV_SETTINGS_SAVED; }
    else save_state=UAV_SETTINGS_FAILED;
    taskEXIT_CRITICAL();
    uav_logf(result>0 ? "INFO":"ERROR","SETTINGS","save_verified=%u",(unsigned)(result>0));
}
