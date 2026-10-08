#ifndef STAR_GUI_DASHBOARD_H
#define STAR_GUI_DASHBOARD_H
#include "gui_canvas.h"
#include "gui_plot.h"
#include "gui_menu.h"
#define GUI_PAGE_COUNT 7
#define GUI_CHART_COUNT 6
enum {
    GUI_PAGE_OVERVIEW = 1, GUI_PAGE_ATTITUDE, GUI_PAGE_TRENDS, GUI_PAGE_SENSORS,
    GUI_PAGE_REMOTE, GUI_PAGE_FLOW, GUI_PAGE_HEALTH,
    GUI_PAGE_REMOTE_CAL = 19, GUI_PAGE_MAG_CAL = 20, GUI_PAGE_ACCEL_CAL=21, GUI_PAGE_HEATER=22
};
typedef enum { GUI_INPUT_NEXT = 1, GUI_INPUT_ENTER, GUI_INPUT_BACK, GUI_INPUT_CALIBRATION,
               GUI_INPUT_NEXT_PAGE, GUI_INPUT_NEXT_VIEW, GUI_INPUT_PREVIOUS } gui_input_t;
typedef enum { GUI_COMMAND_NONE, GUI_COMMAND_REMOTE_START, GUI_COMMAND_REMOTE_SAVE,
               GUI_COMMAND_MAG_START, GUI_COMMAND_MAG_CANCEL, GUI_COMMAND_ACCEL_START,
               GUI_COMMAND_ACCEL_CANCEL, GUI_COMMAND_REMOTE_CANCEL,
               GUI_COMMAND_SOUND_NORMAL, GUI_COMMAND_SOUND_QUIET, GUI_COMMAND_SOUND_MUTED,
               GUI_COMMAND_SETTINGS_SAVE, GUI_COMMAND_SETTINGS_DEFAULTS,
               GUI_COMMAND_SETTING_INCREASE, GUI_COMMAND_SETTING_DECREASE,
               GUI_COMMAND_SETTING_CANCEL, GUI_COMMAND_HEATER_TOGGLE } gui_command_t;
typedef enum { GUI_SCREEN_ROOT, GUI_SCREEN_PAGE, GUI_SCREEN_CALIBRATION,
               GUI_SCREEN_HELP, GUI_SCREEN_CONFIRM, GUI_SCREEN_MESSAGE,
               GUI_SCREEN_SETTINGS, GUI_SCREEN_SOUND, GUI_SCREEN_PARAMETERS, GUI_SCREEN_EDIT } gui_screen_t;
/* Presentation-only snapshot. Units: rad, rad/s, m/s2, uT, C, Pa, flow mm/s/mm.
 * No HAL, RTOS, global flight structures, control writes or dynamic allocation. */
typedef struct {
    uint32_t now_ms, heap_free, heap_min, log_dropped, uart_errors;
    float attitude[3], acc[3], gyro[3], mag[3];
    float acc_bias[3], gyro_bias[3], mag_bias[3], mag_scale[3];
    float temperature_c, pressure_pa, flow_velocity[3];
    int16_t flow_height_mm;
    uint16_t remote_raw[8], remote_pwm[8], remote_min[8], remote_max[8];
    uint8_t attitude_valid, state, rc_connected, rc_raw_connected, remote_calibrating;
    uint8_t rc_receiver_present, rc_parameters_valid, remote_saving, rc_invalid_ranges;
    uint8_t rc_ui_active, rc_ui_ready, remote_result;
    uint8_t sound_mode, settings_dirty, settings_save_state;
    uint16_t settings_value[8];
    float heater_temperature, heater_target, heater_duty, heater_p, heater_i, heater_d;
    uint8_t heater_state, heater_fault, heater_temperature_valid;
    uint8_t imu_calibrating, imu_cal_failed, fusion_mag_used, mag_calibrating, mag_calibration_step;
    uint8_t imu_ok, mag_ok, baro_ok, flash_ok, flow_valid, flow_quality;
    uint16_t mag_cal_samples, mag_cal_rms_permille;
    uint8_t mag_cal_coverage, mag_cal_reason;
    uint8_t mag_cal_hint, mag_cal_quality_ready, mag_sphere_ready, mag_sphere_fitted, mag_cursor_valid, mag_sphere_covered;
    uint32_t mag_sphere_mask;
    float mag_sphere_cursor[3], mag_sphere_goal[3];
    uint8_t accel_calibrating, accel_cal_faces, accel_cal_target, accel_cal_detected, accel_cal_phase, accel_cal_reason;
    uint16_t accel_cal_samples, accel_cal_rms_permille;
    float accel_cal_raw[3];
    gui_present_stats_t display_stats;
    uint16_t render_ms;
} gui_model_t;
typedef struct {
    gui_history_t history[GUI_CHART_COUNT];
    uint32_t last_sample_ms;
    uint8_t page, view, has_sample;
    gui_menu_t menu;
    uint8_t screen, help_page, confirm_command, message, message_return_screen;
    uint8_t edit_field, edit_return_screen, edit_selection;
    uint16_t edit_original;
} gui_dashboard_t;
void gui_dashboard_init(gui_dashboard_t *dashboard);
void gui_dashboard_set_page(gui_dashboard_t *dashboard, uint8_t page);
void gui_dashboard_next_page(gui_dashboard_t *dashboard);
void gui_dashboard_next_view(gui_dashboard_t *dashboard);
unsigned gui_dashboard_view_count(uint8_t page);
gui_command_t gui_dashboard_input(gui_dashboard_t *dashboard, gui_input_t input, const gui_model_t *model);
void gui_dashboard_update(gui_dashboard_t *dashboard, const gui_model_t *model);
void gui_dashboard_render(gui_dashboard_t *dashboard, gui_canvas_t *canvas, const gui_model_t *model);
#endif
