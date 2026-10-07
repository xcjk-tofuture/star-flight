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
    GUI_PAGE_REMOTE_CAL = 19, GUI_PAGE_MAG_CAL = 20
};
typedef enum { GUI_INPUT_NEXT = 1, GUI_INPUT_ENTER, GUI_INPUT_BACK, GUI_INPUT_CALIBRATION,
               GUI_INPUT_NEXT_PAGE, GUI_INPUT_NEXT_VIEW } gui_input_t;
typedef enum { GUI_COMMAND_NONE, GUI_COMMAND_REMOTE_START, GUI_COMMAND_REMOTE_SAVE,
               GUI_COMMAND_MAG_START } gui_command_t;
typedef enum { GUI_SCREEN_ROOT, GUI_SCREEN_PAGE, GUI_SCREEN_CALIBRATION,
               GUI_SCREEN_HELP, GUI_SCREEN_CONFIRM, GUI_SCREEN_MESSAGE } gui_screen_t;
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
    uint8_t imu_calibrating, mag_calibrating, mag_calibration_step;
    uint8_t imu_ok, mag_ok, baro_ok, flash_ok, flow_valid, flow_quality;
    gui_present_stats_t display_stats;
    uint16_t render_ms;
} gui_model_t;
typedef struct {
    gui_history_t history[GUI_CHART_COUNT];
    uint32_t last_sample_ms;
    uint8_t page, view, has_sample;
    gui_menu_t menu;
    uint8_t screen, help_page, confirm_command, message;
} gui_dashboard_t;
void gui_dashboard_init(gui_dashboard_t *dashboard);
void gui_dashboard_set_page(gui_dashboard_t *dashboard, uint8_t page);
void gui_dashboard_next_page(gui_dashboard_t *dashboard);
void gui_dashboard_next_view(gui_dashboard_t *dashboard);
gui_command_t gui_dashboard_input(gui_dashboard_t *dashboard, gui_input_t input, const gui_model_t *model);
void gui_dashboard_update(gui_dashboard_t *dashboard, const gui_model_t *model);
void gui_dashboard_render(gui_dashboard_t *dashboard, gui_canvas_t *canvas, const gui_model_t *model);
#endif
