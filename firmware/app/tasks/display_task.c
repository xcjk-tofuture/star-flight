#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "display_service.h"
#include "gui_dashboard.h"
#include "gui_display_port.h"
#include "flight_snapshot.h"
#include "AHRS.h"
#include "sbus_proc.h"
#include "flow_proc.h"
#include "platform_time.h"
#include "serial_port.h"
#include "log_service.h"
#include "boot_log.h"
#include "gui_oled_config.h"
#include <string.h>
osThreadId OLEDTaskHandle;
extern uint8_t Bmi088Init_Flag, AK8975Flag, SPL06Flag;
static gui_canvas_t canvas;
static gui_presenter_t presenter;
static gui_dashboard_t dashboard;
static gui_model_t model;
static uav_display_stats_t published_stats;
static uint8_t requested_page, page_pending;
static uint8_t input_events[16], input_head, input_tail, input_count;
static void enqueue_input(gui_input_t input) {
    taskENTER_CRITICAL();
    if (input_count < sizeof(input_events)) {
        input_events[input_head] = (uint8_t)input;
        input_head = (uint8_t)((input_head+1) % sizeof(input_events)); input_count++;
    }
    taskEXIT_CRITICAL();
}
void uav_display_input(uav_display_input_t input) {
    switch (input) {
    case UAV_DISPLAY_NEXT: enqueue_input(GUI_INPUT_NEXT); break;
    case UAV_DISPLAY_CONFIRM: enqueue_input(GUI_INPUT_ENTER); break;
    case UAV_DISPLAY_BACK: enqueue_input(GUI_INPUT_BACK); break;
    case UAV_DISPLAY_CAL_MENU: enqueue_input(GUI_INPUT_CALIBRATION); break;
    }
}
void uav_display_request_page(uint8_t page) {
    taskENTER_CRITICAL(); requested_page = page; page_pending = 1; taskEXIT_CRITICAL();
}
void uav_display_next_page(void) {
    enqueue_input(GUI_INPUT_NEXT_PAGE);
}
void uav_display_next_view(void) {
    enqueue_input(GUI_INPUT_NEXT_VIEW);
}
void uav_display_get_stats(uav_display_stats_t *out) {
    if (!out) return;
    taskENTER_CRITICAL(); *out = published_stats; taskEXIT_CRITICAL();
}
static void process_navigation(void) {
    uint8_t page, set;
    taskENTER_CRITICAL();
    page = requested_page; set = page_pending; page_pending = 0;
    taskEXIT_CRITICAL();
    if (set) gui_dashboard_set_page(&dashboard, page);
    for (unsigned n = 0; n < sizeof(input_events); n++) {
        uint8_t input, have;
        taskENTER_CRITICAL();
        have = input_count != 0; input = have ? input_events[input_tail] : 0;
        if (have) { input_tail = (uint8_t)((input_tail+1) % sizeof(input_events)); input_count--; }
        taskEXIT_CRITICAL();
        if (!have) break;
        gui_command_t command = gui_dashboard_input(&dashboard, (gui_input_t)input, &model);
        if (command == GUI_COMMAND_REMOTE_START) sbus_request_calibration(0);
        else if (command == GUI_COMMAND_REMOTE_SAVE) sbus_request_calibration(1);
        else if (command == GUI_COMMAND_MAG_START) sensors_request_mag_calibration();
    }
}
static void read_model(void) {
    flight_snapshot_t flight;
    _imuData_all imu;
    _sbus_ch_cal_struct remote;
    _sbus_ch_struct raw;
    _flow_data flow;
    serial_port_stats_t uart;
    flight_snapshot_read(&flight); sensor_snapshot_read(&imu);
    sbus_snapshot(&remote); sbus_raw_snapshot(&raw); flow_snapshot(&flow);
    serial_port_get_stats(&uart);
    model.now_ms = platform_millis();
    model.attitude[0] = flight.roll_rad; model.attitude[1] = flight.pitch_rad; model.attitude[2] = flight.yaw_rad;
    model.attitude_valid = flight.valid && (uint32_t)(model.now_ms - flight.attitude_ms) <= 100u;
    model.state = flight.state;
    model.acc[0] = imu.acc.x; model.acc[1] = imu.acc.y; model.acc[2] = imu.acc.z;
    model.gyro[0] = imu.gyro.roll; model.gyro[1] = imu.gyro.pitch; model.gyro[2] = imu.gyro.yaw;
    model.mag[0] = imu.mag.x; model.mag[1] = imu.mag.y; model.mag[2] = imu.mag.z;
    model.acc_bias[0] = imu.accoffsetbias.x; model.acc_bias[1] = imu.accoffsetbias.y; model.acc_bias[2] = imu.accoffsetbias.z;
    model.gyro_bias[0] = imu.gyrooffsetbias.x; model.gyro_bias[1] = imu.gyrooffsetbias.y; model.gyro_bias[2] = imu.gyrooffsetbias.z;
    model.mag_bias[0] = imu.magoffsetbias.x; model.mag_bias[1] = imu.magoffsetbias.y; model.mag_bias[2] = imu.magoffsetbias.z;
    model.mag_scale[0] = imu.magscalebias.x; model.mag_scale[1] = imu.magscalebias.y; model.mag_scale[2] = imu.magscalebias.z;
    model.temperature_c = imu.f_temperature; model.pressure_pa = imu.Pressure;
    const uint16_t rc_raw[] = {raw.CH1, raw.CH2, raw.CH3, raw.CH4, raw.CH5, raw.CH6, raw.CH7, raw.CH8};
    const uint16_t rc_pwm[] = {remote.CAL_CH1, remote.CAL_CH2, remote.CAL_CH3, remote.CAL_CH4,
                               remote.CAL_CH5, remote.CAL_CH6, remote.CAL_CH7, remote.CAL_CH8};
    const uint16_t rc_min[] = {raw.CH1_MIN, raw.CH2_MIN, raw.CH3_MIN, raw.CH4_MIN,
                               raw.CH5_MIN, raw.CH6_MIN, raw.CH7_MIN, raw.CH8_MIN};
    const uint16_t rc_max[] = {raw.CH1_MAX, raw.CH2_MAX, raw.CH3_MAX, raw.CH4_MAX,
                               raw.CH5_MAX, raw.CH6_MAX, raw.CH7_MAX, raw.CH8_MAX};
    memcpy(model.remote_raw, rc_raw, sizeof(rc_raw)); memcpy(model.remote_pwm, rc_pwm, sizeof(rc_pwm));
    memcpy(model.remote_min, rc_min, sizeof(rc_min)); memcpy(model.remote_max, rc_max, sizeof(rc_max));
    model.rc_connected = remote.Connect_State;
    model.rc_raw_connected = raw.Connect_State;
    model.remote_calibrating = sbus_calibration_active();
    model.imu_calibrating = sensor_imu_calibrating(); model.mag_calibrating = sensors_mag_calibration_active();
    model.mag_calibration_step = sensor_calibration_step();
    model.imu_ok = !Bmi088Init_Flag; model.mag_ok = !AK8975Flag; model.baro_ok = !SPL06Flag;
    model.flash_ok = (uint8_t)app_boot_flash_ready();
    model.flow_valid = !!flow.flowFlag; model.flow_quality = flow.flowConf;
    model.flow_height_mm = flow.zNowHeight;
    model.flow_velocity[0] = flow.xFlowVel; model.flow_velocity[1] = flow.yFlowVel; model.flow_velocity[2] = flow.zFlowVel;
    model.heap_free = xPortGetFreeHeapSize(); model.heap_min = xPortGetMinimumEverFreeHeapSize();
    model.log_dropped = uav_log_dropped(); model.uart_errors = uart.start_errors + uart.timeouts + uart.dma_errors;
    model.display_stats = presenter.stats;
}
void OLED_Task_Proc(void const *argument) {
    (void)argument;
    gui_canvas_init(&canvas); gui_dashboard_init(&dashboard);
    gui_presenter_init(&presenter, gui_display_port_write, NULL);
    uint8_t configured = gui_display_port_init() == 0, panel_enabled = 0;
    uint32_t last_init_attempt = platform_millis(), last_repaint = platform_millis();
    if (!configured) { presenter.stats.errors++; uav_logf("WARN", "OLED", "initialization TX failed; retry pending"); }
    uav_logf("INFO", "GUI", "U8g2 128x64 Chinese_menu=12px rows=3 titles=12px pages=7 target=20Hz history=64@10Hz keys=2");
    TickType_t wake = xTaskGetTickCount();
    uint8_t mag_cal_was_active = 0;
    for (;;) {
        uint32_t begin = platform_millis();
        if (!configured && (uint32_t)(begin-last_init_attempt) >= 1000u) {
            configured = gui_display_port_init() == 0;
            last_init_attempt = platform_millis();
            gui_presenter_invalidate(&presenter);
            if (!configured) presenter.stats.errors++;
        }
        read_model(); process_navigation();
        if (model.mag_calibrating) mag_cal_was_active = 1;
        else if (mag_cal_was_active && dashboard.page == GUI_PAGE_MAG_CAL) {
            gui_dashboard_set_page(&dashboard, GUI_PAGE_OVERVIEW); mag_cal_was_active = 0;
        }
        gui_dashboard_update(&dashboard, &model);
        gui_dashboard_render(&dashboard, &canvas, &model);
        if (configured) {
            /* SPI OLED has no ACK/readback. Periodically reassert mapping and
             * repaint all pages rather than trusting a stale differential cache. */
            if (!panel_enabled || presenter.valid_pages != 0xffu ||
                (uint32_t)(begin-last_repaint) >= GUI_OLED_REPAINT_MS) {
                if (gui_display_port_reassert() != 0) presenter.stats.errors++;
                gui_presenter_invalidate(&presenter);
                last_repaint = begin;
            }
            gui_present(&presenter, &canvas);
            if (!panel_enabled && presenter.valid_pages == 0xffu) {
                if (gui_display_port_enable() == 0) {
                    panel_enabled = 1;
                    uav_logf("INFO", "OLED", "display_on=1 complete_first_frame=1 repaint=2000ms");
                } else presenter.stats.errors++;
            }
        }
        model.render_ms = (uint16_t)(platform_millis() - begin);
        taskENTER_CRITICAL();
        published_stats.frames = presenter.stats.frames;
        published_stats.spans = presenter.stats.spans;
        published_stats.bytes = presenter.stats.bytes;
        published_stats.errors = presenter.stats.errors;
        published_stats.last_bytes = presenter.stats.last_bytes;
        published_stats.render_ms = model.render_ms;
        taskEXIT_CRITICAL();
        if ((TickType_t)(xTaskGetTickCount() - wake) >= pdMS_TO_TICKS(50))
            wake = xTaskGetTickCount(); /* Do not burst redraws after a long shared-bus wait. */
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(50));
    }
}
