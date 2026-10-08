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
#include "accel_calibration.h"
#include "rc_ui.h"
#include "beeper.h"
#include "flash_proc.h"
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
static rc_ui_t radio_ui;
static void dispatch_command(gui_command_t command) {
    if (command==GUI_COMMAND_REMOTE_START) sbus_request_calibration(0);
    else if (command==GUI_COMMAND_REMOTE_SAVE) sbus_request_calibration(1);
    else if (command==GUI_COMMAND_MAG_START) sensors_request_mag_calibration();
    else if (command==GUI_COMMAND_MAG_CANCEL) sensors_cancel_mag_calibration();
    else if (command==GUI_COMMAND_ACCEL_START) sensors_request_accel_calibration();
    else if (command==GUI_COMMAND_ACCEL_CANCEL) sensors_cancel_accel_calibration();
    else if (command==GUI_COMMAND_REMOTE_CANCEL) sbus_cancel_calibration();
}
static void dashboard_event(gui_input_t input) {
    uint8_t screen=dashboard.screen,page=dashboard.page,view=dashboard.view,selection=dashboard.menu.selected;
    gui_command_t command=gui_dashboard_input(&dashboard,input,&model);
    dispatch_command(command);
    int changed=screen!=dashboard.screen || page!=dashboard.page || view!=dashboard.view || selection!=dashboard.menu.selected;
    int cancel=command==GUI_COMMAND_REMOTE_CANCEL || command==GUI_COMMAND_MAG_CANCEL || command==GUI_COMMAND_ACCEL_CANCEL;
    if (dashboard.screen==GUI_SCREEN_MESSAGE && screen!=GUI_SCREEN_MESSAGE) uav_beeper_request(UAV_BEEP_FAILURE);
    else if (!cancel && (changed || command!=GUI_COMMAND_NONE))
        uav_beeper_request(input==GUI_INPUT_ENTER ? UAV_BEEP_ACCEPT:input==GUI_INPUT_BACK ? UAV_BEEP_BACK:UAV_BEEP_CLICK);
}
static void process_navigation(void) {
    uint8_t page,set;
    taskENTER_CRITICAL(); page=requested_page; set=page_pending; page_pending=0; taskEXIT_CRITICAL();
    if (set) gui_dashboard_set_page(&dashboard,page);
    for (unsigned n=0;n<sizeof(input_events);n++) {
        uint8_t input,have;
        taskENTER_CRITICAL();
        have=input_count!=0; input=have ? input_events[input_tail]:0;
        if (have) { input_tail=(uint8_t)((input_tail+1)%sizeof(input_events)); input_count--; }
        taskEXIT_CRITICAL();
        if (!have) break;
        dashboard_event((gui_input_t)input);
    }
}
static void process_radio(void) {
    rc_ui_frame_t frame; sbus_ui_snapshot(&frame);
    rc_ui_context_t context={.now_ms=model.now_ms,.state=model.state,.remote_calibrating=model.remote_calibrating,
        .remote_ranges_ready=model.rc_parameters_valid,.saving=(uint8_t)(uav_storage_busy() || model.remote_saving),
        .screen=dashboard.screen,.page=dashboard.page,
        .remote_capture_view=(uint8_t)(dashboard.screen==GUI_SCREEN_PAGE && dashboard.page==GUI_PAGE_REMOTE_CAL)};
    if (dashboard.screen==GUI_SCREEN_ROOT || dashboard.screen==GUI_SCREEN_CALIBRATION || dashboard.screen==GUI_SCREEN_CONFIRM) {
        context.select_count=dashboard.menu.count; context.select_index=dashboard.menu.selected;
    } else if (dashboard.screen==GUI_SCREEN_HELP) {
        context.select_count=2; context.select_index=dashboard.help_page;
    } else if (dashboard.screen==GUI_SCREEN_PAGE) {
        context.select_count=dashboard.page<=GUI_PAGE_COUNT ? GUI_PAGE_COUNT:0;
        context.select_index=dashboard.page<=GUI_PAGE_COUNT ? dashboard.page-1:0;
        context.view_count=(uint8_t)gui_dashboard_view_count(dashboard.page); context.view_index=dashboard.view;
    }
    rc_ui_action_t actions[4]; unsigned count=rc_ui_update(&radio_ui,&frame,&context,actions);
    model.rc_ui_active=radio_ui.active; model.rc_ui_ready=radio_ui.ready;
    for (unsigned i=0;i<count;i++) {
        rc_ui_action_t action=actions[i];
        if (action.kind==RC_UI_SELECT) {
            if (dashboard.screen!=context.screen || dashboard.page!=context.page) continue;
            if (dashboard.screen==GUI_SCREEN_ROOT || dashboard.screen==GUI_SCREEN_CALIBRATION || dashboard.screen==GUI_SCREEN_CONFIRM)
                gui_menu_select(&dashboard.menu,action.value);
            else if (dashboard.screen==GUI_SCREEN_HELP) dashboard.help_page=action.value%2;
            else if (dashboard.screen==GUI_SCREEN_PAGE && dashboard.page<=GUI_PAGE_COUNT)
                gui_dashboard_set_page(&dashboard,action.value+1);
            uav_beeper_request(UAV_BEEP_CLICK);
        } else if (action.kind==RC_UI_VIEW) {
            if (dashboard.screen!=context.screen || dashboard.page!=context.page) continue;
            if (dashboard.screen==GUI_SCREEN_PAGE && action.value<gui_dashboard_view_count(dashboard.page)) dashboard.view=action.value;
            uav_beeper_request(UAV_BEEP_CLICK);
        } else if (action.kind==RC_UI_SAVE_DIALOG) {
            dashboard_event(GUI_INPUT_BACK); dashboard_event(GUI_INPUT_ENTER);
        } else {
            gui_input_t input=action.kind==RC_UI_NEXT ? GUI_INPUT_NEXT:action.kind==RC_UI_PREVIOUS ? GUI_INPUT_PREVIOUS
                :action.kind==RC_UI_ENTER ? GUI_INPUT_ENTER:action.kind==RC_UI_BACK ? GUI_INPUT_BACK:GUI_INPUT_CALIBRATION;
            dashboard_event(input);
        }
    }
}
static void sound_status(void) {
    static uint8_t initialized,gyro,gyro_failed,mag,mag_phase,acc,acc_phase,faces,remote,remote_result,link;
    static uint32_t link_change_ms;
    if (initialized) {
        if ((model.mag_calibrating && !mag) || (model.accel_calibrating && !acc) || (model.remote_calibrating && !remote))
            uav_beeper_request(UAV_BEEP_START);
        if (!model.imu_calibrating && gyro && !model.accel_calibrating && !acc)
            uav_beeper_request(model.imu_cal_failed ? UAV_BEEP_FAILURE:UAV_BEEP_DONE);
        if (model.imu_cal_failed && !gyro_failed) uav_beeper_request(UAV_BEEP_FAILURE);
        if (model.mag_calibration_step!=mag_phase && model.mag_calibration_step==SENSOR_MAG_DONE)
            uav_beeper_request(UAV_BEEP_DONE);
        if (model.mag_calibration_step!=mag_phase && model.mag_calibration_step==SENSOR_MAG_FAILED)
            uav_beeper_request(UAV_BEEP_FAILURE);
        if (model.accel_cal_phase!=acc_phase && model.accel_cal_phase==UAV_ACCEL_DONE)
            uav_beeper_request(UAV_BEEP_DONE);
        if (model.accel_cal_phase!=acc_phase && model.accel_cal_phase==UAV_ACCEL_FAILED)
            uav_beeper_request(UAV_BEEP_FAILURE);
        if (model.accel_calibrating && model.accel_cal_faces!=faces)
            uav_beeper_request((model.accel_cal_faces & faces)==faces ? UAV_BEEP_ACCEPT:UAV_BEEP_FAILURE);
        if ((!model.mag_calibrating && mag && model.mag_calibration_step==SENSOR_MAG_COLLECT) ||
            (!model.accel_calibrating && acc && model.accel_cal_phase==UAV_ACCEL_PLACE))
            uav_beeper_request(UAV_BEEP_BACK);
        if (model.remote_result!=remote_result) {
            if (model.remote_result==2) uav_beeper_request(UAV_BEEP_DONE);
            else if (model.remote_result==3) uav_beeper_request(UAV_BEEP_FAILURE);
            else if (model.remote_result==4) uav_beeper_request(UAV_BEEP_BACK);
        }
        if (model.rc_raw_connected!=link && (uint32_t)(model.now_ms-link_change_ms)>=300u) {
            link=model.rc_raw_connected; link_change_ms=model.now_ms;
            uav_beeper_request(link ? UAV_BEEP_LINK_OK:UAV_BEEP_FAILURE);
        }
    } else { initialized=1; link=model.rc_raw_connected; link_change_ms=model.now_ms; }
    if (model.rc_raw_connected==link) link_change_ms=model.now_ms;
    gyro=model.imu_calibrating; gyro_failed=model.imu_cal_failed; mag=model.mag_calibrating;
    mag_phase=model.mag_calibration_step; acc=model.accel_calibrating; acc_phase=model.accel_cal_phase;
    faces=model.accel_cal_faces; remote=model.remote_calibrating; remote_result=model.remote_result;
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
    model.remote_saving=sbus_calibration_saving();
    model.remote_result=sbus_calibration_result();
    sbus_diagnostics_t rc_diag; sbus_diagnostics_read(&rc_diag);
    model.rc_receiver_present=rc_diag.receiver_present; model.rc_parameters_valid=rc_diag.calibrated;
    model.rc_invalid_ranges=rc_diag.invalid_ranges;
    model.imu_calibrating = sensor_imu_calibrating(); model.mag_calibrating = sensors_mag_calibration_active();
    model.imu_cal_failed=sensor_imu_calibration_failed();
    sensor_processing_stats_t processing;
    sensor_processing_stats_read(&processing);
    model.fusion_mag_used=processing.mag_used;
    sensor_mag_calibration_stats_t mag_cal;
    sensor_mag_calibration_stats_read(&mag_cal);
    model.mag_calibration_step=mag_cal.step; model.mag_cal_samples=mag_cal.samples;
    model.mag_cal_rms_permille=mag_cal.rms_permille; model.mag_cal_coverage=mag_cal.coverage;
    model.mag_cal_reason=mag_cal.reason;
    model.mag_cal_hint=mag_cal.hint; model.mag_cal_quality_ready=mag_cal.quality_ready;
    model.mag_sphere_ready=mag_cal.sphere_ready; model.mag_sphere_fitted=mag_cal.sphere_fitted;
    model.mag_cursor_valid=mag_cal.sphere_cursor_valid; model.mag_sphere_covered=mag_cal.sphere_covered;
    model.mag_sphere_mask=mag_cal.sphere_mask;
    memcpy(model.mag_sphere_cursor,mag_cal.sphere_cursor,sizeof(mag_cal.sphere_cursor));
    memcpy(model.mag_sphere_goal,mag_cal.sphere_goal,sizeof(mag_cal.sphere_goal));
    sensor_accel_calibration_stats_t accel_cal;
    sensor_accel_calibration_stats_read(&accel_cal);
    model.accel_calibrating=accel_cal.active; model.accel_cal_faces=accel_cal.faces;
    model.accel_cal_target=accel_cal.target; model.accel_cal_detected=accel_cal.detected;
    model.accel_cal_phase=accel_cal.phase; model.accel_cal_reason=accel_cal.reason;
    model.accel_cal_samples=accel_cal.samples; model.accel_cal_rms_permille=accel_cal.rms_permille;
    memcpy(model.accel_cal_raw,accel_cal.raw_acc,sizeof(accel_cal.raw_acc));
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
    uint8_t remote_cal_was_active = 0;
    uint8_t accel_cal_was_active=0;
    for (;;) {
        uint32_t begin = platform_millis();
        if (!configured && (uint32_t)(begin-last_init_attempt) >= 1000u) {
            configured = gui_display_port_init() == 0;
            last_init_attempt = platform_millis();
            gui_presenter_invalidate(&presenter);
            if (!configured) presenter.stats.errors++;
        }
        read_model(); sound_status(); process_navigation(); process_radio();
        if (model.remote_calibrating) remote_cal_was_active=1;
        else if (remote_cal_was_active) {
            if (dashboard.page==GUI_PAGE_REMOTE_CAL) gui_dashboard_set_page(&dashboard,GUI_PAGE_OVERVIEW);
            remote_cal_was_active=0;
        }
        if (model.mag_calibrating) mag_cal_was_active = 1;
        else if (mag_cal_was_active) {
            if (dashboard.page==GUI_PAGE_MAG_CAL && model.mag_calibration_step==SENSOR_MAG_DONE)
                gui_dashboard_set_page(&dashboard,GUI_PAGE_OVERVIEW);
            mag_cal_was_active=0;
        }
        if (model.accel_calibrating) accel_cal_was_active=1;
        else if (accel_cal_was_active) {
            if (dashboard.page==GUI_PAGE_ACCEL_CAL && model.accel_cal_phase==UAV_ACCEL_DONE)
                gui_dashboard_set_page(&dashboard,GUI_PAGE_OVERVIEW);
            accel_cal_was_active=0;
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
