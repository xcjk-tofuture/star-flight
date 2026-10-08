#include "gui_dashboard.h"
#include <assert.h>
#include <stdio.h>

static gui_dashboard_t dashboard;
static gui_model_t model;
static void open_calibration(void) {
    gui_dashboard_init(&dashboard);
    if (model.remote_calibrating) {
        gui_dashboard_set_page(&dashboard,GUI_PAGE_REMOTE_CAL);
        assert(gui_dashboard_input(&dashboard,GUI_INPUT_BACK,&model)==GUI_COMMAND_NONE);
    } else assert(gui_dashboard_input(&dashboard, GUI_INPUT_CALIBRATION, &model) == GUI_COMMAND_NONE);
}
static gui_command_t confirm_selection(void) {
    assert(dashboard.screen == GUI_SCREEN_CONFIRM);
    assert(gui_dashboard_input(&dashboard, GUI_INPUT_NEXT, &model) == GUI_COMMAND_NONE);
    return gui_dashboard_input(&dashboard, GUI_INPUT_ENTER, &model);
}
int main(void) {
    model.flash_ok = model.rc_raw_connected = model.mag_ok = model.attitude_valid = 1;
    open_calibration();
    assert(gui_dashboard_input(&dashboard, GUI_INPUT_ENTER, &model) == GUI_COMMAND_NONE);
    assert(confirm_selection() == GUI_COMMAND_REMOTE_START);
    for (unsigned state = 1; state <= 3; state++) {
        model.state = state;
        open_calibration();
        assert(gui_dashboard_input(&dashboard, GUI_INPUT_ENTER, &model) == GUI_COMMAND_NONE);
        assert(dashboard.screen == GUI_SCREEN_MESSAGE);
    }
    model.state = 0;
    open_calibration();
    gui_dashboard_input(&dashboard, GUI_INPUT_ENTER, &model);
    model.state = 1; /* State can change while the confirmation is open. */
    assert(confirm_selection() == GUI_COMMAND_NONE);
    assert(dashboard.screen == GUI_SCREEN_MESSAGE);
    model.state = 0;
    open_calibration();
    gui_dashboard_input(&dashboard, GUI_INPUT_NEXT, &model);
    gui_dashboard_input(&dashboard, GUI_INPUT_ENTER, &model);
    assert(confirm_selection() == GUI_COMMAND_MAG_START);
    model.remote_calibrating = 1;
    open_calibration();
    gui_dashboard_input(&dashboard, GUI_INPUT_ENTER, &model);
    assert(confirm_selection() == GUI_COMMAND_REMOTE_SAVE);
    model.flash_ok = 0;
    open_calibration();
    gui_dashboard_input(&dashboard, GUI_INPUT_ENTER, &model);
    assert(dashboard.screen == GUI_SCREEN_MESSAGE);
    model.flash_ok=1; model.imu_cal_failed=1;
    open_calibration();
    gui_dashboard_input(&dashboard,GUI_INPUT_NEXT,&model);
    gui_dashboard_input(&dashboard,GUI_INPUT_ENTER,&model);
    assert(dashboard.screen==GUI_SCREEN_MESSAGE);
    model.imu_cal_failed=0;
    /* Reading pages stays available in flight. */
    model.state = 2;
    gui_dashboard_init(&dashboard);
    gui_dashboard_input(&dashboard, GUI_INPUT_ENTER, &model);
    assert(dashboard.screen == GUI_SCREEN_PAGE && dashboard.page == GUI_PAGE_OVERVIEW);
    puts("PASS GUI: locked calibration, confirmation recheck, Flash guard and flight browsing");
    return 0;
}
