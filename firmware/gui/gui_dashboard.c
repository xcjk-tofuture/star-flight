#include "gui_dashboard.h"
#include "gui_scene.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define DEG_PER_RAD 57.29577951f
static const char *const titles[] = {"", "飞行总览", "三维姿态", "趋势曲线", "传感数据", "遥控通道", "光流测距", "系统诊断"};
enum { MENU_CALIBRATION = 100, MENU_HELP, MENU_BACK, MENU_CONFIRM, MENU_CANCEL,
       MENU_REMOTE_START, MENU_REMOTE_SAVE, MENU_MAG_START };
enum { MESSAGE_NONE, MESSAGE_NO_RC, MESSAGE_FLASH, MESSAGE_MAG, MESSAGE_IMU };
static const gui_menu_item_t root_items[] = {
    {"飞行总览", 1}, {"三维姿态", 2}, {"趋势曲线", 3}, {"传感数据", 4},
    {"遥控通道", 5}, {"光流测距", 6}, {"系统诊断", 7},
    {"校准管理", MENU_CALIBRATION}, {"按键说明", MENU_HELP}
};
static const gui_menu_item_t calibration_start[] = {
    {"遥控校准", MENU_REMOTE_START}, {"磁力校准", MENU_MAG_START}, {"返回菜单", MENU_BACK}
};
static const gui_menu_item_t calibration_save[] = {
    {"保存退出", MENU_REMOTE_SAVE}, {"磁力校准", MENU_MAG_START}, {"返回菜单", MENU_BACK}
};
static const gui_menu_item_t confirmation[] = {{"确认", MENU_CONFIRM}, {"取消", MENU_CANCEL}};
static void open_root(gui_dashboard_t *d, unsigned selection) {
    d->screen = GUI_SCREEN_ROOT;
    gui_menu_open(&d->menu, root_items, sizeof(root_items)/sizeof(root_items[0]), (uint8_t)selection);
}
static void open_calibration(gui_dashboard_t *d, const gui_model_t *m) {
    d->screen = GUI_SCREEN_CALIBRATION;
    gui_menu_open(&d->menu, m->remote_calibrating ? calibration_save : calibration_start, 3, 0);
}
static const char *status(const gui_model_t *m) {
    if (m->imu_calibrating) return "校准中";
    if (m->mag_calibrating) return "磁校中";
    if (m->remote_calibrating) return "遥控校准";
    if (!m->imu_ok) return "惯导异常";
    if (!m->attitude_valid) return "数据过期";
    if (!m->rc_connected) return "待遥控";
    return m->state == 3 ? "紧急" : m->state == 2 ? "飞行" : m->state == 1 ? "解锁" : "锁定";
}
static void number(gui_canvas_t *c, int x, int y, float value, unsigned decimals,
                   int sign, int valid, gui_font_t font) {
    char text[20];
    if (valid) gui_fixed(text, sizeof(text), value, decimals, sign);
    else snprintf(text, sizeof(text), "--");
    gui_text(c, x, y, text, font);
}
static void header(gui_canvas_t *c, const char *title, const char *badge) {
    gui_text(c, 2, 0, title, GUI_FONT_CN12);
    int width = gui_text_width(c, badge, GUI_FONT_CN12) + 6;
    gui_box(c, GUI_WIDTH-width, 0, width, 13, 1);
    gui_color(c, 0); gui_text(c, GUI_WIDTH-width+3, 0, badge, GUI_FONT_CN12); gui_color(c, 1);
    gui_line(c, 0, 14, GUI_WIDTH-1, 14);
}
static void footer(gui_canvas_t *c, const gui_dashboard_t *d) {
    gui_line(c, 0, 57, GUI_WIDTH-1, 57);
    const char *hint = d->page == GUI_PAGE_REMOTE_CAL ? "2VIEW 2HOLD CAL 1HOLD BACK"
                      : d->page > GUI_PAGE_COUNT ? "1HOLD MENU 2HOLD CAL" : "1PAGE 2VIEW 1HOLD MENU";
    gui_text(c, 1, 58, hint, GUI_FONT_TINY);
    if (d->page <= GUI_PAGE_COUNT)
        for (unsigned i = 1; i <= GUI_PAGE_COUNT; i++)
            gui_box(c, 91+(int)i*4, 60, 3, 3, i == d->page);
}
static unsigned view_count(uint8_t page) {
    if (page == GUI_PAGE_TRENDS) return GUI_CHART_COUNT;
    if (page == GUI_PAGE_SENSORS || page == GUI_PAGE_HEALTH) return 3;
    if (page == GUI_PAGE_REMOTE) return 4;
    if (page == GUI_PAGE_ATTITUDE || page == GUI_PAGE_FLOW ||
        page == GUI_PAGE_REMOTE_CAL) return 2;
    return 1;
}
void gui_dashboard_init(gui_dashboard_t *d) {
    memset(d, 0, sizeof(*d)); d->page = GUI_PAGE_OVERVIEW; open_root(d, 0);
}
void gui_dashboard_set_page(gui_dashboard_t *d, uint8_t page) {
    if ((page >= 1 && page <= GUI_PAGE_COUNT) || page == GUI_PAGE_REMOTE_CAL || page == GUI_PAGE_MAG_CAL) {
        d->page = page; d->view = 0; d->screen = GUI_SCREEN_PAGE;
    }
}
void gui_dashboard_next_page(gui_dashboard_t *d) {
    gui_dashboard_set_page(d, d->page >= GUI_PAGE_COUNT ? 1 : d->page + 1);
}
void gui_dashboard_next_view(gui_dashboard_t *d) {
    d->view = (uint8_t)((d->view + 1u) % view_count(d->page));
}
gui_command_t gui_dashboard_input(gui_dashboard_t *d, gui_input_t input, const gui_model_t *m) {
    if (d->screen == GUI_SCREEN_CALIBRATION)
        d->menu.items = m->remote_calibrating ? calibration_save : calibration_start;
    if (input == GUI_INPUT_NEXT_PAGE) { gui_dashboard_next_page(d); return GUI_COMMAND_NONE; }
    if (input == GUI_INPUT_NEXT_VIEW) { gui_dashboard_next_view(d); return GUI_COMMAND_NONE; }
    if (input == GUI_INPUT_BACK) {
        if (d->screen == GUI_SCREEN_ROOT) d->screen = GUI_SCREEN_PAGE;
        else if (d->screen == GUI_SCREEN_CONFIRM || d->screen == GUI_SCREEN_MESSAGE) open_calibration(d, m);
        else if (d->screen == GUI_SCREEN_PAGE && d->page > GUI_PAGE_COUNT) open_calibration(d, m);
        else open_root(d, d->screen == GUI_SCREEN_HELP ? 8 : d->screen == GUI_SCREEN_CALIBRATION ? 7 : d->page-1);
        return GUI_COMMAND_NONE;
    }
    if (input == GUI_INPUT_CALIBRATION) {
        if (d->screen != GUI_SCREEN_CONFIRM && d->screen != GUI_SCREEN_MESSAGE) open_calibration(d, m);
        return GUI_COMMAND_NONE;
    }
    if (d->screen == GUI_SCREEN_MESSAGE) { open_calibration(d, m); return GUI_COMMAND_NONE; }
    if (input == GUI_INPUT_NEXT) {
        if (d->screen == GUI_SCREEN_PAGE) gui_dashboard_next_page(d);
        else if (d->screen == GUI_SCREEN_HELP) d->help_page ^= 1;
        else gui_menu_next(&d->menu);
        return GUI_COMMAND_NONE;
    }
    if (input != GUI_INPUT_ENTER) return GUI_COMMAND_NONE;
    if (d->screen == GUI_SCREEN_PAGE) { gui_dashboard_next_view(d); return GUI_COMMAND_NONE; }
    if (d->screen == GUI_SCREEN_HELP) { open_root(d, 8); return GUI_COMMAND_NONE; }
    unsigned action = d->menu.items[d->menu.selected].action;
    if (d->screen == GUI_SCREEN_ROOT) {
        if (action <= GUI_PAGE_COUNT) gui_dashboard_set_page(d, (uint8_t)action);
        else if (action == MENU_CALIBRATION) open_calibration(d, m);
        else if (action == MENU_HELP) { d->screen = GUI_SCREEN_HELP; d->help_page = 0; }
    } else if (d->screen == GUI_SCREEN_CALIBRATION) {
        if (action == MENU_BACK) { open_root(d, 7); return GUI_COMMAND_NONE; }
        if (action == MENU_MAG_START && m->mag_calibrating) {
            gui_dashboard_set_page(d, GUI_PAGE_MAG_CAL); return GUI_COMMAND_NONE;
        }
        d->message = !m->flash_ok ? MESSAGE_FLASH
                     : action == MENU_REMOTE_START && !m->rc_raw_connected ? MESSAGE_NO_RC
                     : action == MENU_MAG_START && !m->mag_ok ? MESSAGE_MAG
                     : action == MENU_MAG_START && (!m->attitude_valid || m->imu_calibrating) ? MESSAGE_IMU : MESSAGE_NONE;
        if (d->message) { d->screen = GUI_SCREEN_MESSAGE; return GUI_COMMAND_NONE; }
        d->confirm_command = action == MENU_REMOTE_START ? GUI_COMMAND_REMOTE_START
                             : action == MENU_REMOTE_SAVE ? GUI_COMMAND_REMOTE_SAVE : GUI_COMMAND_MAG_START;
        d->screen = GUI_SCREEN_CONFIRM;
        gui_menu_open(&d->menu, confirmation, 2, 1); /* Cancel is the initial choice. */
    } else if (d->screen == GUI_SCREEN_CONFIRM) {
        if (action == MENU_CANCEL) { open_calibration(d, m); return GUI_COMMAND_NONE; }
        gui_command_t command = (gui_command_t)d->confirm_command;
        if (!m->flash_ok) { d->message = MESSAGE_FLASH; d->screen = GUI_SCREEN_MESSAGE; return GUI_COMMAND_NONE; }
        if (command == GUI_COMMAND_REMOTE_START && !m->rc_raw_connected) {
            d->message = MESSAGE_NO_RC; d->screen = GUI_SCREEN_MESSAGE; return GUI_COMMAND_NONE;
        }
        if (command == GUI_COMMAND_MAG_START && (!m->mag_ok || !m->attitude_valid || m->imu_calibrating)) {
            d->message = MESSAGE_IMU; d->screen = GUI_SCREEN_MESSAGE; return GUI_COMMAND_NONE;
        }
        gui_dashboard_set_page(d, command == GUI_COMMAND_REMOTE_SAVE ? 1
                                  : command == GUI_COMMAND_REMOTE_START ? GUI_PAGE_REMOTE_CAL : GUI_PAGE_MAG_CAL);
        return command;
    }
    return GUI_COMMAND_NONE;
}
void gui_dashboard_update(gui_dashboard_t *d, const gui_model_t *m) {
    uint32_t elapsed = (uint32_t)(m->now_ms - d->last_sample_ms);
    if (d->has_sample && elapsed < 100u) return;
    if (d->has_sample && elapsed > 500u) {
        const float empty[3] = {0};
        for (unsigned i = 0; i < GUI_CHART_COUNT; i++) gui_history_push(&d->history[i], empty, 0, 1);
    }
    d->last_sample_ms = m->now_ms; d->has_sample = 1;
    float v[3] = {m->attitude[0]*DEG_PER_RAD, m->attitude[1]*DEG_PER_RAD, 0};
    gui_history_push(&d->history[0], v, m->attitude_valid ? 3 : 0, 10);
    for (unsigned i = 0; i < 3; i++) v[i] = m->gyro[i]*DEG_PER_RAD;
    gui_history_push(&d->history[1], v, m->imu_ok ? 7 : 0, 1);
    gui_history_push(&d->history[2], m->mag, m->mag_ok ? 7 : 0, 10);
    v[0] = m->temperature_c; v[1] = v[2] = 0;
    gui_history_push(&d->history[3], v, m->imu_ok ? 1 : 0, 10);
    v[0] = m->pressure_pa / 100.0f;
    gui_history_push(&d->history[4], v, m->baro_ok && m->pressure_pa > 0 ? 1 : 0, 10);
    v[0] = m->flow_velocity[0]; v[1] = m->flow_velocity[1];
    gui_history_push(&d->history[5], v, m->flow_valid ? 3 : 0, .1f);
}
static void overview(gui_canvas_t *c, const gui_model_t *m) {
    gui_text(c, 2, 16, "ROLL deg", GUI_FONT_TINY);
    gui_text(c, 68, 16, "PITCH deg", GUI_FONT_TINY);
    number(c, 2, 24, m->attitude[0]*DEG_PER_RAD, 1, 1, m->attitude_valid, GUI_FONT_LARGE);
    number(c, 68, 24, m->attitude[1]*DEG_PER_RAD, 1, 1, m->attitude_valid, GUI_FONT_LARGE);
    gui_line(c, 63, 14, 63, 37);
    gui_text(c, 2, 40, "Y", GUI_FONT_BODY);
    number(c, 12, 40, m->attitude[2]*DEG_PER_RAD, 1, 1, m->attitude_valid, GUI_FONT_BODY);
    number(c, 88, 40, m->temperature_c, 1, 0, m->imu_ok, GUI_FONT_BODY);
    gui_text(c, 79, 40, "T", GUI_FONT_TINY);
    gui_text(c, 2, 50, m->mag_ok ? "9AXIS" : "6AXIS", GUI_FONT_TINY);
    gui_text(c, 38, 50, m->baro_ok ? "BARO OK" : "BARO ERR", GUI_FONT_TINY);
    gui_text(c, 84, 50, m->flash_ok ? "FLASH OK" : "FLASH ERR", GUI_FONT_TINY);
}
static void attitude(gui_canvas_t *c, const gui_model_t *m, unsigned view) {
    u8g2_SetClipWindow(&c->graphics, 0, 16, 82, 56);
    if (m->attitude_valid) {
        if (!view) gui_drone_3d(c, m->attitude[0], m->attitude[1], m->attitude[2], 40, 34, 22);
        else gui_horizon(c, m->attitude[0], m->attitude[1], 2, 16, 78, 38);
    } else gui_text(c, 12, 28, m->imu_calibrating ? "校准中" : "等待姿态", GUI_FONT_CN12);
    u8g2_SetMaxClipWindow(&c->graphics);
    gui_line(c, 82, 16, 82, 54);
    static const char *const labels[] = {"R", "P", "Y"};
    for (unsigned i = 0; i < 3; i++) {
        gui_text(c, 85, 16+(int)i*14, labels[i], GUI_FONT_TINY);
        number(c, 91, 16+(int)i*14, m->attitude[i]*DEG_PER_RAD, 1, 1,
               m->attitude_valid, GUI_FONT_TINY);
    }
    gui_text(c, 85, 49, view ? "HORIZON" : "3D / deg", GUI_FONT_TINY);
}
static void trends(gui_canvas_t *c, const gui_dashboard_t *d, const gui_model_t *m) {
    static const char *const legends[] = {"ROLL -  PITCH :", "GYRO X- Y: Z.", "MAG X- Y: Z.",
                                          "TEMP C", "BARO hPa", "FLOW X- Y: mm/s"};
    unsigned kind = d->view % GUI_CHART_COUNT;
    unsigned channels = kind == 0 || kind == 5 ? 2 : kind == 1 || kind == 2 ? 3 : 1;
    int low, high;
    if (!kind) { low = -600; high = 600; }
    else gui_history_range(&d->history[kind], channels, kind == 2 ? 400 : kind == 1 ? 40 : 20, &low, &high);
    float factor = kind == 1 ? 1 : kind == 5 ? 10 : .1f;
    number(c, 0, 24, high*factor, 0, 0, 1, GUI_FONT_TINY);
    number(c, 0, 44, low*factor, 0, 0, 1, GUI_FONT_TINY);
    gui_text(c, 2, 16, legends[kind], GUI_FONT_TINY);
    gui_text(c, 101, 16, "64pt", GUI_FONT_TINY);
    gui_plot(c, &d->history[kind], 23, 23, 105, 26, channels, low, high);
    float value = kind == 0 ? m->attitude[0]*DEG_PER_RAD
                : kind == 1 ? m->gyro[0]*DEG_PER_RAD : kind == 2 ? m->mag[0]
                : kind == 3 ? m->temperature_c : kind == 4 ? m->pressure_pa / 100 : m->flow_velocity[0];
    int valid = kind == 0 ? m->attitude_valid : kind == 2 ? m->mag_ok
               : kind == 4 ? m->baro_ok : kind == 5 ? m->flow_valid : m->imu_ok;
    gui_text(c, 2, 50, "NOW", GUI_FONT_TINY);
    number(c, 20, 50, value, 1, 0, valid, GUI_FONT_TINY);
    gui_text(c, 83, 50, "K2 METRIC", GUI_FONT_TINY);
}
static void vector_row(gui_canvas_t *c, int y, const char *label, const float v[3], int valid,
                       float scale, unsigned precision) {
    gui_text(c, 2, y, label, GUI_FONT_TINY);
    for (unsigned i = 0; i < 3; i++)
        number(c, 23+(int)i*35, y, v[i]*scale, precision, 1, valid, GUI_FONT_TINY);
}
static void sensors(gui_canvas_t *c, const gui_model_t *m, unsigned view) {
    gui_text(c, 26, 16, "X", GUI_FONT_TINY); gui_text(c, 62, 16, "Y", GUI_FONT_TINY);
    gui_text(c, 99, 16, "Z", GUI_FONT_TINY);
    if (!view) {
        vector_row(c, 23, "ACC", m->acc, m->imu_ok, 1, 2);
        vector_row(c, 33, "GYR", m->gyro, m->imu_ok, DEG_PER_RAD, 1);
        vector_row(c, 43, "MAG", m->mag, m->mag_ok, 1, 1);
        gui_text(c, 2, 50, "m/s2   deg/s   uT", GUI_FONT_TINY);
    } else if (view == 1) {
        vector_row(c, 23, "ACC", m->acc_bias, m->imu_ok, 1, 2);
        vector_row(c, 33, "GYR", m->gyro_bias, m->imu_ok, DEG_PER_RAD, 2);
        vector_row(c, 43, "MAG", m->mag_bias, m->mag_ok, 1, 1);
        gui_text(c, 2, 50, "CALIBRATION OFFSETS", GUI_FONT_TINY);
    } else {
        vector_row(c, 23, "M-S", m->mag_scale, m->mag_ok, 1, 2);
        gui_text(c, 2, 35, "TEMP C", GUI_FONT_TINY);
        number(c, 72, 35, m->temperature_c, 1, 0, m->imu_ok, GUI_FONT_BODY);
        gui_text(c, 2, 47, "BARO hPa", GUI_FONT_TINY);
        number(c, 72, 47, m->pressure_pa/100, 1, 0, m->baro_ok, GUI_FONT_TINY);
    }
}
static void remote(gui_canvas_t *c, const gui_model_t *m, unsigned view, int calibration) {
    unsigned first = view % 2 ? 4 : 0;
    int raw_view = !calibration && view >= 2;
    int connected = calibration || raw_view ? m->rc_raw_connected : m->rc_connected;
    if (calibration) {
        gui_text(c, 25, 16, "MIN", GUI_FONT_TINY);
        gui_text(c, 62, 16, "NOW", GUI_FONT_TINY);
        gui_text(c, 99, 16, "MAX", GUI_FONT_TINY);
    }
    for (unsigned i = 0; i < 4; i++) {
        unsigned ch = first+i;
        int y = calibration ? 23+(int)i*8 : 16+(int)i*10;
        char label[8];
        snprintf(label, sizeof(label), "CH%u", ch+1);
        gui_text(c, 2, y, label, GUI_FONT_TINY);
        if (!calibration) {
            int value = raw_view ? m->remote_raw[ch] : m->remote_pwm[ch];
            gui_bar(c, 21, y+1, 57, 7, connected ? value : 0, raw_view ? 0 : 1000, raw_view ? 2047 : 2000);
            number(c, 83, y, value, 0, 0, connected, GUI_FONT_BODY);
            if (!connected) gui_dashed_line(c, 23, y+4, 75, y+4, 3);
        } else {
            number(c, 25, y, m->remote_min[ch], 0, 0, connected, GUI_FONT_TINY);
            number(c, 62, y, m->remote_raw[ch], 0, 0, connected, GUI_FONT_TINY);
            number(c, 99, y, m->remote_max[ch], 0, 0, connected, GUI_FONT_TINY);
        }
    }
}
static void flow(gui_canvas_t *c, const gui_dashboard_t *d, const gui_model_t *m) {
    if (!d->view) {
        if (m->flow_valid) {
            gui_circle(c, 26, 34, 18);
            gui_dashed_line(c, 8, 34, 44, 34, 3);
            gui_dashed_line(c, 26, 16, 26, 52, 3);
        } else gui_text(c, 7, 28, "无数据", GUI_FONT_CN12);
        if (m->flow_valid && isfinite(m->flow_velocity[0]) && isfinite(m->flow_velocity[1])) {
            float vx = m->flow_velocity[0], vy = m->flow_velocity[1];
            float length = sqrtf(vx*vx+vy*vy);
            float scale = length > 500 ? 16/length : .032f;
            int dx = (int)(vx*scale), dy = (int)(-vy*scale);
            gui_line(c, 26, 34, 26+dx, 34+dy);
            if (dx || dy) {
                float angle = atan2f((float)dy, (float)dx);
                gui_line(c, 26+dx, 34+dy, 26+dx-(int)(5*cosf(angle-.6f)), 34+dy-(int)(5*sinf(angle-.6f)));
                gui_line(c, 26+dx, 34+dy, 26+dx-(int)(5*cosf(angle+.6f)), 34+dy-(int)(5*sinf(angle+.6f)));
            }
        }
        gui_text(c, 49, 16, "HEIGHT mm", GUI_FONT_TINY);
        number(c, 49, 22, m->flow_height_mm, 0, 0, m->flow_valid, GUI_FONT_LARGE);
        gui_text(c, 49, 40, "QUALITY", GUI_FONT_TINY);
        gui_bar(c, 49, 48, 76, 6, m->flow_valid ? m->flow_quality : 0, 0, 255);
    } else {
        gui_text(c, 2, 16, "XY VELOCITY mm/s", GUI_FONT_TINY);
        int low, high; gui_history_range(&d->history[5], 2, 40, &low, &high);
        gui_plot(c, &d->history[5], 2, 24, 124, 22, 2, low, high);
        gui_text(c, 2, 49, "X", GUI_FONT_TINY);
        number(c, 9, 49, m->flow_velocity[0], 1, 1, m->flow_valid, GUI_FONT_TINY);
        gui_text(c, 67, 49, "Y", GUI_FONT_TINY);
        number(c, 74, 49, m->flow_velocity[1], 1, 1, m->flow_valid, GUI_FONT_TINY);
    }
}
static void health_row(gui_canvas_t *c, int y, const char *label, int okay, const char *detail) {
    gui_box(c, 2, y+1, 5, 5, okay);
    gui_text(c, 12, y, label, GUI_FONT_TINY);
    gui_text(c, 78, y, detail, GUI_FONT_TINY);
}
static void health(gui_canvas_t *c, const gui_model_t *m, unsigned view) {
    if (!view) {
        health_row(c, 16, "IMU / MAG", m->imu_ok && m->mag_ok, m->imu_ok ? m->mag_ok ? "9AXIS OK" : "6AXIS" : "FAILED");
        health_row(c, 24, "BAROMETER", m->baro_ok, m->baro_ok ? "OK" : "FAILED");
        health_row(c, 34, "RC / FLOW", m->rc_connected, m->rc_connected ? m->flow_valid ? "BOTH OK" : "NO FLOW" : "NO RC");
        health_row(c, 44, "FLASH BOOT", m->flash_ok, m->flash_ok ? "OK" : "ID ERROR");
    } else if (view == 1) {
        gui_text(c, 2, 16, "RTOS FREE / MIN B", GUI_FONT_TINY);
        number(c, 2, 23, m->heap_free, 0, 0, 1, GUI_FONT_LARGE);
        number(c, 80, 28, m->heap_min, 0, 0, 1, GUI_FONT_TINY);
        gui_text(c, 2, 42, "LOG DROP", GUI_FONT_TINY);
        number(c, 79, 42, m->log_dropped, 0, 0, 1, GUI_FONT_TINY);
        gui_text(c, 2, 50, "UART ERR", GUI_FONT_TINY);
        number(c, 79, 50, m->uart_errors, 0, 0, 1, GUI_FONT_TINY);
    } else {
        gui_text(c, 2, 16, "DISPLAY 20Hz TARGET", GUI_FONT_TINY);
        gui_text(c, 2, 25, "FRAME ms / BYTES", GUI_FONT_TINY);
        number(c, 2, 34, m->render_ms, 0, 0, 1, GUI_FONT_BODY);
        number(c, 77, 34, m->display_stats.last_bytes, 0, 0, 1, GUI_FONT_BODY);
        gui_text(c, 2, 48, "SPI ERR", GUI_FONT_TINY);
        number(c, 42, 48, m->display_stats.errors, 0, 0, 1, GUI_FONT_TINY);
        gui_text(c, 76, 48, "PARTIAL TX", GUI_FONT_TINY);
    }
}
static void mag_cal(gui_canvas_t *c, const gui_model_t *m) {
    static const char *const prompts[] = {"绕横滚轴旋转", "绕俯仰轴旋转", "绕航向轴旋转", "正在保存", "校准完成"};
    unsigned step = m->mag_calibration_step > 4 ? 4 : m->mag_calibration_step;
    gui_text(c, 10, 16, prompts[step], GUI_FONT_CN12);
    gui_bar(c, 4, 31, 120, 6, step, 0, 4);
    vector_row(c, 46, "BIAS", m->mag_bias, m->mag_ok, 1, 1);
}
void gui_dashboard_render(gui_dashboard_t *d, gui_canvas_t *c, const gui_model_t *m) {
    gui_clear(c);
    if (d->screen == GUI_SCREEN_ROOT || d->screen == GUI_SCREEN_CALIBRATION) {
        if (d->screen == GUI_SCREEN_CALIBRATION)
            d->menu.items = m->remote_calibrating ? calibration_save : calibration_start;
        gui_menu_render(&d->menu, c, d->screen == GUI_SCREEN_ROOT ? "主菜单" : "校准管理", status(m), m->now_ms);
        return;
    }
    if (d->screen == GUI_SCREEN_CONFIRM) {
        gui_menu_render(&d->menu, c, d->confirm_command == GUI_COMMAND_REMOTE_SAVE ? "保存校准" : "开始校准",
                         d->confirm_command == GUI_COMMAND_MAG_START ? "磁场" : "遥控", m->now_ms);
        return;
    }
    if (d->screen == GUI_SCREEN_MESSAGE) {
        header(c, "暂不可用", "返回");
        const char *reason = d->message == MESSAGE_FLASH ? "闪存异常"
                             : d->message == MESSAGE_NO_RC ? "遥控未连接"
                             : d->message == MESSAGE_MAG ? "磁场未就绪" : "等待校准";
        const char *detail = d->message == MESSAGE_FLASH ? "校准数据无法保存"
                             : d->message == MESSAGE_NO_RC ? "请先连接接收机"
                             : d->message == MESSAGE_MAG ? "请等待器件初始化" : "请静置等待校准";
        gui_text(c, (GUI_WIDTH-gui_text_width(c, reason, GUI_FONT_CN16))/2, 20, reason, GUI_FONT_CN16);
        gui_text(c, (GUI_WIDTH-gui_text_width(c, detail, GUI_FONT_CN12))/2, 41, detail, GUI_FONT_CN12);
        gui_line(c, 0, 57, 127, 57); gui_text(c, 1, 58, "1/2 BACK  HOLD1 MENU", GUI_FONT_TINY);
        return;
    }
    if (d->screen == GUI_SCREEN_HELP) {
        header(c, "按键说明", "返回");
        static const char *const help[2][3] = {
            {"1短按：下一项", "2短按：确认", "1长按：返回"},
            {"页面1：翻页", "页面2：切视图", "2长按：校准菜单"}
        };
        for (unsigned i = 0; i < 3; i++) gui_text(c, 4, 16+(int)i*13, help[d->help_page][i], GUI_FONT_CN12);
        gui_line(c, 0, 57, 127, 57); gui_text(c, 1, 58, "1 NEXT 2 BACK HOLD=600ms", GUI_FONT_TINY);
        return;
    }
    const char *title = d->page == GUI_PAGE_REMOTE_CAL ? "遥控校准"
                       : d->page == GUI_PAGE_MAG_CAL ? "磁力校准"
                       : d->page == GUI_PAGE_ATTITUDE && d->view ? "人工地平"
                       : d->page == GUI_PAGE_REMOTE && d->view >= 2 ? "原始通道"
                       : d->page <= GUI_PAGE_COUNT ? titles[d->page] : "显示页面";
    header(c, title, status(m));
    switch (d->page) {
    case GUI_PAGE_OVERVIEW: overview(c, m); break;
    case GUI_PAGE_ATTITUDE: attitude(c, m, d->view); break;
    case GUI_PAGE_TRENDS: trends(c, d, m); break;
    case GUI_PAGE_SENSORS: sensors(c, m, d->view); break;
    case GUI_PAGE_REMOTE: remote(c, m, d->view, 0); break;
    case GUI_PAGE_FLOW: flow(c, d, m); break;
    case GUI_PAGE_HEALTH: health(c, m, d->view); break;
    case GUI_PAGE_REMOTE_CAL: remote(c, m, d->view, 1); break;
    case GUI_PAGE_MAG_CAL: mag_cal(c, m); break;
    default: break;
    }
    footer(c, d);
}
