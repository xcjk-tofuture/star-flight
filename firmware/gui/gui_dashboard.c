#include "gui_dashboard.h"
#include "gui_scene.h"
#include "gui_cards.h"
#include "accel_calibration.h"
#include "app_settings.h"
#include "imu_heater.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define DEG_PER_RAD 57.29577951f
static const char *const titles[] = {"", "飞行总览", "三维姿态", "趋势曲线", "传感数据", "遥控通道", "光流测距", "系统诊断"};
enum { MENU_CALIBRATION = 100, MENU_HELP, MENU_BACK, MENU_CONFIRM, MENU_CANCEL,
       MENU_REMOTE_START, MENU_REMOTE_SAVE, MENU_MAG_START, MENU_ACCEL_START,
       MENU_SETTINGS, MENU_SOUND, MENU_SETTINGS_SAVE, MENU_SETTINGS_DEFAULTS,
       MENU_SOUND_NORMAL, MENU_SOUND_QUIET, MENU_SOUND_MUTED,
       MENU_PARAMETERS,
       MENU_EDIT_INCREASE, MENU_EDIT_DECREASE, MENU_EDIT_DONE, MENU_EDIT_CANCEL,
       MENU_TUNING_HEATER, MENU_TUNING_INNER, MENU_TUNING_OUTER, MENU_TUNING_SPEED, MENU_TUNING_HEIGHT,
       MENU_FIELD_BASE=200 };
enum { ROOT_CALIBRATION=7, ROOT_SETTINGS=8, ROOT_PARAMETERS=9, ROOT_HELP=10 };
enum { MESSAGE_NONE, MESSAGE_NO_RC, MESSAGE_FLASH, MESSAGE_MAG, MESSAGE_IMU, MESSAGE_STATE, MESSAGE_ACTIVE, MESSAGE_MODULE };
static const gui_menu_item_t root_items[] = {
    {"飞行总览", 1}, {"三维姿态", 2}, {"趋势曲线", 3}, {"传感数据", 4},
    {"遥控通道", 5}, {"光流测距", 6}, {"系统诊断", 7},
    {"校准管理", MENU_CALIBRATION}, {"系统设置", MENU_SETTINGS},
    {"参数控制", MENU_PARAMETERS}, {"按键说明", MENU_HELP}
};
static const gui_menu_item_t calibration_start[] = {
    {"遥控校准", MENU_REMOTE_START}, {"磁力校准", MENU_MAG_START},
    {"加速度校准", MENU_ACCEL_START}, {"返回菜单", MENU_BACK}
};
static const gui_menu_item_t calibration_save[] = {
    {"保存退出", MENU_REMOTE_SAVE}, {"磁力校准", MENU_MAG_START},
    {"加速度校准", MENU_ACCEL_START}, {"返回菜单", MENU_BACK}
};
static const gui_menu_item_t confirmation[] = {{"确认", MENU_CONFIRM}, {"取消", MENU_CANCEL}};
static const gui_menu_item_t settings_items[] = {
    {"声音模式", MENU_SOUND}, {"启动页面", MENU_FIELD_BASE+UAV_SETTING_BOOT_PAGE}, {"保存设置", MENU_SETTINGS_SAVE},
    {"恢复默认", MENU_SETTINGS_DEFAULTS}, {"返回菜单", MENU_BACK}
};
static const gui_menu_item_t parameter_items[] = {
    {"IMU恒温", MENU_TUNING_HEATER}, {"内环控制", MENU_TUNING_INNER},
    {"外环控制", MENU_TUNING_OUTER}, {"速度控制", MENU_TUNING_SPEED},
    {"高度控制", MENU_TUNING_HEIGHT}, {"返回菜单", MENU_BACK}
};
static const gui_menu_item_t edit_items[] = {
    {"增加数值", MENU_EDIT_INCREASE}, {"减少数值", MENU_EDIT_DECREASE},
    {"完成调整", MENU_EDIT_DONE}, {"取消调整", MENU_EDIT_CANCEL}
};
static const char *const field_titles[UAV_SETTING_COUNT]={"声音模式","启动页面","恒温控制","功率上限","目标温度","比例参数","积分参数","微分参数"};
static const gui_menu_item_t sound_items[] = {
    {"正常提示", MENU_SOUND_NORMAL}, {"安静模式", MENU_SOUND_QUIET},
    {"完全静音", MENU_SOUND_MUTED}, {"返回设置", MENU_BACK}
};
static const char *sound_name(uint8_t mode) {
    return mode==UAV_SOUND_MUTED ? "完全静音":mode==UAV_SOUND_QUIET ? "安静模式":"正常提示";
}
static void open_settings(gui_dashboard_t *d, unsigned selection) {
    d->screen=GUI_SCREEN_SETTINGS; d->message_return_screen=GUI_SCREEN_SETTINGS;
    gui_menu_open(&d->menu,settings_items,sizeof(settings_items)/sizeof(settings_items[0]),(uint8_t)selection);
}
static void open_parameters(gui_dashboard_t *d, unsigned selection) {
    d->screen=GUI_SCREEN_PARAMETERS; d->message_return_screen=GUI_SCREEN_PARAMETERS;
    gui_menu_open(&d->menu,parameter_items,sizeof(parameter_items)/sizeof(parameter_items[0]),(uint8_t)selection);
}
static gui_tuning_model_t tuning_model(const gui_model_t *m) {
    return (gui_tuning_model_t){.values=m->settings_value,.now_ms=m->now_ms,.actual=m->heater_temperature,
        .setpoint=m->settings_value[UAV_SETTING_HEATER_TARGET]*.1f,.output=m->heater_duty,.valid=m->heater_temperature_valid,
        .fault=m->heater_fault,.dirty=m->settings_dirty,.save_state=m->settings_save_state,
        .enabled=(uint8_t)m->settings_value[UAV_SETTING_HEATER_ENABLED],.monitor_only=m->heater_state==UAV_HEATER_MONITOR};
}
static void finish_edit(gui_dashboard_t *d) {
    if (d->edit_return_screen==GUI_SCREEN_PARAMETERS) open_parameters(d,d->edit_selection);
    else open_settings(d,d->edit_selection);
}
static void open_root(gui_dashboard_t *d, unsigned selection) {
    d->screen = GUI_SCREEN_ROOT;
    gui_menu_open(&d->menu, root_items, sizeof(root_items)/sizeof(root_items[0]), (uint8_t)selection);
}
static void open_calibration(gui_dashboard_t *d, const gui_model_t *m) {
    d->screen = GUI_SCREEN_CALIBRATION;
    d->message_return_screen=GUI_SCREEN_CALIBRATION;
    gui_menu_open(&d->menu, m->remote_calibrating ? calibration_save : calibration_start, 4, 0);
}
static void close_message(gui_dashboard_t *d, const gui_model_t *m) {
    if (d->message_return_screen==GUI_SCREEN_SETTINGS) open_settings(d,2);
    else if (d->message_return_screen==GUI_SCREEN_PARAMETERS) open_parameters(d,d->parameter_selection);
    else if (d->message_return_screen==GUI_SCREEN_TUNING) d->screen=GUI_SCREEN_TUNING;
    else open_calibration(d,m);
}
static const char *status(const gui_model_t *m) {
    if (m->accel_calibrating) return "六面校准";
    if (m->imu_cal_failed) return "校准失败";
    if (m->imu_calibrating) return "校准中";
    if (m->mag_calibrating) return "磁校中";
    if (m->remote_calibrating) return "遥控校准";
    if (!m->imu_ok) return "惯导异常";
    if (!m->attitude_valid) return "数据过期";
    if (!m->rc_raw_connected) return m->rc_receiver_present ? "遥控失联":"待遥控";
    if (!m->rc_parameters_valid) return "遥控待校";
    if (!m->rc_connected) return "遥控未就绪";
    if (m->rc_ui_active) return m->rc_ui_ready ? "锁定遥控":"等待回中";
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
static void remote_footer(gui_canvas_t *c, const gui_dashboard_t *d, const gui_model_t *m) {
    if (!m->rc_ui_active) return;
    gui_color(c,0); gui_box(c,0,57,128,7,1); gui_color(c,1);
    gui_text(c,1,58,!m->rc_ui_ready ? "RC: CENTER STICKS FIRST"
        : d->screen==GUI_SCREEN_EDIT ? "CH7 ACTION CH8 VALUE CH1 OK/BACK"
        : m->remote_calibrating && d->screen==GUI_SCREEN_PAGE && d->page==GUI_PAGE_REMOTE_CAL ? "YAW> HOLD SAVE  YAW< CANCEL"
        : "CH2 SELECT CH1 OK/BACK 7/8 KNOB",GUI_FONT_TINY);
}
static void footer(gui_canvas_t *c, const gui_dashboard_t *d, const gui_model_t *m) {
    const char *hint = d->page == GUI_PAGE_REMOTE_CAL ? "2 VIEW 1HOLD SAVE 2HOLD CANCEL"
                      : d->page == GUI_PAGE_ACCEL_CAL ? "1/2 VIEW HOLD2 CANCEL"
                      : d->page == GUI_PAGE_MAG_CAL ? "2 FLIP HOLD2 CANCEL"
                      : d->page > GUI_PAGE_COUNT ? "1HOLD MENU 2HOLD CAL" : "1PAGE 2VIEW 1HOLD MENU";
    gui_text(c, 1, 58, hint, GUI_FONT_TINY);
    if (d->page <= GUI_PAGE_COUNT)
        for (unsigned i = 1; i <= GUI_PAGE_COUNT; i++)
            gui_box(c, 91+(int)i*4, 60, 3, 3, i == d->page);
    remote_footer(c,d,m);
}
unsigned gui_dashboard_view_count(uint8_t page) {
    if (page == GUI_PAGE_TRENDS) return GUI_CHART_COUNT;
    if (page==GUI_PAGE_FLOW) return 5;
    if (page == GUI_PAGE_SENSORS || page == GUI_PAGE_HEALTH) return 3;
    if (page == GUI_PAGE_REMOTE) return 4;
    if (page == GUI_PAGE_MAG_CAL || page == GUI_PAGE_ACCEL_CAL) return 2;
    if (page == GUI_PAGE_ATTITUDE ||
        page == GUI_PAGE_REMOTE_CAL) return 2;
    return 1;
}
void gui_dashboard_init(gui_dashboard_t *d) {
    memset(d, 0, sizeof(*d)); d->page = GUI_PAGE_OVERVIEW; open_root(d, 0);
    gui_tuning_init(&d->tuning);
    d->screen=GUI_SCREEN_PAGE;
}
void gui_dashboard_set_page(gui_dashboard_t *d, uint8_t page) {
    if (page==GUI_PAGE_HEATER) {
        d->page=page; d->screen=GUI_SCREEN_TUNING; d->message_return_screen=GUI_SCREEN_TUNING;
        gui_tuning_open(&d->tuning,&gui_tuning_heater); return;
    }
    if ((page >= 1 && page <= GUI_PAGE_COUNT) || page == GUI_PAGE_REMOTE_CAL || page == GUI_PAGE_MAG_CAL || page == GUI_PAGE_ACCEL_CAL || page==GUI_PAGE_HEATER) {
        d->page = page; d->view = 0; d->screen = GUI_SCREEN_PAGE;
    }
}
void gui_dashboard_next_page(gui_dashboard_t *d) {
    gui_dashboard_set_page(d, d->page >= GUI_PAGE_COUNT ? 1 : d->page + 1);
}
void gui_dashboard_next_view(gui_dashboard_t *d) {
    d->view = (uint8_t)((d->view + 1u) % gui_dashboard_view_count(d->page));
}
gui_command_t gui_dashboard_input(gui_dashboard_t *d, gui_input_t input, const gui_model_t *m) {
    if (d->screen==GUI_SCREEN_TUNING) {
        unsigned event=input==GUI_INPUT_NEXT ? GUI_TUNE_NEXT:input==GUI_INPUT_PREVIOUS ? GUI_TUNE_PREVIOUS
            :input==GUI_INPUT_ENTER ? GUI_TUNE_ENTER:input==GUI_INPUT_BACK ? GUI_TUNE_BACK
            :input==GUI_INPUT_CALIBRATION || input==GUI_INPUT_NEXT_VIEW || input==GUI_INPUT_NEXT_PAGE ? GUI_TUNE_SWITCH:0;
        unsigned field=gui_tuning_field(&d->tuning);
        if (input==GUI_INPUT_ENTER && !d->tuning.editing && m->state!=0 &&
            (d->tuning.page==0 || d->tuning.selected==1 || d->tuning.selected>=2 ||
             !m->settings_value[UAV_SETTING_HEATER_ENABLED])) {
            d->message=MESSAGE_STATE; d->screen=GUI_SCREEN_MESSAGE; return GUI_COMMAND_NONE;
        }
        d->edit_field=(uint8_t)field;
        gui_tuning_model_t model=tuning_model(m);
        unsigned result=gui_tuning_input(&d->tuning,event,&model);
        d->edit_original=d->tuning.original;
        if (result==GUI_TUNE_EXIT) { open_parameters(d,0); return GUI_COMMAND_NONE; }
        if (result==GUI_TUNE_INCREASE) return GUI_COMMAND_SETTING_INCREASE;
        if (result==GUI_TUNE_DECREASE) return GUI_COMMAND_SETTING_DECREASE;
        if (result==GUI_TUNE_CANCEL) return GUI_COMMAND_SETTING_CANCEL;
        if (result==GUI_TUNE_TOGGLE) return GUI_COMMAND_HEATER_TOGGLE;
        if (result==GUI_TUNE_DEFAULTS) return GUI_COMMAND_HEATER_DEFAULTS;
        if (result==GUI_TUNE_SAVE) {
            if (m->settings_save_state==UAV_SETTINGS_SAVING) return GUI_COMMAND_NONE;
            d->message=m->state!=0 ? MESSAGE_STATE:!m->flash_ok ? MESSAGE_FLASH
                :m->imu_calibrating || m->mag_calibrating || m->accel_calibrating || m->remote_calibrating ? MESSAGE_ACTIVE:MESSAGE_NONE;
            if (d->message) d->screen=GUI_SCREEN_MESSAGE;
            else return GUI_COMMAND_SETTINGS_SAVE;
        }
        return GUI_COMMAND_NONE;
    }
    if (d->screen == GUI_SCREEN_CALIBRATION)
        d->menu.items = m->remote_calibrating ? calibration_save : calibration_start;
    if (input == GUI_INPUT_NEXT_PAGE) { gui_dashboard_next_page(d); return GUI_COMMAND_NONE; }
    if (input == GUI_INPUT_NEXT_VIEW) { gui_dashboard_next_view(d); return GUI_COMMAND_NONE; }
    if (input==GUI_INPUT_PREVIOUS) {
        if (d->screen==GUI_SCREEN_PAGE) {
            if (d->page>GUI_PAGE_COUNT)
                d->view=(uint8_t)((d->view+gui_dashboard_view_count(d->page)-1)%gui_dashboard_view_count(d->page));
            else gui_dashboard_set_page(d,d->page<=1 ? GUI_PAGE_COUNT:d->page-1);
        } else if (d->screen==GUI_SCREEN_HELP) d->help_page^=1;
        else if (d->screen==GUI_SCREEN_MESSAGE) close_message(d,m);
        else gui_menu_previous(&d->menu);
        return GUI_COMMAND_NONE;
    }
    if (input == GUI_INPUT_BACK) {
        if (d->screen==GUI_SCREEN_EDIT) { finish_edit(d); return GUI_COMMAND_SETTING_CANCEL; }
        if (d->screen == GUI_SCREEN_PAGE && d->page == GUI_PAGE_MAG_CAL && m->mag_calibrating) {
            if (m->mag_calibration_step >= 2) return GUI_COMMAND_NONE;
            open_calibration(d, m); return GUI_COMMAND_MAG_CANCEL;
        }
        if (d->screen == GUI_SCREEN_PAGE && d->page == GUI_PAGE_ACCEL_CAL && m->accel_calibrating) {
            if (m->accel_cal_phase>=UAV_ACCEL_SAVE) return GUI_COMMAND_NONE;
            open_calibration(d,m); return GUI_COMMAND_ACCEL_CANCEL;
        }
        if (d->screen == GUI_SCREEN_ROOT) d->screen = d->page==GUI_PAGE_HEATER ? GUI_SCREEN_TUNING:GUI_SCREEN_PAGE;
        else if (d->screen == GUI_SCREEN_SOUND) open_settings(d,0);
        else if (d->screen == GUI_SCREEN_SETTINGS) open_root(d,ROOT_SETTINGS);
        else if (d->screen == GUI_SCREEN_PARAMETERS) open_root(d,ROOT_PARAMETERS);
        else if (d->screen == GUI_SCREEN_MESSAGE) close_message(d,m);
        else if (d->screen == GUI_SCREEN_CONFIRM) open_calibration(d, m);
        else if (d->screen == GUI_SCREEN_PAGE && d->page > GUI_PAGE_COUNT) open_calibration(d, m);
        else open_root(d, d->screen == GUI_SCREEN_HELP ? ROOT_HELP : d->screen == GUI_SCREEN_CALIBRATION ? ROOT_CALIBRATION : d->page-1);
        return GUI_COMMAND_NONE;
    }
    if (input == GUI_INPUT_CALIBRATION) {
        if (m->mag_calibrating) {
            if (m->mag_calibration_step>=2) { gui_dashboard_set_page(d,GUI_PAGE_MAG_CAL); return GUI_COMMAND_NONE; }
            open_calibration(d,m); return GUI_COMMAND_MAG_CANCEL;
        }
        if (m->accel_calibrating) {
            if (m->accel_cal_phase>=UAV_ACCEL_SAVE) { gui_dashboard_set_page(d,GUI_PAGE_ACCEL_CAL); return GUI_COMMAND_NONE; }
            open_calibration(d,m); return GUI_COMMAND_ACCEL_CANCEL;
        }
        if (m->remote_calibrating) {
            if (m->remote_saving) { gui_dashboard_set_page(d,GUI_PAGE_REMOTE_CAL); return GUI_COMMAND_NONE; }
            open_calibration(d,m); return GUI_COMMAND_REMOTE_CANCEL;
        }
        if (d->screen != GUI_SCREEN_CONFIRM && d->screen != GUI_SCREEN_MESSAGE) open_calibration(d, m);
        return GUI_COMMAND_NONE;
    }
    if (d->screen == GUI_SCREEN_MESSAGE) { close_message(d,m); return GUI_COMMAND_NONE; }
    if (input == GUI_INPUT_NEXT) {
        if (d->screen == GUI_SCREEN_PAGE && (d->page==GUI_PAGE_ACCEL_CAL || d->page==GUI_PAGE_MAG_CAL)) gui_dashboard_next_view(d);
        else if (d->screen == GUI_SCREEN_PAGE) gui_dashboard_next_page(d);
        else if (d->screen == GUI_SCREEN_HELP) d->help_page ^= 1;
        else gui_menu_next(&d->menu);
        return GUI_COMMAND_NONE;
    }
    if (input != GUI_INPUT_ENTER) return GUI_COMMAND_NONE;
    if (d->screen == GUI_SCREEN_PAGE) {
        gui_dashboard_next_view(d); return GUI_COMMAND_NONE;
    }
    if (d->screen == GUI_SCREEN_HELP) { open_root(d, ROOT_HELP); return GUI_COMMAND_NONE; }
    unsigned action = d->menu.items[d->menu.selected].action;
    if (d->screen == GUI_SCREEN_ROOT) {
        if (action <= GUI_PAGE_COUNT) gui_dashboard_set_page(d, (uint8_t)action);
        else if (action == MENU_CALIBRATION) open_calibration(d, m);
        else if (action == MENU_SETTINGS) open_settings(d,0);
        else if (action == MENU_PARAMETERS) open_parameters(d,0);
        else if (action == MENU_HELP) { d->screen = GUI_SCREEN_HELP; d->help_page = 0; }
    } else if (d->screen==GUI_SCREEN_SETTINGS || d->screen==GUI_SCREEN_PARAMETERS) {
        int parameters=d->screen==GUI_SCREEN_PARAMETERS;
        if (parameters) d->parameter_selection=d->menu.selected;
        if (action==MENU_BACK) open_root(d,parameters ? ROOT_PARAMETERS:ROOT_SETTINGS);
        else if (action==MENU_SOUND) {
            d->screen=GUI_SCREEN_SOUND;
            gui_menu_open(&d->menu,sound_items,4,m->sound_mode);
        } else if (action==MENU_SETTINGS_DEFAULTS) {
            if (m->state!=0) { d->message=MESSAGE_STATE; d->screen=GUI_SCREEN_MESSAGE; }
            else return GUI_COMMAND_SETTINGS_DEFAULTS;
        }
        else if (action==MENU_TUNING_HEATER) gui_dashboard_set_page(d,GUI_PAGE_HEATER);
        else if (action>=MENU_TUNING_INNER && action<=MENU_TUNING_HEIGHT) {
            d->message=MESSAGE_MODULE; d->screen=GUI_SCREEN_MESSAGE;
        }
        else if (action>=MENU_FIELD_BASE && action<MENU_FIELD_BASE+UAV_SETTING_COUNT) {
            unsigned field=action-MENU_FIELD_BASE;
            if (field>=UAV_SETTING_HEATER_ENABLED && m->state!=0) {
                d->message=MESSAGE_STATE; d->screen=GUI_SCREEN_MESSAGE;
            } else {
                d->edit_return_screen=d->screen; d->edit_selection=d->menu.selected;
                d->edit_field=(uint8_t)field; d->edit_original=m->settings_value[field];
                d->screen=GUI_SCREEN_EDIT; gui_menu_open(&d->menu,edit_items,4,0);
            }
        }
        else if (action==MENU_SETTINGS_SAVE) {
            if (m->settings_save_state==UAV_SETTINGS_SAVING) return GUI_COMMAND_NONE;
            d->message=m->state!=0 ? MESSAGE_STATE:!m->flash_ok ? MESSAGE_FLASH
                :m->imu_calibrating || m->mag_calibrating || m->accel_calibrating || m->remote_calibrating ? MESSAGE_ACTIVE:MESSAGE_NONE;
            if (d->message) d->screen=GUI_SCREEN_MESSAGE;
            else return GUI_COMMAND_SETTINGS_SAVE;
        }
    } else if (d->screen==GUI_SCREEN_SOUND) {
        if (action==MENU_BACK) open_settings(d,0);
        else return action==MENU_SOUND_NORMAL ? GUI_COMMAND_SOUND_NORMAL
            :action==MENU_SOUND_QUIET ? GUI_COMMAND_SOUND_QUIET:GUI_COMMAND_SOUND_MUTED;
    } else if (d->screen==GUI_SCREEN_EDIT) {
        if (action==MENU_EDIT_INCREASE) return GUI_COMMAND_SETTING_INCREASE;
        if (action==MENU_EDIT_DECREASE) return GUI_COMMAND_SETTING_DECREASE;
        finish_edit(d);
        if (action==MENU_EDIT_CANCEL) return GUI_COMMAND_SETTING_CANCEL;
    } else if (d->screen == GUI_SCREEN_CALIBRATION) {
        if (action == MENU_BACK) { open_root(d, ROOT_CALIBRATION); return GUI_COMMAND_NONE; }
        if (action == MENU_MAG_START && m->mag_calibrating) {
            gui_dashboard_set_page(d, GUI_PAGE_MAG_CAL); return GUI_COMMAND_NONE;
        }
        if (action==MENU_ACCEL_START && m->accel_calibrating) {
            gui_dashboard_set_page(d,GUI_PAGE_ACCEL_CAL); return GUI_COMMAND_NONE;
        }
        d->message = m->state != 0 ? MESSAGE_STATE : !m->flash_ok ? MESSAGE_FLASH
                     : (action!=MENU_REMOTE_SAVE && (m->remote_calibrating || m->mag_calibrating || m->accel_calibrating)) ? MESSAGE_ACTIVE
                     : action == MENU_REMOTE_START && !m->rc_raw_connected ? MESSAGE_NO_RC
                     : action == MENU_MAG_START && !m->mag_ok ? MESSAGE_MAG
                     : action == MENU_MAG_START && (!m->attitude_valid || m->imu_calibrating || m->imu_cal_failed) ? MESSAGE_IMU : MESSAGE_NONE;
        if (!d->message && action==MENU_ACCEL_START && (!m->imu_ok || m->imu_calibrating)) d->message=MESSAGE_IMU;
        if (!d->message && action==MENU_REMOTE_START && m->imu_calibrating) d->message=MESSAGE_IMU;
        if (d->message) { d->screen = GUI_SCREEN_MESSAGE; return GUI_COMMAND_NONE; }
        d->confirm_command = action == MENU_REMOTE_START ? GUI_COMMAND_REMOTE_START
                             : action == MENU_REMOTE_SAVE ? GUI_COMMAND_REMOTE_SAVE
                             : action == MENU_ACCEL_START ? GUI_COMMAND_ACCEL_START : GUI_COMMAND_MAG_START;
        d->screen = GUI_SCREEN_CONFIRM;
        gui_menu_open(&d->menu, confirmation, 2, 1); /* Cancel is the initial choice. */
    } else if (d->screen == GUI_SCREEN_CONFIRM) {
        if (action == MENU_CANCEL) { open_calibration(d, m); return GUI_COMMAND_NONE; }
        gui_command_t command = (gui_command_t)d->confirm_command;
        if (m->state != 0) { d->message = MESSAGE_STATE; d->screen = GUI_SCREEN_MESSAGE; return GUI_COMMAND_NONE; }
        if (!m->flash_ok) { d->message = MESSAGE_FLASH; d->screen = GUI_SCREEN_MESSAGE; return GUI_COMMAND_NONE; }
        if (command!=GUI_COMMAND_REMOTE_SAVE && (m->remote_calibrating || m->mag_calibrating || m->accel_calibrating)) {
            d->message=MESSAGE_ACTIVE; d->screen=GUI_SCREEN_MESSAGE; return GUI_COMMAND_NONE;
        }
        if (command==GUI_COMMAND_ACCEL_START && (!m->imu_ok || m->imu_calibrating)) {
            d->message=MESSAGE_IMU; d->screen=GUI_SCREEN_MESSAGE; return GUI_COMMAND_NONE;
        }
        if (command == GUI_COMMAND_REMOTE_START && !m->rc_raw_connected) {
            d->message = MESSAGE_NO_RC; d->screen = GUI_SCREEN_MESSAGE; return GUI_COMMAND_NONE;
        }
        if (command == GUI_COMMAND_MAG_START && (!m->mag_ok || !m->attitude_valid || m->imu_calibrating || m->imu_cal_failed)) {
            d->message = MESSAGE_IMU; d->screen = GUI_SCREEN_MESSAGE; return GUI_COMMAND_NONE;
        }
        gui_dashboard_set_page(d, command == GUI_COMMAND_REMOTE_SAVE ? GUI_PAGE_REMOTE_CAL
                                  : command==GUI_COMMAND_ACCEL_START ? GUI_PAGE_ACCEL_CAL
                                  : command == GUI_COMMAND_REMOTE_START ? GUI_PAGE_REMOTE_CAL : GUI_PAGE_MAG_CAL);
        return command;
    }
    return GUI_COMMAND_NONE;
}
void gui_dashboard_update(gui_dashboard_t *d, const gui_model_t *m) {
    gui_tuning_model_t tune=tuning_model(m); gui_tuning_update(&d->tuning,&tune);
    uint32_t elapsed = (uint32_t)(m->now_ms - d->last_sample_ms);
    if (d->has_sample && elapsed < 100u) return;
    if (d->has_sample && elapsed > 500u) {
        const float empty[3] = {0};
        for (unsigned i = 0; i < GUI_CHART_COUNT; i++) gui_history_push(&d->history[i], empty, 0, 1);
        gui_history_push(&d->range_history,empty,0,1);
        for (unsigned i=0;i<2;i++) gui_history_push(&d->flow_comp_history[i],empty,0,1);
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
    v[0]=m->flow_range_m*1000; v[1]=m->flow_agl_m*1000; v[2]=0;
    gui_history_push(&d->range_history,v,(uint8_t)((m->range_valid ? 1:0)|(m->flow_height_valid ? 2:0)),.1f);
    for (unsigned i=0;i<2;i++) {
        v[0]=m->flow_raw_rate[i]*1000; v[1]=m->flow_rotation_rate[i]*1000; v[2]=m->flow_compensated_rate[i]*1000;
        uint8_t valid=m->flow_raw_fresh && m->flow_quality ? 1:0;
        if (m->flow_comparison_valid) valid=7;
        gui_history_push(&d->flow_comp_history[i],v,valid,.1f);
    }
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
    gui_text(c, 2, 50, m->fusion_mag_used ? "9AXIS" : "6AXIS", GUI_FONT_TINY);
    gui_text(c, 38, 50, m->baro_ok ? "BARO OK" : "BARO ERR", GUI_FONT_TINY);
    gui_text(c, 84, 50, m->flash_ok ? "NVM OK" : "NVM ERR", GUI_FONT_TINY);
}
static void attitude(gui_canvas_t *c, const gui_model_t *m, unsigned view) {
    u8g2_SetClipWindow(&c->graphics, 0, 16, 82, 56);
    if (m->attitude_valid) {
        if (!view) gui_drone_3d(c, m->attitude[0], m->attitude[1], m->attitude[2], 40, 34, 22);
        else gui_horizon(c, m->attitude[0], m->attitude[1], 2, 16, 78, 38);
    } else gui_text(c, 12, 28, m->imu_cal_failed ? "校准失败" : m->imu_calibrating ? "校准中" : "等待姿态", GUI_FONT_CN12);
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
    int raw_view = !calibration && (view >= 2 || !m->rc_parameters_valid);
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
    if (!calibration && !m->rc_parameters_valid) gui_text(c,2,51,"RAW / CALIBRATION NEEDED",GUI_FONT_TINY);
}
static void flow(gui_canvas_t *c, const gui_dashboard_t *d, const gui_model_t *m) {
    if (!d->view) {
        if (m->flow_valid) {
            gui_circle(c, 26, 34, 18);
            gui_dashed_line(c, 8, 34, 44, 34, 3);
            gui_dashed_line(c, 26, 16, 26, 52, 3);
        } else gui_text(c, 2, 28, m->flow_frames ? "未就绪":"无数据", GUI_FONT_CN12);
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
        gui_text(c, 49, 16, m->flow_height_valid ? "AGL mm":"RANGE mm", GUI_FONT_TINY);
        number(c, 49, 22, m->flow_height_mm, 0, 0, m->range_valid, GUI_FONT_LARGE);
        gui_text(c, 49, 40, "Q FLOW/RNG", GUI_FONT_TINY);
        number(c,49,47,m->flow_quality,0,0,m->flow_raw_fresh,GUI_FONT_TINY);
        number(c,88,47,m->range_quality,0,0,m->flow_raw_fresh,GUI_FONT_TINY);
        if (!m->flow_valid) { gui_text(c,2,50,"WHY",GUI_FONT_TINY); number(c,21,50,m->flow_reason,0,0,1,GUI_FONT_TINY); }
    } else if (d->view==1) {
        gui_text(c, 2, 16, "SENSOR XY mm/s", GUI_FONT_TINY);
        int low, high; gui_history_range(&d->history[5], 2, 40, &low, &high);
        gui_plot(c, &d->history[5], 2, 24, 124, 22, 2, low, high);
        gui_text(c, 2, 49, "X", GUI_FONT_TINY);
        number(c, 9, 49, m->flow_velocity[0], 1, 1, m->flow_valid, GUI_FONT_TINY);
        gui_text(c, 67, 49, "Y", GUI_FONT_TINY);
        number(c, 74, 49, m->flow_velocity[1], 1, 1, m->flow_valid, GUI_FONT_TINY);
    } else if (d->view==2) {
        gui_text(c,2,16,"RANGE- AGL: mm",GUI_FONT_TINY);
        int low,high; gui_history_range(&d->range_history,2,40,&low,&high);
        gui_plot(c,&d->range_history,2,24,124,22,2,low,high);
        gui_text(c,2,49,"RAW",GUI_FONT_TINY); number(c,20,49,m->flow_raw_range_mm,0,0,m->flow_raw_fresh,GUI_FONT_TINY);
        gui_text(c,70,49,"Vz",GUI_FONT_TINY); number(c,83,49,m->flow_velocity[2],0,1,m->flow_height_valid,GUI_FONT_TINY);
    } else {
        unsigned axis=d->view==3 ? 0:1;
        static const char *status[]={"OFF","TIME","MOUNT?","DELAY?","READY","GYRO GAP","CONFIG"};
        gui_text(c,2,16,axis ? "Y RAW-/GYR:/CMP.":"X RAW-/GYR:/CMP.",GUI_FONT_TINY);
        int low,high; gui_history_range(&d->flow_comp_history[axis],3,20,&low,&high);
        gui_plot(c,&d->flow_comp_history[axis],2,24,124,22,3,low,high);
        gui_text(c,2,49,"mrad/s",GUI_FONT_TINY);
        gui_text(c,48,49,status[m->flow_comp_status<7 ? m->flow_comp_status:6],GUI_FONT_TINY);
    }
}
static void health_row(gui_canvas_t *c, int y, const char *label, int okay, const char *detail) {
    gui_box(c, 2, y+1, 5, 5, okay);
    gui_text(c, 12, y, label, GUI_FONT_TINY);
    gui_text(c, 78, y, detail, GUI_FONT_TINY);
}
static void health(gui_canvas_t *c, const gui_model_t *m, unsigned view) {
    if (!view) {
        health_row(c, 16, "IMU / MAG", m->imu_ok && m->mag_ok && !m->imu_cal_failed,
                   m->imu_cal_failed ? "CAL FAIL" : m->imu_ok ? m->fusion_mag_used ? "9AXIS OK" : "6AXIS" : "FAILED");
        health_row(c, 24, "BAROMETER", m->baro_ok, m->baro_ok ? "OK" : "FAILED");
        health_row(c, 34, "RC / FLOW", m->rc_raw_connected,
                   !m->rc_raw_connected ? m->rc_receiver_present ? "FAILSAFE":"NO DATA"
                   : !m->rc_parameters_valid ? "NEED CAL":m->flow_valid ? "BOTH OK":"NO FLOW");
        health_row(c, 44, "PARAM NVM", m->flash_ok, m->flash_ok ? "INTERNAL" : "FAILED");
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
static void mag_sphere(gui_canvas_t *c, const gui_model_t *m, unsigned back) {
    const int cx=25,cy=38,radius=17;
    gui_circle(c,cx,cy,radius);
    for (int latitude=-1;latitude<=1;latitude++) {
        float z=latitude*.5f; int width=(int)(radius*sqrtf(1-z*z));
        gui_dashed_line(c,cx-width,cy-(int)(radius*z),cx+width,cy-(int)(radius*z),3);
    }
    int last_x=cx,last_y=cy-radius;
    for (unsigned i=1;i<=24;i++) {
        float angle=i*(6.28318530718f/24);
        int x=cx+(int)(radius*.55f*sinf(angle)),y=cy-(int)(radius*cosf(angle));
        gui_line(c,last_x,last_y,x,y); last_x=x; last_y=y;
    }
    for (unsigned i=0;i<32;i++) {
        float z=-.75f+.5f*(i/8),angle=-3.14159265359f+((i%8)+.5f)*(6.28318530718f/8);
        float horizontal=sqrtf(1-z*z),x=horizontal*cosf(angle),depth=horizontal*sinf(angle);
        if ((back ? -depth:depth)<0) continue;
        int px=cx+(int)(radius*(back ? -x:x)),py=cy-(int)(radius*z);
        if (m->mag_sphere_ready && (m->mag_sphere_mask & (1u<<i))) gui_box(c,px-1,py-1,3,3,1);
        else gui_circle(c,px,py,1);
    }
    const float *goal=m->mag_sphere_goal;
    if ((back ? -goal[1]:goal[1])>=0 && (m->now_ms/300)%2) {
        int x=cx+(int)(radius*(back ? -goal[0]:goal[0])),y=cy-(int)(radius*goal[2]);
        gui_circle(c,x,y,3); gui_line(c,x-3,y,x+3,y); gui_line(c,x,y-3,x,y+3);
    }
    if (m->mag_cursor_valid) {
        const float *point=m->mag_sphere_cursor;
        int x=cx+(int)(radius*(back ? -point[0]:point[0])),y=cy-(int)(radius*point[2]);
        if ((back ? -point[1]:point[1])>=0) {
            gui_color(c,0); gui_box(c,x-2,y-2,5,5,1); gui_color(c,1); gui_circle(c,x,y,2); gui_box(c,x,y,1,1,1);
        } else gui_circle(c,x,y,2);
    }
    gui_text(c,3,16,back ? "BACK":"FRONT",GUI_FONT_TINY);
}
static void mag_cal(gui_canvas_t *c, const gui_model_t *m, unsigned view) {
    static const char *const prompts[]={"慢慢翻转","正在校准","正在保存","校准完成","校准失败"};
    unsigned step=m->mag_calibration_step>4 ? 4:m->mag_calibration_step;
    const char *prompt=prompts[step];
    if (!step) {
        int goal_behind=(view ? -m->mag_sphere_goal[1]:m->mag_sphere_goal[1])<0;
        prompt=m->mag_cal_hint==1 ? "请慢一点":!m->mag_sphere_ready ? "多换方向"
               : goal_behind ? "补背面点":"补空白点";
        if (m->mag_cal_reason==4 || m->mag_cal_reason==6) prompt="避开金属";
    }
    mag_sphere(c,m,view);
    gui_text(c,53,16,prompt,GUI_FONT_CN12);
    gui_text(c,53,29,m->mag_sphere_fitted ? "FIT":"PREVIEW",GUI_FONT_TINY);
    number(c,87,29,m->mag_sphere_covered,0,0,m->mag_sphere_ready,GUI_FONT_TINY);
    gui_text(c,99,29,"/32",GUI_FONT_TINY);
    gui_text(c,53,38,"N",GUI_FONT_TINY);
    number(c,61,38,m->mag_cal_samples,0,0,1,GUI_FONT_TINY);
    gui_text(c,87,38,"ERR",GUI_FONT_TINY);
    number(c,104,38,m->mag_cal_reason,0,0,1,GUI_FONT_TINY);
    gui_text(c,53,48,"RMS%",GUI_FONT_TINY);
    number(c,76,48,m->mag_cal_rms_permille*.1f,1,0,m->mag_cal_quality_ready,GUI_FONT_TINY);
}
static void accel_cal(gui_canvas_t *c, const gui_model_t *m, unsigned view) {
    static const char *const faces[]={"Z轴朝上","Z轴朝下","X轴朝上","X轴朝下","Y轴朝上","Y轴朝下"};
    static const char *const labels[]={"Z+","Z-","X+","X-","Y+","Y-"};
    unsigned target=m->accel_cal_target<6 ? m->accel_cal_target:0;
    if (view) {
        vector_row(c,17,"RAW",m->accel_cal_raw,m->imu_ok,1,1);
        gui_text(c,3,29,"WANT",GUI_FONT_TINY); gui_text(c,25,29,labels[target],GUI_FONT_TINY);
        gui_text(c,44,29,"NOW",GUI_FONT_TINY);
        gui_text(c,60,29,m->accel_cal_detected<6 ? labels[m->accel_cal_detected]:"--",GUI_FONT_TINY);
        gui_text(c,3,40,"水平放置",GUI_FONT_CN12);
        unsigned axis=target<2 ? 2:target<4 ? 0:1;
        unsigned first=(axis+1)%3,second=(axis+2)%3;
        int x=(int)fmaxf(-8,fminf(8,m->accel_cal_raw[first]*6));
        int y=(int)fmaxf(-8,fminf(8,m->accel_cal_raw[second]*6));
        gui_circle(c,108,44,10); gui_line(c,98,44,118,44); gui_line(c,108,34,108,54);
        gui_circle(c,108+x,44-y,2);
        return;
    }
    const char *prompt=faces[target],*detail="任意面静置";
    if (m->accel_cal_detected<6 && (m->accel_cal_faces & (1u<<m->accel_cal_detected))) detail="此面已完成";
    if (m->accel_cal_phase==UAV_ACCEL_SETTLE)
        detail=m->accel_cal_reason==UAV_ACCEL_REASON_ORIENTATION ? "方向需调整"
               : m->accel_cal_reason==UAV_ACCEL_REASON_NOISE ? "振动请重采":"请保持静止";
    else if (m->accel_cal_phase==UAV_ACCEL_SAMPLE) detail="正在采集";
    else if (m->accel_cal_phase==UAV_ACCEL_FIT) detail="检查六面质量";
    else if (m->accel_cal_phase==UAV_ACCEL_SAVE) detail="正在保存";
    else if (m->accel_cal_phase==UAV_ACCEL_DONE) { prompt="六面已完成"; detail="校准完成"; }
    else if (m->accel_cal_phase==UAV_ACCEL_FAILED) { prompt="校准失败"; detail="返回后重试"; }
    else if (m->accel_cal_reason==UAV_ACCEL_REASON_FIT) detail="静置后重采";
    gui_text(c,3,16,prompt,GUI_FONT_CN12);
    gui_text(c,3,29,detail,GUI_FONT_CN12);
    gui_text(c,98,18,"ERR",GUI_FONT_TINY);
    number(c,116,18,m->accel_cal_reason,0,0,1,GUI_FONT_TINY);
    gui_bar(c,3,43,120,3,m->accel_cal_samples,0,UAV_ACCEL_CAL_SAMPLES);
    for (unsigned i=0;i<6;i++) {
        int x=3+(int)i*20,done=!!(m->accel_cal_faces & (1u<<i));
        gui_box(c,x,48,18,8,done);
        if (!done && i==target && (m->now_ms/300)%2) gui_box(c,x-1,47,20,10,0);
        gui_color(c,(uint8_t)!done); gui_text(c,x+4,49,labels[i],GUI_FONT_TINY); gui_color(c,1);
    }
}
void gui_dashboard_render(gui_dashboard_t *d, gui_canvas_t *c, const gui_model_t *m) {
    gui_clear(c);
    if (d->screen==GUI_SCREEN_PARAMETERS) {
        static const uint8_t icons[]={GUI_ICON_HEATER,GUI_ICON_INNER,GUI_ICON_OUTER,GUI_ICON_SPEED,GUI_ICON_HEIGHT,GUI_ICON_BACK};
        gui_cards_render(&d->menu,c,icons,"参数控制",d->menu.selected>0 && d->menu.selected<5 ? "未开放":"选模块");
        remote_footer(c,d,m); return;
    }
    if (d->screen==GUI_SCREEN_TUNING) {
        gui_tuning_model_t tune=tuning_model(m); gui_tuning_render(&d->tuning,c,&tune);
        if (m->rc_ui_active) {
            gui_color(c,0); gui_box(c,0,57,128,7,1); gui_color(c,1);
            gui_text(c,1,58,!m->rc_ui_ready ? "RC CENTER STICKS FIRST":d->tuning.editing
                ? "CH2 +/- CH8 VALUE CH1 DONE/BACK":"CH7 FIELD CH1 EDIT CH8 PAGE",GUI_FONT_TINY);
        }
        return;
    }
    if (d->screen==GUI_SCREEN_SETTINGS || d->screen==GUI_SCREEN_SOUND || d->screen==GUI_SCREEN_PARAMETERS || d->screen==GUI_SCREEN_EDIT) {
        const char *badge=m->settings_save_state==UAV_SETTINGS_SAVING ? "正在保存"
            :m->settings_save_state==UAV_SETTINGS_FAILED ? "保存失败"
            :m->settings_dirty ? "未保存":"已保存";
        if (d->screen==GUI_SCREEN_SOUND) badge=sound_name(m->sound_mode);
        if (d->screen==GUI_SCREEN_PARAMETERS) badge=d->menu.selected>0 && d->menu.selected<5 ? "未开放":"模块";
        char value[16];
        if (d->screen==GUI_SCREEN_EDIT) {
            unsigned f=d->edit_field,v=m->settings_value[f];
            if (f==UAV_SETTING_HEATER_TARGET) snprintf(value,sizeof(value),"%u.%uC",v/10,v%10);
            else if (f==UAV_SETTING_HEATER_KP || f==UAV_SETTING_HEATER_KD) snprintf(value,sizeof(value),"%u.%02u",v/100,v%100);
            else if (f==UAV_SETTING_HEATER_KI) snprintf(value,sizeof(value),"%u.%03u",v/1000,v%1000);
            else snprintf(value,sizeof(value),f==UAV_SETTING_HEATER_LIMIT ? "%u%%":"%u",v);
            badge=value;
        }
        gui_menu_render(&d->menu,c,d->screen==GUI_SCREEN_SOUND ? "声音模式":d->screen==GUI_SCREEN_PARAMETERS ? "参数控制"
            :d->screen==GUI_SCREEN_EDIT ? field_titles[d->edit_field]:"系统设置",badge,m->now_ms);
        gui_color(c,0); gui_box(c,0,57,128,7,1); gui_color(c,1);
        gui_text(c,1,58,d->screen==GUI_SCREEN_SOUND ? "QUIET: NO KEYS MUTE: ALL OFF"
            :d->screen==GUI_SCREEN_EDIT ? "CH8 VALUE / 1 SELECT 2 APPLY":"1 NEXT 2 OK HOLD1 BACK",GUI_FONT_TINY);
        remote_footer(c,d,m);
        return;
    }
    if (d->screen == GUI_SCREEN_ROOT || d->screen == GUI_SCREEN_CALIBRATION) {
        if (d->screen == GUI_SCREEN_CALIBRATION)
            d->menu.items = m->remote_calibrating ? calibration_save : calibration_start;
        gui_menu_render(&d->menu, c, d->screen == GUI_SCREEN_ROOT ? "主菜单" : "校准管理", status(m), m->now_ms);
        remote_footer(c,d,m);
        return;
    }
    if (d->screen == GUI_SCREEN_CONFIRM) {
        gui_menu_render(&d->menu, c, d->confirm_command == GUI_COMMAND_REMOTE_SAVE ? "保存校准" : "开始校准",
                         d->confirm_command == GUI_COMMAND_MAG_START ? "磁场"
                         : d->confirm_command==GUI_COMMAND_ACCEL_START ? "六面" : "遥控", m->now_ms);
        remote_footer(c,d,m);
        return;
    }
    if (d->screen == GUI_SCREEN_MESSAGE) {
        header(c, "暂不可用", "返回");
        const char *reason = d->message == MESSAGE_STATE ? "等待锁定"
                             : d->message==MESSAGE_MODULE ? "暂不可用"
                             : d->message==MESSAGE_ACTIVE ? "校准进行中"
                             : d->message == MESSAGE_FLASH ? "闪存异常"
                             : d->message == MESSAGE_NO_RC ? "遥控未连接"
                             : d->message == MESSAGE_MAG ? "磁场未就绪" : m->imu_cal_failed ? "校准失败" : "等待校准";
        const char *detail = d->message == MESSAGE_STATE ? "请先锁定飞控"
                             : d->message==MESSAGE_MODULE ? "当前模块尚不可调"
                             : d->message==MESSAGE_ACTIVE ? "请先完成当前校准"
                             : d->message == MESSAGE_FLASH ? "校准数据无法保存"
                             : d->message == MESSAGE_NO_RC ? "请先连接接收机"
                             : d->message == MESSAGE_MAG ? "请等待器件初始化" : m->imu_cal_failed ? "请先静置设备" : "请静置等待校准";
        gui_text(c, (GUI_WIDTH-gui_text_width(c, reason, GUI_FONT_CN16))/2, 20, reason, GUI_FONT_CN16);
        gui_text(c, (GUI_WIDTH-gui_text_width(c, detail, GUI_FONT_CN12))/2, 41, detail, GUI_FONT_CN12);
        gui_text(c, 1, 58, "1/2 BACK  HOLD1 MENU", GUI_FONT_TINY);
        remote_footer(c,d,m);
        return;
    }
    if (d->screen == GUI_SCREEN_HELP) {
        header(c, "按键说明", "返回");
        static const char *const help[2][3] = {
            {"1短按：下一项", "2短按：确认", "1长按：返回"},
            {"页面1：翻页", "页面2：切视图", "2长按：校准菜单"}
        };
        for (unsigned i = 0; i < 3; i++) {
            const char *hint=d->help_page==1 && i==2 && (m->mag_calibrating || m->accel_calibrating || m->remote_calibrating)
                             ? "2长按：取消校准":help[d->help_page][i];
            gui_text(c,4,16+(int)i*13,hint,GUI_FONT_CN12);
        }
        gui_text(c, 1, 58, "1 NEXT 2 BACK HOLD=600ms", GUI_FONT_TINY);
        remote_footer(c,d,m);
        return;
    }
    const char *title = d->page == GUI_PAGE_REMOTE_CAL ? "遥控校准"
                       : d->page == GUI_PAGE_MAG_CAL ? "磁力校准"
                       : d->page == GUI_PAGE_ACCEL_CAL ? "加速度校准"
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
    case GUI_PAGE_MAG_CAL: mag_cal(c, m, d->view); break;
    case GUI_PAGE_ACCEL_CAL: accel_cal(c,m,d->view); break;
    default: break;
    }
    footer(c, d, m);
}
