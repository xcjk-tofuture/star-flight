/* Render the firmware's actual portable UI into PBM files using simulated data.
 * This is a presentation/export tool, not a hardware diagnostic. */
#include "gui_dashboard.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
static gui_canvas_t canvas;
static gui_dashboard_t dashboard;
static gui_model_t model;
static int save_frame(const char *folder, const char *name) {
    char path[512]; snprintf(path, sizeof(path), "%s/%s.pbm", folder, name);
    FILE *out = fopen(path, "wb");
    if (!out) return -1;
    fprintf(out, "P4\n%d %d\n", GUI_WIDTH, GUI_HEIGHT);
    for (int y = 0; y < GUI_HEIGHT; y++)
        for (int x = 0; x < GUI_WIDTH; x += 8) {
            unsigned char byte = 0;
            for (int bit = 0; bit < 8; bit++)
                if (canvas.pixels[(y/8)*GUI_WIDTH+x+bit] & (1u << (y%8))) byte |= (unsigned char)(0x80u >> bit);
            fwrite(&byte, 1, 1, out);
        }
    return fclose(out);
}
static void sample(unsigned tick) {
    float t = tick * .1f;
    model.now_ms = tick * 100;
    model.attitude[0] = .28f*sinf(t*.8f); model.attitude[1] = .16f*cosf(t*.6f); model.attitude[2] = -.9f;
    model.acc[0] = .25f*sinf(t); model.acc[1] = -.15f; model.acc[2] = 9.78f;
    model.gyro[0] = .12f*sinf(t*.6f); model.gyro[1] = .08f*cosf(t*.9f); model.gyro[2] = .03f*sinf(t);
    model.mag[0] = -12+2*sinf(t); model.mag[1] = -33+cosf(t); model.mag[2] = -44;
    model.temperature_c = 34.8f+.3f*sinf(t*.2f); model.pressure_pa = 101325+15*cosf(t);
    model.flow_velocity[0] = 280*sinf(t*.7f); model.flow_velocity[1] = 120*cosf(t*.6f);
}
int main(int argc, char **argv) {
    if (argc != 2) { fprintf(stderr, "Usage: gui-preview OUTPUT_DIRECTORY\n"); return 1; }
    gui_canvas_init(&canvas); gui_dashboard_init(&dashboard);
    model.attitude_valid = model.imu_ok = model.mag_ok = model.baro_ok = model.rc_connected = model.rc_raw_connected = 1;
    model.flow_valid = model.flash_ok = 1; model.flow_quality = 210; model.flow_height_mm = 1250;
    model.fusion_mag_used = 1;
    model.heap_free = 2672; model.heap_min = 2440; model.render_ms = 3; model.display_stats.last_bytes = 280;
    for (unsigned i = 0; i < 8; i++) {
        model.remote_pwm[i] = (uint16_t)(1100+i*110); model.remote_raw[i] = (uint16_t)(300+i*150);
        model.remote_min[i] = 300; model.remote_max[i] = 1700; model.mag_scale[i%3] = 1;
    }
    for (unsigned i = 0; i < 90; i++) { sample(i); gui_dashboard_update(&dashboard, &model); }
    gui_dashboard_render(&dashboard, &canvas, &model);
    if (save_frame(argv[1], "menu-main")) return 1;
    gui_dashboard_input(&dashboard, GUI_INPUT_NEXT, &model);
    dashboard.menu.highlight_q8 = (int16_t)((16+(dashboard.menu.selected%3)*13)*256);
    gui_dashboard_render(&dashboard, &canvas, &model);
    if (save_frame(argv[1], "menu-main-next")) return 1;
    gui_dashboard_input(&dashboard, GUI_INPUT_CALIBRATION, &model);
    gui_dashboard_render(&dashboard, &canvas, &model);
    if (save_frame(argv[1], "menu-calibration")) return 1;
    gui_dashboard_input(&dashboard, GUI_INPUT_ENTER, &model);
    gui_dashboard_render(&dashboard, &canvas, &model);
    if (save_frame(argv[1], "menu-confirm")) return 1;
    gui_dashboard_input(&dashboard, GUI_INPUT_BACK, &model);
    model.flash_ok = 0;
    gui_dashboard_input(&dashboard, GUI_INPUT_ENTER, &model);
    gui_dashboard_render(&dashboard, &canvas, &model);
    if (save_frame(argv[1], "menu-flash-warning")) return 1;
    model.flash_ok = 1;
    gui_dashboard_input(&dashboard, GUI_INPUT_BACK, &model);
    gui_dashboard_input(&dashboard, GUI_INPUT_BACK, &model);
    gui_dashboard_input(&dashboard, GUI_INPUT_NEXT, &model);
    gui_dashboard_input(&dashboard, GUI_INPUT_ENTER, &model);
    gui_dashboard_render(&dashboard, &canvas, &model);
    if (save_frame(argv[1], "menu-help")) return 1;
    gui_dashboard_input(&dashboard, GUI_INPUT_NEXT, &model);
    gui_dashboard_render(&dashboard, &canvas, &model);
    if (save_frame(argv[1], "menu-help-pages")) return 1;
    struct { const char *name; unsigned page, view; } scenes[] = {
        {"overview",1,0},{"attitude-3d",2,0},{"horizon",2,1},{"trend-attitude",3,0},
        {"trend-gyro",3,1},{"trend-mag",3,2},{"trend-temp",3,3},{"trend-baro",3,4},{"trend-flow",3,5},
        {"sensors",4,0},{"offsets",4,1},{"environment",4,2},{"remote",5,0},{"remote-5-8",5,1},
        {"remote-raw",5,2},{"remote-raw-5-8",5,3},
        {"flow",6,0},{"flow-chart",6,1},{"health",7,0},{"memory",7,1},{"display-stats",7,2},
        {"remote-cal",19,0},{"mag-cal",20,0}
    };
    for (unsigned i = 0; i < sizeof(scenes)/sizeof(scenes[0]); i++) {
        model.remote_calibrating = scenes[i].page == GUI_PAGE_REMOTE_CAL;
        model.mag_calibrating = scenes[i].page == GUI_PAGE_MAG_CAL;
        gui_dashboard_set_page(&dashboard, scenes[i].page); dashboard.view = (uint8_t)scenes[i].view;
        gui_dashboard_render(&dashboard, &canvas, &model);
        if (save_frame(argv[1], scenes[i].name)) return 1;
    }
    model.remote_calibrating = model.mag_calibrating = 0;
    model.imu_cal_failed=1; model.attitude_valid=0;
    gui_dashboard_set_page(&dashboard,2); gui_dashboard_render(&dashboard,&canvas,&model);
    if (save_frame(argv[1],"attitude-cal-failed")) return 1;
    model.imu_cal_failed=0; model.attitude_valid=1;
    model.rc_connected = 0; model.state = 3; model.flash_ok = 0;
    gui_dashboard_set_page(&dashboard, 1); gui_dashboard_render(&dashboard, &canvas, &model);
    if (save_frame(argv[1], "overview-no-rc")) return 1;
    gui_dashboard_set_page(&dashboard, 2);
    for (unsigned frame = 0; frame < 48; frame++) {
        char name[32]; snprintf(name, sizeof(name), "pose-%02u", frame);
        float a = frame * 6.283185307f/48;
        model.attitude[0] = .42f*sinf(a); model.attitude[1] = .25f*cosf(a); model.attitude[2] = a;
        gui_dashboard_render(&dashboard, &canvas, &model);
        if (save_frame(argv[1], name)) return 1;
    }
    printf("Rendered %u pages and 48 pose frames from the firmware UI (simulated data).\n",
           (unsigned)(sizeof(scenes)/sizeof(scenes[0])+8));
    return 0;
}
