#ifndef STAR_GUI_PLOT_H
#define STAR_GUI_PLOT_H
#include "gui_canvas.h"
#define GUI_HISTORY_SAMPLES 64
#define GUI_HISTORY_CHANNELS 3
typedef struct {
    int16_t values[GUI_HISTORY_SAMPLES][GUI_HISTORY_CHANNELS];
    uint8_t valid[GUI_HISTORY_SAMPLES], head, count;
} gui_history_t;
void gui_history_push(gui_history_t *history, const float values[3], uint8_t valid, float scale);
void gui_plot(gui_canvas_t *canvas, const gui_history_t *history, int x, int y,
              int width, int height, unsigned channels, int low, int high);
void gui_history_range(const gui_history_t *history, unsigned channels, int minimum_span,
                       int *low, int *high);
#endif
