#include "gui_plot.h"
#include <limits.h>
#include <math.h>
#include <stdlib.h>
static void trace_segment(gui_canvas_t *c, int x0, int y0, int x1, int y1, unsigned channel) {
    if (!channel) { gui_line(c,x0,y0,x1,y1); return; }
    int dx=abs(x1-x0),dy=abs(y1-y0),steps=dx>dy ? dx:dy;
    unsigned period=channel==1 ? 4:6, on=channel==1 ? 2:1;
    for (int i=0;i<=steps;i++) {
        int x=steps ? x0+(x1-x0)*i/steps:x0;
        int y=steps ? y0+(y1-y0)*i/steps:y0;
        /* Anchor the pattern to display X. Restarting it on every short
         * sample segment made dense secondary traces look solid. */
        if ((unsigned)x%period<on) gui_line(c,x,y,x,y);
    }
}
void gui_history_push(gui_history_t *h, const float values[3], uint8_t valid, float scale) {
    uint8_t mask = 0;
    for (unsigned i = 0; i < GUI_HISTORY_CHANNELS; i++) {
        float v = values[i] * scale;
        if ((valid & (1u << i)) && isfinite(v) && fabsf(v) <= INT16_MAX) {
            h->values[h->head][i] = (int16_t)v;
            mask |= (uint8_t)(1u << i);
        } else h->values[h->head][i] = 0;
    }
    h->valid[h->head] = mask;
    h->head = (uint8_t)((h->head + 1u) % GUI_HISTORY_SAMPLES);
    if (h->count < GUI_HISTORY_SAMPLES) h->count++;
}
void gui_history_range(const gui_history_t *h, unsigned channels, int span, int *low, int *high) {
    int a = INT16_MAX, b = INT16_MIN;
    if (channels > GUI_HISTORY_CHANNELS) channels = GUI_HISTORY_CHANNELS;
    for (unsigned n = 0; n < h->count; n++)
        for (unsigned ch = 0; ch < channels; ch++)
            if (h->valid[n] & (1u << ch)) {
                if (h->values[n][ch] < a) a = h->values[n][ch];
                if (h->values[n][ch] > b) b = h->values[n][ch];
            }
    if (a > b) { *low = -span / 2; *high = span / 2; return; }
    int center = (a + b) / 2, range = b - a;
    if (range < span) range = span;
    range += range / 5 + 2;
    *low = center - range / 2; *high = center + range / 2;
}
void gui_plot(gui_canvas_t *c, const gui_history_t *h, int x, int y, int w, int height,
              unsigned channels, int low, int high) {
    if (w < 3 || height < 3 || high <= low) return;
    if (channels > GUI_HISTORY_CHANNELS) channels = GUI_HISTORY_CHANNELS;
    gui_box(c, x, y, w, height, 0);
    for (int i = 1; i < 4; i++)
        gui_dashed_line(c, x + 1, y + height * i / 4, x + w - 2, y + height * i / 4, 5);
    for (unsigned ch = 0; ch < channels; ch++) {
        int last_x = 0, last_y = 0, last_valid = 0;
        for (unsigned n = 0; n < h->count; n++) {
            unsigned index = (h->head + GUI_HISTORY_SAMPLES - h->count + n) % GUI_HISTORY_SAMPLES;
            int valid = (h->valid[index] & (1u << ch)) != 0;
            int value = h->values[index][ch];
            if (value < low) value = low;
            if (value > high) value = high;
            int px = x + 1 + (int)(n * (unsigned)(w - 3) / (GUI_HISTORY_SAMPLES - 1u));
            int py = y + height - 2 - (int)((int32_t)(value - low) * (height - 3) / (high - low));
            if (valid && last_valid) trace_segment(c,last_x,last_y,px,py,ch);
            last_x = px; last_y = py; last_valid = valid;
        }
    }
}
