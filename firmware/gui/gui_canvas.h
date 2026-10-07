#ifndef STAR_GUI_CANVAS_H
#define STAR_GUI_CANVAS_H
#include "u8g2.h"
#include <stddef.h>
#include <stdint.h>
#define GUI_WIDTH 128
#define GUI_HEIGHT 64
#define GUI_FRAME_BYTES (GUI_WIDTH * GUI_HEIGHT / 8)
typedef enum { GUI_FONT_TINY, GUI_FONT_BODY, GUI_FONT_LARGE,
               GUI_FONT_CN12, GUI_FONT_CN16 } gui_font_t;
typedef struct {
    u8g2_t graphics;
    uint8_t pixels[GUI_FRAME_BYTES];
} gui_canvas_t;
void gui_canvas_init(gui_canvas_t *canvas);
void gui_clear(gui_canvas_t *canvas);
void gui_text(gui_canvas_t *canvas, int x, int y, const char *text, gui_font_t font);
int gui_text_width(gui_canvas_t *canvas, const char *text, gui_font_t font);
void gui_color(gui_canvas_t *canvas, uint8_t color);
void gui_line(gui_canvas_t *canvas, int x0, int y0, int x1, int y1);
void gui_dashed_line(gui_canvas_t *canvas, int x0, int y0, int x1, int y1, unsigned pattern);
void gui_box(gui_canvas_t *canvas, int x, int y, int width, int height, int filled);
void gui_circle(gui_canvas_t *canvas, int x, int y, unsigned radius);
void gui_bar(gui_canvas_t *canvas, int x, int y, int width, int height, int value, int low, int high);
void gui_fixed(char *out, size_t capacity, float value, unsigned decimals, int show_sign);
typedef int (*gui_write_span_fn)(void *context, uint8_t page, uint8_t x,
                                 const uint8_t *bytes, uint8_t count);
typedef struct {
    uint32_t frames, spans, bytes, errors;
    uint16_t last_bytes;
} gui_present_stats_t;
typedef struct {
    uint8_t previous[GUI_FRAME_BYTES], valid_pages;
    gui_write_span_fn write;
    void *context;
    gui_present_stats_t stats;
} gui_presenter_t;
void gui_presenter_init(gui_presenter_t *presenter, gui_write_span_fn write, void *context);
void gui_presenter_invalidate(gui_presenter_t *presenter);
void gui_present(gui_presenter_t *presenter, const gui_canvas_t *canvas);
#endif
