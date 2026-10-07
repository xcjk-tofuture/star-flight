#include "gui_canvas.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static const u8x8_display_info_t geometry = {
    .tile_width = GUI_WIDTH / 8, .tile_height = GUI_HEIGHT / 8,
    .pixel_width = GUI_WIDTH, .pixel_height = GUI_HEIGHT
};
extern const uint8_t gui_font_cn12[], gui_font_cn16[];
static const uint8_t *font_data(gui_font_t font) {
    return font == GUI_FONT_CN12 ? gui_font_cn12
         : font == GUI_FONT_CN16 ? gui_font_cn16
         : font == GUI_FONT_TINY ? u8g2_font_4x6_tf
         : font == GUI_FONT_LARGE ? u8g2_font_9x15_tf : u8g2_font_6x10_tf;
}
void gui_canvas_init(gui_canvas_t *c) {
    memset(c, 0, sizeof(*c));
    /* U8g2 is used as a pure graphics engine; no hardware callbacks needed. */
    u8g2_GetU8x8(&c->graphics)->display_info = &geometry;
    u8g2_SetupBuffer(&c->graphics, c->pixels, GUI_HEIGHT / 8,
                     u8g2_ll_hvline_vertical_top_lsb, U8G2_R0);
    u8g2_SetFontMode(&c->graphics, 1);
    u8g2_SetFontPosTop(&c->graphics);
}
void gui_clear(gui_canvas_t *c) {
    u8g2_ClearBuffer(&c->graphics);
    u8g2_SetDrawColor(&c->graphics, 1);
    u8g2_SetMaxClipWindow(&c->graphics);
}
void gui_color(gui_canvas_t *c, uint8_t color) { u8g2_SetDrawColor(&c->graphics, color); }
void gui_text(gui_canvas_t *c, int x, int y, const char *text, gui_font_t font) {
    if (!text || x < 0 || y < 0 || x >= GUI_WIDTH || y >= GUI_HEIGHT)
        return;
    u8g2_SetFont(&c->graphics, font_data(font));
    if (font == GUI_FONT_CN12 || font == GUI_FONT_CN16)
        u8g2_SetFontRefHeightAll(&c->graphics);
    else u8g2_SetFontRefHeightText(&c->graphics);
    u8g2_SetFontMode(&c->graphics, 1);
    u8g2_SetFontPosTop(&c->graphics);
    u8g2_DrawUTF8(&c->graphics, (u8g2_uint_t)x, (u8g2_uint_t)y, text);
}
int gui_text_width(gui_canvas_t *c, const char *text, gui_font_t font) {
    u8g2_SetFont(&c->graphics, font_data(font));
    return text ? u8g2_GetUTF8Width(&c->graphics, text) : 0;
}
static unsigned outcode(int x, int y) {
    return (x < 0 ? 1u : x >= GUI_WIDTH ? 2u : 0u) |
           (y < 0 ? 4u : y >= GUI_HEIGHT ? 8u : 0u);
}
void gui_line(gui_canvas_t *c, int x0, int y0, int x1, int y1) {
    /* Clip signed coordinates before passing them to U8g2's unsigned API. */
    for (unsigned n = 0; n < 8; n++) {
        unsigned a = outcode(x0, y0), b = outcode(x1, y1), code = a ? a : b;
        if (!(a | b)) {
            u8g2_DrawLine(&c->graphics, x0, y0, x1, y1);
            return;
        }
        if (a & b)
            return;
        int x, y;
        if (code & (4u | 8u)) {
            y = code & 4u ? 0 : GUI_HEIGHT - 1;
            x = x0 + (int)((int64_t)(x1 - x0) * (y - y0) / (y1 - y0));
        } else {
            x = code & 1u ? 0 : GUI_WIDTH - 1;
            y = y0 + (int)((int64_t)(y1 - y0) * (x - x0) / (x1 - x0));
        }
        if (a) { x0 = x; y0 = y; }
        else { x1 = x; y1 = y; }
    }
}
void gui_dashed_line(gui_canvas_t *c, int x0, int y0, int x1, int y1, unsigned pattern) {
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int steps = dx > dy ? dx : dy;
    if (steps > 1024) return;
    for (int i = 0; i <= steps; i++)
        if (!pattern || (unsigned)i % (pattern + 2u) < 2u) {
            int x = steps ? x0 + (x1 - x0) * i / steps : x0;
            int y = steps ? y0 + (y1 - y0) * i / steps : y0;
            if (!outcode(x, y)) u8g2_DrawPixel(&c->graphics, x, y);
        }
}
void gui_box(gui_canvas_t *c, int x, int y, int w, int h, int filled) {
    if (w <= 0 || h <= 0) return;
    if (!filled) {
        gui_line(c, x, y, x + w - 1, y); gui_line(c, x, y, x, y + h - 1);
        gui_line(c, x, y + h - 1, x + w - 1, y + h - 1);
        gui_line(c, x + w - 1, y, x + w - 1, y + h - 1);
    } else {
        int right = x + w, bottom = y + h;
        if (x < 0) x = 0;
        if (y < 0) y = 0;
        if (right > GUI_WIDTH) right = GUI_WIDTH;
        if (bottom > GUI_HEIGHT) bottom = GUI_HEIGHT;
        if (right > x && bottom > y) u8g2_DrawBox(&c->graphics, x, y, right - x, bottom - y);
    }
}
void gui_circle(gui_canvas_t *c, int x, int y, unsigned radius) {
    if (x >= (int)radius && y >= (int)radius && x + (int)radius < GUI_WIDTH && y + (int)radius < GUI_HEIGHT)
        u8g2_DrawCircle(&c->graphics, x, y, radius, U8G2_DRAW_ALL);
}
void gui_bar(gui_canvas_t *c, int x, int y, int w, int h, int value, int low, int high) {
    gui_box(c, x, y, w, h, 0);
    if (high <= low || w < 3 || h < 3) return;
    if (value < low) value = low;
    if (value > high) value = high;
    gui_box(c, x + 1, y + 1, (int)((int64_t)(value - low) * (w - 2) / (high - low)), h - 2, 1);
}
void gui_fixed(char *out, size_t capacity, float value, unsigned decimals, int show_sign) {
    if (!out || !capacity) return;
    if (!isfinite(value) || fabsf(value) > 1000000.0f) { snprintf(out, capacity, "--"); return; }
    if (decimals > 2) decimals = 2;
    int scale = decimals == 2 ? 100 : decimals == 1 ? 10 : 1;
    long scaled = (long)(fabsf(value) * scale + 0.5f);
    const char *sign = value < 0 ? "-" : show_sign ? "+" : "";
    if (decimals)
        snprintf(out, capacity, "%s%ld.%0*ld", sign, scaled / scale, (int)decimals, scaled % scale);
    else snprintf(out, capacity, "%s%ld", sign, scaled);
}
void gui_presenter_init(gui_presenter_t *p, gui_write_span_fn write, void *context) {
    memset(p, 0, sizeof(*p)); p->write = write; p->context = context;
}
void gui_presenter_invalidate(gui_presenter_t *p) { p->valid_pages = 0; }
void gui_present(gui_presenter_t *p, const gui_canvas_t *c) {
    p->stats.last_bytes = 0;
    for (unsigned page = 0; page < GUI_HEIGHT / 8; page++) {
        const uint8_t *next = c->pixels + page * GUI_WIDTH;
        uint8_t *old = p->previous + page * GUI_WIDTH;
        unsigned first = 0, last = GUI_WIDTH;
        if (p->valid_pages & (1u << page)) {
            while (first < last && next[first] == old[first]) first++;
            while (last > first && next[last - 1] == old[last - 1]) last--;
        }
        if (last == first) continue;
        if (!p->write || p->write(p->context, page, first, next + first, last - first) != 0) {
            /* A failed transfer may have changed part of the physical page.
             * Force a whole-page retry even if the next image returns to old data. */
            p->valid_pages &= (uint8_t)~(1u << page);
            p->stats.errors++;
            continue;
        }
        memcpy(old + first, next + first, last - first);
        p->valid_pages |= (uint8_t)(1u << page);
        p->stats.spans++; p->stats.bytes += last - first;
        p->stats.last_bytes += (uint16_t)(last - first);
    }
    p->stats.frames++;
}
