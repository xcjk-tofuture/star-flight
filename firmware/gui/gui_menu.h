#ifndef STAR_GUI_MENU_H
#define STAR_GUI_MENU_H
#include "gui_canvas.h"
typedef struct { const char *label; uint8_t action; } gui_menu_item_t;
typedef struct {
    const gui_menu_item_t *items;
    uint32_t animation_ms;
    int16_t highlight_q8;
    uint8_t count, selected;
} gui_menu_t;
void gui_menu_open(gui_menu_t *menu, const gui_menu_item_t *items, uint8_t count, uint8_t selection);
void gui_menu_next(gui_menu_t *menu);
void gui_menu_render(gui_menu_t *menu, gui_canvas_t *canvas, const char *title,
                      const char *status, uint32_t now_ms);
#endif
