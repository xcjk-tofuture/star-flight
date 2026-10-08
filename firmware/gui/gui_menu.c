#include "gui_menu.h"
#include <stdio.h>
#define MENU_VISIBLE_ROWS 3
#define MENU_ROW_PITCH 13
#define MENU_FIRST_Y 16
#define MENU_ROW_HEIGHT 13
void gui_menu_open(gui_menu_t *m, const gui_menu_item_t *items, uint8_t count, uint8_t selection) {
    m->items = items; m->count = count;
    m->selected = selection < count ? selection : 0;
    m->highlight_q8 = (int16_t)((MENU_FIRST_Y + (m->selected % MENU_VISIBLE_ROWS)*MENU_ROW_PITCH)*256);
    m->animation_ms = 0;
}
void gui_menu_next(gui_menu_t *m) {
    if (!m->count) return;
    unsigned old_window = m->selected/MENU_VISIBLE_ROWS;
    m->selected = (uint8_t)((m->selected+1) % m->count);
    if (old_window != m->selected/MENU_VISIBLE_ROWS)
        m->highlight_q8 = (int16_t)((MENU_FIRST_Y + (m->selected % MENU_VISIBLE_ROWS)*MENU_ROW_PITCH)*256);
}
void gui_menu_select(gui_menu_t *m, uint8_t selection) {
    if (!m->count || selection>=m->count) return;
    unsigned previous=m->selected/MENU_VISIBLE_ROWS;
    m->selected=selection;
    if (previous!=m->selected/MENU_VISIBLE_ROWS)
        m->highlight_q8=(int16_t)((MENU_FIRST_Y+(m->selected%MENU_VISIBLE_ROWS)*MENU_ROW_PITCH)*256);
}
void gui_menu_previous(gui_menu_t *m) {
    if (m->count) gui_menu_select(m,(uint8_t)((m->selected+m->count-1)%m->count));
}
void gui_menu_render(gui_menu_t *m, gui_canvas_t *c, const char *title,
                      const char *status, uint32_t now) {
    if (!m->items || !m->count) return;
    gui_text(c, 2, 0, title, GUI_FONT_CN12);
    if (status) {
        int width = gui_text_width(c, status, GUI_FONT_CN12);
        if (width <= 48) gui_text(c, 106-width, 0, status, GUI_FONT_CN12);
    }
    char count[12]; snprintf(count, sizeof(count), "%u/%u", m->selected+1u, m->count);
    gui_text(c, 114, 4, count, GUI_FONT_TINY);
    gui_line(c, 0, 14, 127, 14);
    int target = (MENU_FIRST_Y + (m->selected % MENU_VISIBLE_ROWS)*MENU_ROW_PITCH)*256;
    uint32_t dt = m->animation_ms ? (uint32_t)(now-m->animation_ms) : 100;
    if (dt > 100) dt = 100;
    m->animation_ms = now;
    m->highlight_q8 += (int16_t)((target-m->highlight_q8)*(int32_t)dt / (80+(int32_t)dt));
    if (target-m->highlight_q8 < 128 && target-m->highlight_q8 > -128) m->highlight_q8 = (int16_t)target;
    u8g2_DrawRBox(&c->graphics, 2, m->highlight_q8/256, 118, MENU_ROW_HEIGHT, 2);
    gui_color(c, 2); /* XOR keeps text legible as the highlight moves between rows. */
    unsigned first = m->selected/MENU_VISIBLE_ROWS*MENU_VISIBLE_ROWS;
    for (unsigned row = 0; row < MENU_VISIBLE_ROWS && first+row < m->count; row++) {
        int y = MENU_FIRST_Y+(int)row*MENU_ROW_PITCH;
        gui_text(c, 18, y, m->items[first+row].label, GUI_FONT_CN12);
        if (first+row == m->selected) {
            gui_line(c, 8, y+3, 12, y+6); gui_line(c, 12, y+6, 8, y+9);
        }
    }
    gui_color(c, 1);
    gui_line(c, 124, 17, 124, 54);
    int thumb_height = 38*MENU_VISIBLE_ROWS/m->count;
    if (thumb_height > 38) thumb_height = 38;
    if (thumb_height < 4) thumb_height = 4;
    int thumb_y = 17 + (m->count > 1 ? m->selected*(38-thumb_height)/(m->count-1) : 0);
    gui_box(c, 123, thumb_y, 3, thumb_height, 1);
    gui_text(c, 1, 58, "1 NEXT  2 OK  HOLD1 BACK", GUI_FONT_TINY);
}
