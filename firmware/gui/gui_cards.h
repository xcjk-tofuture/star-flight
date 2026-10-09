#ifndef STAR_GUI_CARDS_H
#define STAR_GUI_CARDS_H
#include "gui_menu.h"
enum { GUI_ICON_HEATER=0, GUI_ICON_INNER, GUI_ICON_OUTER, GUI_ICON_SPEED, GUI_ICON_HEIGHT, GUI_ICON_BACK };
/* Reuses menu selection/commands; only the visual presentation changes. */
void gui_cards_render(const gui_menu_t *menu, gui_canvas_t *canvas, const uint8_t *icons,
                      const char *title, const char *badge);
#endif
