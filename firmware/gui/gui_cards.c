#include "gui_cards.h"
#include <stdio.h>
static void icon(gui_canvas_t *c, unsigned kind, int x, int y) {
    if (kind==GUI_ICON_HEATER) {
        gui_box(c,x-3,y-12,7,19,0); gui_circle(c,x,y+8,5);
        gui_line(c,x,y-7,x,y+9); gui_box(c,x-2,y+6,5,5,1);
        for (int n=-1;n<=1;n++) gui_line(c,x+7,y+n*6,x+11,y+n*6);
    } else if (kind==GUI_ICON_INNER) {
        gui_circle(c,x,y,11); gui_line(c,x-13,y,x+13,y); gui_line(c,x,y-13,x,y+13);
        gui_circle(c,x,y,4); gui_box(c,x-1,y-1,3,3,1);
    } else if (kind==GUI_ICON_OUTER) {
        gui_line(c,x-13,y,x+13,y); gui_line(c,x,y-10,x,y+10);
        gui_line(c,x-8,y-8,x+8,y+8); gui_line(c,x+8,y-8,x-8,y+8);
        gui_circle(c,x-9,y-9,4); gui_circle(c,x+9,y-9,4);
        gui_circle(c,x-9,y+9,4); gui_circle(c,x+9,y+9,4); gui_box(c,x-2,y-2,5,5,1);
    } else if (kind==GUI_ICON_SPEED) {
        gui_circle(c,x,y,13); gui_line(c,x-13,y+7,x+13,y+7);
        gui_line(c,x,y+3,x+8,y-7); gui_circle(c,x,y+3,2);
        gui_line(c,x,y-12,x,y-8); gui_line(c,x-10,y-7,x-7,y-4); gui_line(c,x+10,y-7,x+7,y-4);
    } else if (kind==GUI_ICON_HEIGHT) {
        gui_line(c,x-7,y-12,x-7,y+12); gui_line(c,x-7,y-12,x-11,y-7); gui_line(c,x-7,y-12,x-3,y-7);
        gui_line(c,x-7,y+12,x-11,y+7); gui_line(c,x-7,y+12,x-3,y+7);
        gui_line(c,x+5,y-12,x+5,y+12);
        for (int n=-2;n<=2;n++) gui_line(c,x+5,y+n*5,x+(n%2 ? 9:12),y+n*5);
    } else {
        gui_line(c,x-11,y,x+9,y); gui_line(c,x-11,y,x-4,y-7); gui_line(c,x-11,y,x-4,y+7);
        gui_line(c,x+9,y,x+9,y+10); gui_line(c,x-2,y+10,x+9,y+10);
    }
}
static void card(gui_canvas_t *c, unsigned kind, int x, int width, int selected) {
    gui_box(c,x,16,width,28,selected);
    if (selected) gui_color(c,0);
    icon(c,kind,x+width/2,30);
    gui_color(c,1);
}
void gui_cards_render(const gui_menu_t *m, gui_canvas_t *c, const uint8_t *icons,
                      const char *title, const char *badge) {
    if (!m->count || !m->items || !icons) return;
    gui_text(c,2,0,title,GUI_FONT_CN12);
    if (badge) {
        int width=gui_text_width(c,badge,GUI_FONT_CN12);
        if (width<=48) gui_text(c,106-width,0,badge,GUI_FONT_CN12);
    }
    char index[12]; snprintf(index,sizeof(index),"%u/%u",m->selected+1u,m->count);
    gui_text(c,GUI_WIDTH-gui_text_width(c,index,GUI_FONT_TINY)-1,4,index,GUI_FONT_TINY);
    gui_line(c,0,14,127,14);
    unsigned previous=(m->selected+m->count-1u)%m->count, next=(m->selected+1u)%m->count;
    card(c,icons[previous],2,34,0); card(c,icons[m->selected],43,42,1); card(c,icons[next],92,34,0);
    const char *label=m->items[m->selected].label;
    gui_text(c,(GUI_WIDTH-gui_text_width(c,label,GUI_FONT_CN12))/2,44,label,GUI_FONT_CN12);
    gui_text(c,1,58,"1 NEXT 2 OK HOLD1 BACK",GUI_FONT_TINY);
}
