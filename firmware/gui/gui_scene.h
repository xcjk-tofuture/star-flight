#ifndef STAR_GUI_SCENE_H
#define STAR_GUI_SCENE_H
#include "gui_canvas.h"
/* Euler angles in radians, body axes forward/right/down; pure presentation. */
void gui_drone_3d(gui_canvas_t *canvas, float roll, float pitch, float yaw, int x, int y, float scale);
void gui_horizon(gui_canvas_t *canvas, float roll, float pitch, int x, int y, int width, int height);
#endif
