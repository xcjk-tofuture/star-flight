#include "gui_scene.h"
#include <math.h>
typedef struct { float x, y, z; } vertex_t;
typedef struct { float r[3][3]; int x, y; float scale; } camera_t;
static camera_t camera(float roll, float pitch, float yaw, int x, int y, float scale) {
    float cr = cosf(roll), sr = sinf(roll), cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw), sy = sinf(yaw);
    camera_t c = {.r = {{cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr},
                       {sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr},
                       {-sp, cp*sr, cp*cr}}, .x = x, .y = y, .scale = scale};
    return c;
}
static void project(const camera_t *c, vertex_t v, int *x, int *y) {
    float a = c->r[0][0]*v.x + c->r[0][1]*v.y + c->r[0][2]*v.z;
    float b = c->r[1][0]*v.x + c->r[1][1]*v.y + c->r[1][2]*v.z;
    float d = c->r[2][0]*v.x + c->r[2][1]*v.y + c->r[2][2]*v.z;
    /* Fixed camera basis plus perspective; denominator remains >3 for this mesh. */
    float depth = 5.0f + 0.55f*a - 0.55f*b - 0.63f*d;
    float gain = 5.0f*c->scale / depth;
    *x = c->x + (int)((0.7071f*a + 0.7071f*b)*gain);
    *y = c->y + (int)((-0.445f*a + 0.445f*b + 0.777f*d)*gain);
}
static void edge(gui_canvas_t *c, const camera_t *cam, vertex_t a, vertex_t b) {
    int x0, y0, x1, y1;
    project(cam, a, &x0, &y0); project(cam, b, &x1, &y1);
    gui_line(c, x0, y0, x1, y1);
}
void gui_drone_3d(gui_canvas_t *c, float roll, float pitch, float yaw, int x, int y, float scale) {
    if (!isfinite(roll) || !isfinite(pitch) || !isfinite(yaw)) return;
    camera_t cam = camera(roll, pitch, yaw, x, y, scale);
    static const vertex_t body[] = {{.30f, .20f, 0}, {-.30f, .20f, 0},
                                    {-.30f, -.20f, 0}, {.30f, -.20f, 0}};
    for (unsigned i = 0; i < 4; i++) {
        edge(c, &cam, body[i], body[(i+1)%4]);
        vertex_t motor = {(i == 0 || i == 3) ? .78f : -.78f, i < 2 ? .78f : -.78f, 0};
        edge(c, &cam, body[i], motor);
        vertex_t last = {motor.x + .19f, motor.y, 0};
        for (unsigned n = 1; n <= 8; n++) {
            float angle = n * 0.785398163f;
            vertex_t next = {motor.x + .19f*cosf(angle), motor.y + .19f*sinf(angle), 0};
            edge(c, &cam, last, next); last = next;
        }
        edge(c, &cam, (vertex_t){motor.x-.12f, motor.y, 0},
                     (vertex_t){motor.x+.12f, motor.y, 0});
    }
    /* Distinct forward arrow and lower body rail make orientation readable. */
    edge(c, &cam, (vertex_t){0, 0, -.10f}, (vertex_t){.58f, 0, -.10f});
    edge(c, &cam, (vertex_t){.58f, 0, -.10f}, (vertex_t){.40f, .12f, -.10f});
    edge(c, &cam, (vertex_t){.58f, 0, -.10f}, (vertex_t){.40f, -.12f, -.10f});
    edge(c, &cam, (vertex_t){-.30f, .20f, .20f}, (vertex_t){.30f, .20f, .20f});
    edge(c, &cam, body[0], (vertex_t){.30f, .20f, .20f});
    edge(c, &cam, body[1], (vertex_t){-.30f, .20f, .20f});
}
void gui_horizon(gui_canvas_t *c, float roll, float pitch, int x, int y, int w, int h) {
    if (!isfinite(roll) || !isfinite(pitch)) return;
    int cx = x + w/2, cy = y + h/2;
    u8g2_SetClipWindow(&c->graphics, x, y, x+w, y+h);
    float cr = cosf(roll), sr = sinf(roll);
    for (int mark = -2; mark <= 2; mark++) {
        float offset = pitch * 25.0f + mark * 8.0f;
        float span = mark == 0 ? w*0.46f : w*0.18f;
        gui_line(c, cx+(int)(-span*cr-offset*sr), cy+(int)(span*sr-offset*cr),
                    cx+(int)(span*cr-offset*sr), cy+(int)(-span*sr-offset*cr));
    }
    gui_line(c, cx-13, cy, cx-4, cy); gui_line(c, cx+4, cy, cx+13, cy);
    gui_line(c, cx-4, cy, cx, cy+3); gui_line(c, cx, cy+3, cx+4, cy);
    u8g2_SetMaxClipWindow(&c->graphics);
    gui_box(c, x, y, w, h, 0);
}
