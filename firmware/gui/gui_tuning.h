#ifndef STAR_GUI_TUNING_H
#define STAR_GUI_TUNING_H
#include "gui_plot.h"
/* Reusable workspace: module -> sibling pages -> focus/edit, with a live response plot.
 * UI returns actions. The application owns parameter validation and storage. */
enum { GUI_TUNE_NEXT=1, GUI_TUNE_PREVIOUS, GUI_TUNE_ENTER, GUI_TUNE_BACK, GUI_TUNE_SWITCH };
enum { GUI_TUNE_NONE=0, GUI_TUNE_INCREASE, GUI_TUNE_DECREASE, GUI_TUNE_CANCEL,
       GUI_TUNE_SAVE, GUI_TUNE_DEFAULTS, GUI_TUNE_TOGGLE, GUI_TUNE_EXIT };
enum { GUI_TUNE_NUMBER=0, GUI_TUNE_SWITCH_VALUE, GUI_TUNE_SAVE_VALUE, GUI_TUNE_DEFAULTS_VALUE };
typedef struct { const char *label; uint8_t field, kind, decimals; uint16_t divisor; const char *suffix; } gui_tuning_item_t;
typedef struct {
    const char *title;
    const gui_tuning_item_t (*pages)[4];
    const char *const *page_titles;
    const char *trace_label, *output_suffix;
    uint16_t sample_period_ms;
    float trace_scale;
    int minimum_span;
    uint8_t page_count;
} gui_tuning_module_t;
extern const gui_tuning_module_t gui_tuning_heater;
typedef struct {
    const gui_tuning_module_t *module;
    gui_history_t history;
    uint32_t sample_ms;
    uint16_t original;
    uint8_t page, selected, editing, sampled;
} gui_tuning_t;
typedef struct {
    const uint16_t *values;
    uint32_t now_ms;
    float actual, setpoint, output;
    uint8_t valid, fault, dirty, save_state, enabled, monitor_only;
} gui_tuning_model_t;
void gui_tuning_init(gui_tuning_t *panel);
void gui_tuning_open(gui_tuning_t *panel, const gui_tuning_module_t *module);
void gui_tuning_update(gui_tuning_t *panel, const gui_tuning_model_t *model);
unsigned gui_tuning_input(gui_tuning_t *panel, unsigned input, const gui_tuning_model_t *model);
unsigned gui_tuning_field(const gui_tuning_t *panel);
unsigned gui_tuning_context(const gui_tuning_t *panel);
void gui_tuning_select(gui_tuning_t *panel, unsigned selection);
void gui_tuning_page(gui_tuning_t *panel, unsigned page);
void gui_tuning_render(gui_tuning_t *panel, gui_canvas_t *canvas, const gui_tuning_model_t *model);
#endif
