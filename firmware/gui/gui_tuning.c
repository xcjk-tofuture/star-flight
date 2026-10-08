#include "gui_tuning.h"
#include "settings_record.h"
#include <stdio.h>
#include <string.h>
/* The same panel/input renderer can be reused for another control module by
 * supplying its field descriptors and actual/setpoint/output model. */
static const gui_tuning_item_t heater_pages[2][4]={
    {{"T",UAV_SETTING_HEATER_TARGET,GUI_TUNE_NUMBER,1,10,"C"},
     {"P",UAV_SETTING_HEATER_KP,GUI_TUNE_NUMBER,2,100,""},
     {"I",UAV_SETTING_HEATER_KI,GUI_TUNE_NUMBER,3,1000,""},
     {"D",UAV_SETTING_HEATER_KD,GUI_TUNE_NUMBER,2,100,""}},
    {{"HEAT",UAV_SETTING_HEATER_ENABLED,GUI_TUNE_SWITCH_VALUE,0,1,""},
     {"MAX",UAV_SETTING_HEATER_LIMIT,GUI_TUNE_NUMBER,0,1,"%"},
     {"SAVE",0,GUI_TUNE_SAVE_VALUE,0,1,""}, {"RESET",0,GUI_TUNE_DEFAULTS_VALUE,0,1,""}}
};
static const char *const heater_titles[]={"调参","限制"};
const gui_tuning_module_t gui_tuning_heater={"IMU恒温",heater_pages,heater_titles,"T- SET: 64s","%",1000,10,40,2};
static const gui_tuning_item_t *item(const gui_tuning_t *p) { return &p->module->pages[p->page%p->module->page_count][p->selected%4]; }
void gui_tuning_init(gui_tuning_t *p) { memset(p,0,sizeof(*p)); p->module=&gui_tuning_heater; }
void gui_tuning_open(gui_tuning_t *p, const gui_tuning_module_t *module) {
    if (p->module!=module) { p->sampled=0; memset(&p->history,0,sizeof(p->history)); }
    p->module=module; p->page=0; p->selected=0; p->editing=0;
}
unsigned gui_tuning_field(const gui_tuning_t *p) { return item(p)->field; }
unsigned gui_tuning_context(const gui_tuning_t *p) { return p->page*16u+p->selected*2u+p->editing; }
void gui_tuning_select(gui_tuning_t *p, unsigned selection) { if (!p->editing && selection<4) p->selected=(uint8_t)selection; }
void gui_tuning_page(gui_tuning_t *p, unsigned page) { if (!p->editing && page<p->module->page_count) { p->page=(uint8_t)page; p->selected=0; } }
void gui_tuning_update(gui_tuning_t *p, const gui_tuning_model_t *m) {
    if (p->sampled && (uint32_t)(m->now_ms-p->sample_ms)<p->module->sample_period_ms) return;
    float values[3]={m->actual,m->setpoint,m->output};
    if (p->sampled && (uint32_t)(m->now_ms-p->sample_ms)>2u*p->module->sample_period_ms) {
        const float empty[3]={0}; gui_history_push(&p->history,empty,0,p->module->trace_scale);
    }
    gui_history_push(&p->history,values,(uint8_t)(m->valid ? 7:2),p->module->trace_scale);
    p->sampled=1; p->sample_ms=m->now_ms;
}
unsigned gui_tuning_input(gui_tuning_t *p, unsigned input, const gui_tuning_model_t *m) {
    const gui_tuning_item_t *f=item(p);
    if (p->editing) {
        if (input==GUI_TUNE_NEXT) return GUI_TUNE_INCREASE;
        if (input==GUI_TUNE_PREVIOUS) return GUI_TUNE_DECREASE;
        if (input==GUI_TUNE_ENTER) { p->editing=0; return GUI_TUNE_NONE; }
        if (input==GUI_TUNE_BACK || input==GUI_TUNE_SWITCH) { p->editing=0; return GUI_TUNE_CANCEL; }
        return GUI_TUNE_NONE;
    }
    if (input==GUI_TUNE_NEXT) p->selected=(uint8_t)((p->selected+1)%4);
    else if (input==GUI_TUNE_PREVIOUS) p->selected=(uint8_t)((p->selected+3)%4);
    else if (input==GUI_TUNE_SWITCH) gui_tuning_page(p,(p->page+1u)%p->module->page_count);
    else if (input==GUI_TUNE_BACK) return GUI_TUNE_EXIT;
    else if (input==GUI_TUNE_ENTER) {
        if (f->kind==GUI_TUNE_NUMBER) { p->editing=1; p->original=m->values[f->field]; }
        else if (f->kind==GUI_TUNE_SWITCH_VALUE) return GUI_TUNE_TOGGLE;
        else if (f->kind==GUI_TUNE_SAVE_VALUE) return GUI_TUNE_SAVE;
        else return GUI_TUNE_DEFAULTS;
    }
    return GUI_TUNE_NONE;
}
void gui_tuning_render(gui_tuning_t *p, gui_canvas_t *c, const gui_tuning_model_t *m) {
    gui_text(c,2,0,p->module->title,GUI_FONT_CN12);
    char page[12]; snprintf(page,sizeof(page),"%u/%u",p->page+1u,p->module->page_count);
    gui_text(c,61,4,page,GUI_FONT_TINY);
    const char *badge=m->save_state==1 ? "保存中":m->save_state==3 ? "保存失败"
        :p->editing ? "编辑":m->fault ? "控制异常"
        :m->save_state==2 && !m->dirty ? "已保存":!m->enabled ? "已关闭":p->module->page_titles[p->page];
    int width=gui_text_width(c,badge,GUI_FONT_CN12);
    gui_text(c,GUI_WIDTH-width-2,0,badge,GUI_FONT_CN12);
    if (m->dirty) gui_text(c,53,3,"*",GUI_FONT_TINY);
    gui_line(c,0,14,127,14); gui_line(c,53,16,53,56);
    for (unsigned row=0;row<4;row++) {
        const gui_tuning_item_t *f=&p->module->pages[p->page][row];
        int y=16+(int)row*10;
        uint8_t selected=p->selected==row;
        if (selected) { gui_box(c,0,y,52,10,1); gui_color(c,0); }
        char value[16];
        if (f->kind==GUI_TUNE_NUMBER) {
            unsigned raw=m->values[f->field];
            if (f->decimals) snprintf(value,sizeof(value),"%u.%0*u%s",raw/f->divisor,f->decimals,raw%f->divisor,f->suffix);
            else snprintf(value,sizeof(value),"%u%s",raw,f->suffix);
            if (!p->page) {
                int value_x=gui_text_width(c,f->label,GUI_FONT_BODY)+4; if (value_x<12) value_x=12;
                gui_text(c,2,y,f->label,GUI_FONT_BODY); gui_text(c,value_x,y,value,GUI_FONT_BODY);
            }
            else {
                char label[24]; snprintf(label,sizeof(label),"%s %s",f->label,value);
                gui_text(c,2,y+2,label,GUI_FONT_TINY);
            }
        } else if (f->kind==GUI_TUNE_SWITCH_VALUE) {
            char label[16]; snprintf(label,sizeof(label),"%s %s",f->label,m->values[f->field] ? "ON":"OFF");
            gui_text(c,2,y+2,label,GUI_FONT_TINY);
        }
        else gui_text(c,2,y,f->label,GUI_FONT_BODY);
        gui_color(c,1);
        if (selected && p->editing) { gui_color(c,0); gui_box(c,48,y+3,2,4,1); gui_color(c,1); }
    }
    gui_text(c,57,16,p->module->trace_label,GUI_FONT_TINY);
    int low,high; gui_history_range(&p->history,2,p->module->minimum_span,&low,&high);
    gui_plot(c,&p->history,56,24,72,23,1,low,high);
    /* Reference dots use a continuous history index. Dashing each short segment
     * independently would turn a dense 64-point trace into an indistinguishable solid line. */
    for (unsigned n=0;n<p->history.count;n+=3) {
        unsigned index=(p->history.head+GUI_HISTORY_SAMPLES-p->history.count+n)%GUI_HISTORY_SAMPLES;
        if (!(p->history.valid[index]&2u)) continue;
        int v=p->history.values[index][1]; if (v<low) v=low; if (v>high) v=high;
        int x=57+(int)(n*69u/(GUI_HISTORY_SAMPLES-1u));
        int y=45-(int)((int32_t)(v-low)*20/(high-low));
        gui_line(c,x,y,x,y);
    }
    char actual[12],output[12];
    if (m->valid) gui_fixed(actual,sizeof(actual),m->actual,1,0);
    else snprintf(actual,sizeof(actual),"--");
    if (m->fault) snprintf(output,sizeof(output),"E%u",m->fault);
    else gui_fixed(output,sizeof(output),m->output,0,0);
    gui_text(c,57,48,actual,GUI_FONT_BODY);
    gui_text(c,105,50,output,GUI_FONT_TINY);
    if (!m->fault) gui_text(c,121,50,p->module->output_suffix,GUI_FONT_TINY);
    gui_text(c,1,58,p->editing ? "1+ HOLD1- 2 DONE HOLD2 UNDO":"1 SELECT 2 EDIT HOLD2 PAGE",GUI_FONT_TINY);
}
