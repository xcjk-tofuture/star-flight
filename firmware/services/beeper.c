#include "beeper.h"
#include "beeper_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log_service.h"
typedef struct { uint16_t hz, ms; } tone_t;
static const tone_t click[]={{2200,25}},back[]={{1200,50}},accept[]={{2200,70}},
    start[]={{1800,70},{0,60},{1800,70}},
    done[]={{1800,80},{0,50},{2200,80},{0,50},{2600,100}},
    link[]={{1600,60},{0,50},{2200,60}},
    arm[]={{1800,120},{0,80},{2600,160}},
    lock[]={{1600,100},{0,60},{1100,120}},
    failure[]={{1200,120},{0,80},{1200,120},{0,80},{1200,120}},
    alarm[]={{1300,160},{0,100},{1300,160},{0,900}};
typedef struct { const tone_t *tones; uint8_t count; } pattern_t;
static const pattern_t patterns[]={{click,1},{back,1},{accept,1},{start,3},{done,5},
    {link,3},{arm,3},{lock,3},{failure,5},{alarm,4}};
static volatile uint32_t pending;
static volatile uint8_t alarm_active;
static uint8_t enabled, playing, event, step;
static uint32_t step_ms;
void uav_beeper_init(void) {
    enabled=uav_beeper_port_init()==0;
    uav_logf(enabled ? "INFO":"WARN","BEEPER","PB4 TIM3_CH1 passive ready=%u nonblocking=1",enabled);
    if (enabled) uav_beeper_request(UAV_BEEP_START);
}
void uav_beeper_request(unsigned e) {
    if (e>UAV_BEEP_FAILURE) return;
    taskENTER_CRITICAL(); pending|=1u<<e; taskEXIT_CRITICAL();
}
void uav_beeper_alarm(uint8_t active) { alarm_active=!!active; }
void uav_beeper_tick(uint32_t now) {
    if (!enabled) return;
    if (playing && event==9 && !alarm_active) { playing=0; uav_beeper_port_tone(0); }
    uint32_t requests;
    taskENTER_CRITICAL(); requests=pending; taskEXIT_CRITICAL();
    unsigned next=0; int have=0;
    for (unsigned i=0;i<=UAV_BEEP_FAILURE;i++) if (requests & (1u<<i)) { next=i; have=1; }
    if (alarm_active) { next=9; have=!playing || event!=9; }
    if (have && (!playing || next>event)) {
        if (next<9) { taskENTER_CRITICAL(); pending&=~(1u<<next); taskEXIT_CRITICAL(); }
        if (next>=UAV_BEEP_START) { taskENTER_CRITICAL(); pending&=~7u; taskEXIT_CRITICAL(); }
        playing=1; event=(uint8_t)next; step=0; step_ms=now;
        uav_beeper_port_tone(patterns[event].tones[0].hz);
    }
    if (!playing) return;
    const pattern_t *p=&patterns[event];
    if ((uint32_t)(now-step_ms)<p->tones[step].ms) return;
    if (++step>=p->count) { playing=0; uav_beeper_port_tone(0); return; }
    step_ms=now; uav_beeper_port_tone(p->tones[step].hz);
}
