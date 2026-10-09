#include "imu_heater.h"
#include "app_settings.h"
#include "sensor_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log_service.h"
#include "flash_proc.h"
#include <math.h>
#include <string.h>
static uav_heater_stats_t stats;
static uav_settings_values_t previous;
static float raw_temperature, integral, previous_filtered, filtered_rate, rise_start_temp;
static uint32_t sample_ms, control_ms, stable_ms, warm_ms, rise_ms, report_ms;
static uint8_t initialized, valid_sample, have_sample, filtering, stable_active, rise_active, fault;
static volatile uint8_t reset_requested;
static float clamp(float x, float low, float high) { return x<low ? low:x>high ? high:x; }
void uav_imu_heater_stats(uav_heater_stats_t *out) {
    taskENTER_CRITICAL(); *out=stats; taskEXIT_CRITICAL();
}
void uav_imu_heater_init(void) {
    uav_device_heater_init(); initialized=1;
    uav_logf("INFO","HEATER","PB8 TIM10_CH1 PWM=1000Hz loop=20Hz temp=10Hz stale=500ms hard_limit=60C enabled_default=0");
}
void uav_imu_heater_sample(float temperature, uint8_t valid, uint32_t now) {
    raw_temperature=temperature; sample_ms=now; have_sample=1;
    valid_sample=valid && isfinite(temperature) && temperature>=-40 && temperature<=85;
}
void uav_imu_heater_pause(void) {
    if (!initialized) return;
    taskENTER_CRITICAL(); uav_device_heater_write(0); stats.duty_percent=0;
    stats.temperature_valid=0;
    if (stats.state==UAV_HEATER_WARMING || stats.state==UAV_HEATER_READY || stats.state==UAV_HEATER_MONITOR) stats.state=UAV_HEATER_WAIT;
    reset_requested=1; taskEXIT_CRITICAL();
}
void uav_imu_heater_watchdog(uint32_t now) {
    if (!initialized) return;
    if (uav_storage_busy()) { uav_imu_heater_pause(); return; }
    uav_settings_snapshot_t s; uav_settings_snapshot(&s);
    taskENTER_CRITICAL();
    if (!s.values.value[UAV_SETTING_HEATER_ENABLED] || !stats.sample_ms || (uint32_t)(now-stats.sample_ms)>500u) {
        uav_device_heater_write(0); stats.duty_percent=0;
        if (!stats.sample_ms || (uint32_t)(now-stats.sample_ms)>500u) stats.temperature_valid=0;
        if (!s.values.value[UAV_SETTING_HEATER_ENABLED]) stats.state=UAV_HEATER_OFF;
        else if (stats.state==UAV_HEATER_WARMING || stats.state==UAV_HEATER_READY || stats.state==UAV_HEATER_MONITOR) {
            stats.state=UAV_HEATER_FAULT; stats.fault=UAV_HEATER_SENSOR;
        }
    }
    taskEXIT_CRITICAL();
}
void uav_imu_heater_tick(uint32_t now) {
    if (!initialized) return;
    uav_settings_snapshot_t snapshot; uav_settings_snapshot(&snapshot);
    const uav_settings_values_t *s=&snapshot.values;
    int changed=memcmp(s->value+UAV_SETTING_HEATER_ENABLED,previous.value+UAV_SETTING_HEATER_ENABLED,
        (UAV_SETTING_COUNT-UAV_SETTING_HEATER_ENABLED)*sizeof(uint16_t))!=0;
    uint8_t reset;
    taskENTER_CRITICAL(); reset=reset_requested; reset_requested=0; taskEXIT_CRITICAL();
    if (changed || reset) {
        previous=*s; integral=filtered_rate=0; stable_active=rise_active=0; warm_ms=now;
        control_ms=now; filtering=0;
    }
    float dt=(uint32_t)(now-control_ms)*.001f; control_ms=now;
    uav_heater_stats_t next; uav_imu_heater_stats(&next);
    uint8_t previous_fault=next.state==UAV_HEATER_FAULT ? next.fault:0;
    uint8_t was_ready=next.state==UAV_HEATER_READY && !changed && !reset;
    next.target=s->value[UAV_SETTING_HEATER_TARGET]*.1f;
    next.duty_percent=next.p=next.d=0;
    next.temperature_valid=have_sample && valid_sample && (uint32_t)(now-sample_ms)<=500u;
    if (next.temperature_valid) {
        if (!filtering || dt<=0 || dt>.5f) {
            next.temperature=raw_temperature; previous_filtered=raw_temperature; filtering=1;
            filtered_rate=0;
        } else {
            next.temperature+=dt/(.3f+dt)*(raw_temperature-next.temperature);
            float rate=(next.temperature-previous_filtered)/dt; previous_filtered=next.temperature;
            filtered_rate+=dt/(.2f+dt)*(rate-filtered_rate);
        }
    }
    uint8_t enabled=(uint8_t)s->value[UAV_SETTING_HEATER_ENABLED];
    if (!enabled) { fault=0; integral=0; stable_active=rise_active=0; next.state=UAV_HEATER_OFF; }
    else {
        if (!have_sample) next.state=UAV_HEATER_WAIT;
        else {
            if (!valid_sample || (uint32_t)(now-sample_ms)>500u) fault=UAV_HEATER_SENSOR;
            if (valid_sample && raw_temperature>=60) fault=UAV_HEATER_OVERHEAT;
            /* A watchdog trip remains latched until the heater is disabled. */
            if (previous_fault) fault=previous_fault;
            if (fault) { next.state=UAV_HEATER_FAULT; integral=0; stable_active=rise_active=0; }
            else if (!(s->value[UAV_SETTING_HEATER_KP] | s->value[UAV_SETTING_HEATER_KI] | s->value[UAV_SETTING_HEATER_KD])) {
                integral=0; stable_active=rise_active=0; next.state=UAV_HEATER_MONITOR;
            }
            else if (dt<=0 || dt>.5f) { integral=0; stable_active=rise_active=0; next.state=UAV_HEATER_WAIT; filtering=0; }
            else {
                float error=next.target-next.temperature, limit=s->value[UAV_SETTING_HEATER_LIMIT];
                next.p=s->value[UAV_SETTING_HEATER_KP]*.01f*error;
                next.d=-s->value[UAV_SETTING_HEATER_KD]*.01f*filtered_rate;
                float proposed=integral+s->value[UAV_SETTING_HEATER_KI]*.001f*error*dt;
                float proposed_out=next.p+proposed+next.d;
                if ((proposed_out>=0 && proposed_out<=limit) || (proposed_out>limit && error<0) || (proposed_out<0 && error>0))
                    integral=clamp(proposed,0,limit);
                next.duty_percent=clamp(next.p+integral+next.d,0,limit);
                if (raw_temperature>=next.target+2) { next.duty_percent=0; integral=0; }
                if (fabsf(error)<=.5f && fabsf(filtered_rate)<=.1f) {
                    if (!stable_active) { stable_active=1; stable_ms=now; }
                } else { stable_active=0; }
                next.state=stable_active && (uint32_t)(now-stable_ms)>=5000u ? UAV_HEATER_READY:UAV_HEATER_WARMING;
                if (was_ready && fabsf(error)<=1 && fabsf(filtered_rate)<=.2f) next.state=UAV_HEATER_READY;
                if (next.state==UAV_HEATER_READY) warm_ms=now;
                if ((uint32_t)(now-warm_ms)>600000u) fault=UAV_HEATER_TIMEOUT;
                if (error>2 && next.duty_percent>=20) {
                    if (!rise_active) { rise_active=1; rise_ms=now; rise_start_temp=next.temperature; }
                    if ((uint32_t)(now-rise_ms)>=120000u) {
                        if (next.temperature-rise_start_temp<1) fault=UAV_HEATER_NO_RISE;
                        rise_ms=now; rise_start_temp=next.temperature;
                    }
                } else rise_active=0;
                if (fault) { next.state=UAV_HEATER_FAULT; next.duty_percent=0; integral=0; }
            }
        }
    }
    next.i=integral; next.fault=fault; next.sample_ms=have_sample ? sample_ms:0;
    /* Recheck the latest profile under the same critical section as the output;
     * a concurrent disable/limit reduction must not be overwritten by old settings. */
    taskENTER_CRITICAL(); uav_settings_snapshot(&snapshot);
    if (stats.state==UAV_HEATER_FAULT && stats.fault && enabled) {
        fault=stats.fault; next.fault=fault; next.state=UAV_HEATER_FAULT; next.duty_percent=0;
    }
    if (!snapshot.values.value[UAV_SETTING_HEATER_ENABLED]) { next.duty_percent=0; next.state=UAV_HEATER_OFF; }
    if (!(snapshot.values.value[UAV_SETTING_HEATER_KP] | snapshot.values.value[UAV_SETTING_HEATER_KI] |
          snapshot.values.value[UAV_SETTING_HEATER_KD])) {
        integral=0; next.duty_percent=next.p=next.i=next.d=0;
        if (next.state!=UAV_HEATER_OFF && next.state!=UAV_HEATER_FAULT && next.temperature_valid)
            next.state=UAV_HEATER_MONITOR;
    }
    if (uav_storage_busy() && next.state!=UAV_HEATER_OFF && next.state!=UAV_HEATER_FAULT) {
        next.duty_percent=0; next.state=UAV_HEATER_WAIT;
    }
    next.duty_percent=clamp(next.duty_percent,0,snapshot.values.value[UAV_SETTING_HEATER_LIMIT]);
    uav_device_heater_write((uint16_t)(next.duty_percent*10)); stats=next; taskEXIT_CRITICAL();
    if ((uint32_t)(now-report_ms)>=1000u) {
        report_ms=now;
        uav_logf(next.fault ? "WARN":"INFO","HEATER","state=%u fault=%u valid=%u temp_centi_C=%ld target_deci_C=%u duty_permille=%u P_centi=%ld I_centi=%ld D_centi=%ld sample_age_ms=%lu",
            next.state,next.fault,next.temperature_valid,(long)(next.temperature*100),previous.value[UAV_SETTING_HEATER_TARGET],
            (unsigned)(next.duty_percent*10),(long)(next.p*100),(long)(next.i*100),(long)(next.d*100),
            (unsigned long)(have_sample ? now-sample_ms:UINT32_MAX));
        uav_temperature_io_t temperature; uav_sensor_temperature_io(&temperature);
        uav_logf(temperature.valid ? "INFO":"WARN","TEMP_IO",
            "ID=0x%02x MSB=0x%02x LSB=0x%02x raw11=%d raw_milli_C=%ld valid=%u HAL=%u reads=%lu errors=%lu",
            temperature.id,temperature.msb,temperature.lsb,temperature.raw_signed,
            (long)(temperature.raw_signed*125+23000),temperature.valid,temperature.hal_status,
            (unsigned long)temperature.reads,(unsigned long)temperature.errors);
        if (temperature.check_read) uav_logf(temperature.check_valid ? "INFO":"WARN","TEMP_CHECK",
            "burst_raw11=%d single_raw11=%d single_MSB=0x%02x single_LSB=0x%02x delta_raw=%d valid=%u HAL=%u at_read=%lu",
            temperature.check_burst_raw_signed,temperature.check_raw_signed,temperature.check_msb,temperature.check_lsb,
            temperature.check_burst_raw_signed-temperature.check_raw_signed,temperature.check_valid,temperature.check_hal,
            (unsigned long)temperature.check_read);
        uav_heater_io_t output; uav_device_heater_io(&output);
        uav_logf("INFO","HEATER_IO","CCR=%u ARR=%u PSC=%u CR1=0x%lx CCMR1=0x%lx CCER=0x%lx PB8=%u mode=%u AF=%u",
            output.ccr,output.arr,output.psc,(unsigned long)output.cr1,(unsigned long)output.ccmr1,
            (unsigned long)output.ccer,output.pb8_high,output.gpio_mode,output.gpio_af);
    }
}
