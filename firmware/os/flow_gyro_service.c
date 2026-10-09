#include "flow_gyro_service.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
static uav_flow_gyro_history_t history, reader_copy;
void uav_flow_gyro_publish(uint32_t us, const float rate[3], uint8_t valid) {
    taskENTER_CRITICAL();
    if (valid) uav_flow_gyro_history_push(&history,us,rate);
    else { history.count=0; history.generation++; }
    taskEXIT_CRITICAL();
}
void uav_flow_gyro_read(uint32_t end, uint32_t span, uav_flow_gyro_interval_t *out) {
    taskENTER_CRITICAL(); reader_copy=history; taskEXIT_CRITICAL();
    uav_flow_gyro_history_integrate(&reader_copy,end,span,out);
    taskENTER_CRITICAL();
    if (out && history.generation!=reader_copy.generation) {
        out->valid=0; out->reason=UAV_FLOW_GYRO_GAP; memset(out->delta,0,sizeof(out->delta));
    }
    taskEXIT_CRITICAL();
}
