#include "flight_snapshot.h"
#include "FreeRTOS.h"
#include "task.h"
#include "platform_time.h"
#include <math.h>
static flight_snapshot_t snapshot;
void flight_attitude_publish(float roll, float pitch, float yaw, float roll_rate, float pitch_rate,
                             float yaw_rate) {
    taskENTER_CRITICAL();
    snapshot.roll_rad = roll * 0.01745329252f;
    snapshot.pitch_rad = pitch * 0.01745329252f;
    snapshot.yaw_rad = yaw * 0.01745329252f;
    snapshot.roll_rate_radps = roll_rate * 0.01745329252f;
    snapshot.pitch_rate_radps = pitch_rate * 0.01745329252f;
    snapshot.yaw_rate_radps = yaw_rate * 0.01745329252f;
    snapshot.attitude_ms = platform_millis();
    snapshot.valid = isfinite(roll) && isfinite(pitch) && isfinite(yaw) && isfinite(roll_rate) &&
                     isfinite(pitch_rate) && isfinite(yaw_rate);
    taskEXIT_CRITICAL();
}
void flight_state_publish(uint8_t state) {
    taskENTER_CRITICAL();
    snapshot.state = state;
    taskEXIT_CRITICAL();
}
void flight_snapshot_read(flight_snapshot_t *s) {
    taskENTER_CRITICAL();
    *s = snapshot;
    taskEXIT_CRITICAL();
}

void flight_attitude_invalidate(void) {
    taskENTER_CRITICAL();
    snapshot.valid = 0;
    taskEXIT_CRITICAL();
}

void flight_fault_publish(uint8_t reason, uint32_t transitions) {
    taskENTER_CRITICAL();
    snapshot.fault = reason;
    snapshot.transitions = transitions;
    taskEXIT_CRITICAL();
}
