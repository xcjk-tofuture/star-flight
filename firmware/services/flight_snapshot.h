#ifndef FLIGHT_SNAPSHOT_H
#define FLIGHT_SNAPSHOT_H
#include <stdint.h>
typedef struct {
    float roll_rad, pitch_rad, yaw_rad;
    float roll_rate_radps, pitch_rate_radps, yaw_rate_radps;
    uint32_t attitude_ms;
    uint8_t valid, state, fault;
    uint32_t transitions;
} flight_snapshot_t;
/* Sensor task publishes attitude; control task publishes state. Other tasks
 * copy a coherent snapshot. Nonblocking, task context only. */
void flight_attitude_publish(float roll_deg, float pitch_deg, float yaw_deg, float roll_dps,
                             float pitch_dps, float yaw_dps);
void flight_state_publish(uint8_t state);
void flight_fault_publish(uint8_t reason, uint32_t transitions);
void flight_snapshot_read(flight_snapshot_t *snapshot);
void flight_attitude_invalidate(void);
void flight_attitude_publish_sample(float roll, float pitch, float yaw, float roll_rate,
                                   float pitch_rate, float yaw_rate, uint32_t sample_ms);
#endif
