#ifndef UAV_FLOW_SERVICE_H
#define UAV_FLOW_SERVICE_H
#include <stdint.h>
typedef struct {
    int16_t xNowOffset, xLastOffset, yNowOffset, yLastOffset;
    uint16_t zNowHeight, zLastHeight;
    uint16_t delatTime;
    float xFlowVel, yFlowVel, zFlowVel;
    uint8_t flowFlag, flowConf;
    uint16_t raw_range_mm;
    int16_t raw_integral_x, raw_integral_y;
    uint8_t range_valid, height_valid, range_quality, reason, raw_fresh;
    uint32_t received_ms, flow_ms, range_ms, frames, checksum_errors, drops;
    float range_m, height_m, velocity_variance;
} _flow_data;
/* Init before task creation; ISR copies bounded chunks, never waits.
 * Queue-full drops a sample; only Flow task writes state. Snapshot is task-only. */
int flow_transport_init(void);
void Flow_Task_Proc(void const *argument);
void Flow_Data_Proc(uint16_t size);
void flow_snapshot(_flow_data *out);
void flow_uart_error_isr(uint32_t error);
#endif
