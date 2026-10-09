#include "flow_proc.h"
#include "flow_stream.h"
#include "flow_config.h"
#include "flow_rx_port.h"
#include "flight_snapshot.h"
#include "platform_time.h"
#include "log_service.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include <string.h>
#include <math.h>
extern uint8_t uart2RX[200];
typedef struct { uint32_t received_ms; uint16_t length; uint8_t bytes[64]; } flow_chunk_t;
static QueueHandle_t chunks;
static _flow_data state;
static uav_flow_stream_t stream;
static uav_flow_processor_t processor;
static uav_flow_frame_t latest_frame;
static volatile uint32_t rx_chunks,rx_bytes,drops,drop_generation;
static uint32_t processed_generation;
osThreadId FlowTaskHandle;
int flow_transport_init(void) { chunks=xQueueCreate(4,sizeof(flow_chunk_t)); return chunks ? 0:-1; }
void Flow_Data_Proc(uint16_t size) {
    if (!size || size>sizeof(uart2RX) || !chunks) return;
    uint32_t ms=platform_millis(); BaseType_t wake=pdFALSE;
    rx_bytes+=size;
    for (uint16_t offset=0;offset<size;) {
        flow_chunk_t chunk={.received_ms=ms};
        chunk.length=(uint16_t)(size-offset); if (chunk.length>sizeof(chunk.bytes)) chunk.length=sizeof(chunk.bytes);
        memcpy(chunk.bytes,uart2RX+offset,chunk.length); rx_chunks++;
        if (xQueueSendFromISR(chunks,&chunk,&wake)!=pdPASS) { drops++; drop_generation++; }
        offset=(uint16_t)(offset+chunk.length);
    }
    portYIELD_FROM_ISR(wake);
}
void flow_uart_error_isr(uint32_t error) {
    uav_flow_rx_error_isr(error); drop_generation++;
    state.flowFlag=state.range_valid=state.height_valid=0;
}
void flow_snapshot(_flow_data *out) {
    if (!out) return;
    taskENTER_CRITICAL(); *out=state; taskEXIT_CRITICAL();
    uint32_t now=platform_millis();
    if ((uint32_t)(now-out->flow_ms)>uav_board_flow_config.timeout_ms) {
        out->flowFlag=0; out->xFlowVel=out->yFlowVel=0;
    }
    if ((uint32_t)(now-out->range_ms)>300u) out->range_valid=out->height_valid=0;
    if ((uint32_t)(now-out->received_ms)>uav_board_flow_config.timeout_ms) { out->raw_fresh=0; out->reason=UAV_FLOW_STALE; }
    if (!out->height_valid) out->zFlowVel=0;
}
static void received(void *context, const uav_flow_frame_t *frame) {
    (void)context; latest_frame=*frame;
    flight_snapshot_t flight; flight_snapshot_read(&flight);
    uav_flow_attitude_t pose={flight.attitude_ms,flight.roll_rad,flight.pitch_rad,
        flight.roll_rate_radps,flight.pitch_rate_radps,flight.valid};
    uav_flow_processor_update(&processor,&uav_board_flow_config,frame,&pose,platform_millis());
}
static int16_t offset_mm(int16_t angle, uint16_t range) {
    float value=angle*.0001f*range;
    if (value>32767) value=32767;
    if (value< -32768) value=-32768;
    return (int16_t)value;
}
static void publish(void) {
    const uav_flow_measurement_t *o=&processor.output;
    _flow_data next={0};
    next.raw_integral_x=latest_frame.integral_x; next.raw_integral_y=latest_frame.integral_y;
    next.raw_range_mm=latest_frame.range_mm; next.delatTime=latest_frame.integration_us;
    next.xNowOffset=offset_mm(latest_frame.integral_x,latest_frame.range_mm);
    next.yNowOffset=offset_mm(latest_frame.integral_y,latest_frame.range_mm);
    next.zNowHeight=(uint16_t)(o->height_valid ? o->height_m*1000:latest_frame.range_mm);
    next.xLastOffset=state.xNowOffset; next.yLastOffset=state.yNowOffset; next.zLastHeight=state.zNowHeight;
    next.xFlowVel=o->flow_valid ? o->velocity_mps[0]*1000:0; next.yFlowVel=o->flow_valid ? o->velocity_mps[1]*1000:0;
    next.zFlowVel=o->height_valid ? o->vertical_velocity_mps*1000:0;
    next.flowFlag=o->flow_valid; next.flowConf=o->flow_quality; next.range_quality=o->range_quality;
    next.range_valid=o->range_valid; next.height_valid=o->height_valid; next.reason=o->reason;
    next.range_m=o->range_m; next.height_m=o->height_m; next.velocity_variance=o->velocity_variance;
    next.received_ms=latest_frame.received_ms; next.flow_ms=o->flow_ms; next.range_ms=o->range_ms;
    next.frames=stream.frames; next.checksum_errors=stream.rejected; next.drops=drops;
    next.raw_fresh=(uint8_t)(o->frames && (uint32_t)(platform_millis()-latest_frame.received_ms)<=uav_board_flow_config.timeout_ms);
    taskENTER_CRITICAL();
    if (!uav_flow_rx_running() || processed_generation!=drop_generation) next.flowFlag=next.range_valid=next.height_valid=0;
    state=next; taskEXIT_CRITICAL();
}
void Flow_Task_Proc(void const *argument) {
    (void)argument; flow_chunk_t chunk;
    uint32_t last_report=platform_millis(); processed_generation=drop_generation;
    uav_flow_processor_init(&processor);
    uav_logf("INFO","FLOW","profile=UPIX14 inferred_977d5b0=1 axes=SENSOR gyro_comp=UNCONFIRMED Qmin=40 range_Qmin=50 range=50..8000mm tilt=45deg preview_only=1");
    for (;;) {
        uint32_t now=platform_millis();
        int recovered=uav_flow_rx_service(now);
        if (recovered || processed_generation!=drop_generation) {
            processed_generation=drop_generation; stream.used=0;
            memset(&latest_frame,0,sizeof(latest_frame));
            uav_flow_processor_init(&processor); xQueueReset(chunks);
        }
        if (xQueueReceive(chunks,&chunk,pdMS_TO_TICKS(10))==pdPASS)
            uav_flow_stream_feed(&stream,chunk.bytes,chunk.length,chunk.received_ms,received,NULL);
        now=platform_millis(); uav_flow_processor_expire(&processor,&uav_board_flow_config,now); publish();
        if ((uint32_t)(now-last_report)>=5000u) {
            last_report=now; uav_flow_rx_stats_t uart; uav_flow_rx_stats_read(&uart);
            uav_logf("INFO","FLOW","chunks=%lu bytes=%lu frames=%lu bad=%lu drop=%lu RX=%u errors=%lu recovered=%lu",
                (unsigned long)rx_chunks,(unsigned long)rx_bytes,(unsigned long)stream.frames,(unsigned long)stream.rejected,
                (unsigned long)drops,uart.running,(unsigned long)uart.errors,(unsigned long)uart.recovered);
            uav_logf("INFO","FLOW_MEAS","raw=%d,%d dt_us=%u range_mm=%u flowQ=%u rangeQ=%u flow_ok=%u range_ok=%u height_ok=%u reason=%u",
                latest_frame.integral_x,latest_frame.integral_y,latest_frame.integration_us,
                latest_frame.range_mm,processor.output.flow_quality,processor.output.range_quality,
                processor.output.flow_valid,processor.output.range_valid,processor.output.height_valid,processor.output.reason);
            uav_logf("INFO","FLOW_EST","height_mm=%ld sensor_v_mmps=%ld,%ld vz_mmps=%ld accepted=%lu/%lu range_reject=%lu",
                (long)(processor.output.height_m*1000),(long)(processor.output.velocity_mps[0]*1000),
                (long)(processor.output.velocity_mps[1]*1000),(long)(processor.output.vertical_velocity_mps*1000),
                (unsigned long)processor.output.accepted_flow,(unsigned long)processor.output.accepted_range,
                (unsigned long)processor.output.range_rejected);
        }
    }
}
