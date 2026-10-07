#include "pc_proc.h"
#include "platform_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "serial_port.h"
#include "star_dispatch.h"
#include <string.h>
#include "flight_snapshot.h"
#include "log_service.h"
#include "uav_events.h"
#include "boot_log.h"
#include "flash_proc.h"
osThreadId PCTaskHandle;
typedef struct {
    uint16_t length;
    uint8_t data[100];
} rx_packet_t;
static QueueHandle_t receive_queue;
static volatile uint8_t receive_lost;
static star_parser_t parser;
static star_dispatch_t dispatcher;
static uint16_t event_sequence;
static uint16_t log_sequence;
extern uint8_t uart1RX[100];
static uint8_t business(void *context, const star_frame_t *q, star_frame_t *r) {
    (void)context;

    flight_snapshot_t s;

    if (q->command == STAR_CMD_UAV_DIAGNOSTICS) {
        if (q->length)
            return STAR_BAD_LENGTH;
        uint32_t rejected, sequence[3];
        flight_snapshot_read(&s);
        uav_storage_diagnostics(&rejected, sequence);
        r->payload[1] = s.fault;
        star_write_u32(r->payload + 2, s.transitions);
        star_write_u32(r->payload + 6, rejected);
        for (unsigned i = 0; i < 3; i++)
            star_write_u32(r->payload + 10 + i * 4, sequence[i]);
        r->payload[22] = uav_storage_busy();
        r->length = 23;
        return STAR_OK;
    }
    if (q->command != STAR_CMD_STATUS && q->command != STAR_CMD_UAV_ATTITUDE)
        return STAR_UNSUPPORTED;
    if (q->length)
        return STAR_BAD_LENGTH;
    flight_snapshot_read(&s);
    r->payload[1] = s.state;
    star_write_f32(r->payload + 2, s.roll_rad);
    star_write_f32(r->payload + 6, s.pitch_rad);
    star_write_f32(r->payload + 10, s.yaw_rad);
    r->length = 14;
    return STAR_OK;
}
int PC_Init(void) {
    receive_queue = xQueueCreate(4, sizeof(rx_packet_t));
    star_parser_init(&parser);
    dispatcher.device_id = 0x58550001u;
    dispatcher.capabilities = STAR_CAP_STATUS | STAR_CAP_TELEMETRY_PERIOD | STAR_CAP_DIAGNOSTICS;
    dispatcher.telemetry_period_ms = 50;
    dispatcher.minimum_period_ms = 20;
    dispatcher.business = business;
    dispatcher.context = NULL;
    return receive_queue ? 0 : -1;
}
/* UART RX ISR: copy bounded bytes before DMA reuses uart1RX. No decoding,
 * printf, transmission, or control writes in this interrupt. Priority must
 * be numerically >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY. */
void PC_Data_Rx_Proc(uint16_t size) {
    rx_packet_t packet;
    BaseType_t woken = pdFALSE;
    if (!receive_queue || !size || size > 100)
        return;
    packet.length = size;
    memcpy(packet.data, uart1RX, size);
    if (xQueueSendFromISR(receive_queue, &packet, &woken) != pdPASS)
        receive_lost = 1;
    portYIELD_FROM_ISR(woken);
}
static int send_frame(const star_frame_t *frame) {
    uint8_t bytes[STAR_FRAME_MAX];
    size_t length = star_encode(frame, bytes, sizeof(bytes));
    /* Only this task owns USART1 TX; the port copies the frame for DMA. */
    return length ? serial_port_send(bytes, length) : -1;
}
static void on_frame(void *context, const star_frame_t *request) {
    star_frame_t response;
    (void)context;
    if (request->flags != STAR_REQUEST)
        return;
    star_dispatch(&dispatcher, request, &response);
    send_frame(&response);
}
void PC_Task_Proc(void const *argument) {
    rx_packet_t packet;
    star_frame_t request = {0}, event;
    uint32_t last_telemetry = platform_millis(), now;
    uint32_t started_ms = last_telemetry;
    uint8_t health_reported = 0;
    uint8_t pending_log[UAV_LOG_FRAME_BYTES];
    size_t pending_log_length = 0;
    (void)argument;
    if (!receive_queue) {
        vTaskDelete(NULL);
        return;
    }
    app_boot_log_running(dispatcher.telemetry_period_ms);
    for (;;) {
        if (receive_lost) {
            taskENTER_CRITICAL();
            xQueueReset(receive_queue);
            receive_lost = 0;
            taskEXIT_CRITICAL();
            star_parser_init(&parser);
        }
        if (xQueueReceive(receive_queue, &packet, pdMS_TO_TICKS(5)) == pdPASS)
            star_parser_feed(&parser, packet.data, packet.length, platform_millis(), on_frame,
                             NULL);
        now = platform_millis();
        if ((health_reported == 0 && (uint32_t)(now - started_ms) >= 2000u) ||
            (health_reported == 1 && (uint32_t)(now - started_ms) >= 8000u)) {
            app_boot_log_health();
            health_reported++;
        }
        star_parser_expire(&parser, now);
        if ((uint32_t)(now - last_telemetry) >= dispatcher.telemetry_period_ms) {
            request.command = STAR_CMD_STATUS;
            request.flags = STAR_REQUEST;
            request.sequence = event_sequence++;
            star_dispatch(&dispatcher, &request, &event);
            event.flags = STAR_EVENT;
            send_frame(&event);
            last_telemetry = now;
        }
        /* One bounded log chunk per loop; responses and telemetry go first.
         * This task owns all USART1 TX, so frame bytes never interleave. */
        if (!pending_log_length)
            pending_log_length = uav_log_receive(pending_log, sizeof(pending_log), 0);
        if (pending_log_length) {
            event.flags = STAR_EVENT;
            event.sequence = log_sequence;
            event.command = UAV_EVENT_LOG;
            event.length = (uint16_t)pending_log_length;
            memcpy(event.payload, pending_log, pending_log_length);
            if (send_frame(&event) == 0) {
                log_sequence++;
                pending_log_length = 0;
            }
        }
    }
}
