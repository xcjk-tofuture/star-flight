#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "sbus_proc.h"
#include "AHRS.h"
#include "sbus_proc.h"
#include "queue.h"
#include "platform_time.h"
#include "flight_snapshot.h"
#include "log_service.h"
#include "flash_proc.h"
#include "sbus_stream.h"
#include "sbus_rx_port.h"
#include <string.h>

extern void UAV_Read_Param_Remote(_sbus_ch_struct *channel_data);
extern int UAV_Write_Param_Remote(_sbus_ch_struct channe_data);
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart3;
extern u8 SbusRxBuf[100];
static u16 SbusChannels[16];
static QueueHandle_t sbus_frames;
static uint32_t last_frame_ms;
static uint8_t frame_seen;
static uint32_t valid_frame_ms, frame_lost_count, failsafe_count;
static volatile uint32_t rx_chunks, rx_bytes, queue_drops;
static uint8_t flags_seen, receiver_seen;
static uav_sbus_stream_t stream;
typedef struct { uint32_t received_ms; uint16_t length; uint8_t bytes[100]; } sbus_chunk_t;
static uint8_t decode_buffer[25];
int sbus_transport_init(void) {
    sbus_frames = xQueueCreate(4, sizeof(sbus_chunk_t));
    return sbus_frames ? 0 : -1;
}

osThreadId SbusUart6TaskHandle;

_sbus_ch_cal_struct CAL_SBUS_CH;
void sbus_snapshot(_sbus_ch_cal_struct *out) {
    taskENTER_CRITICAL();
    *out = CAL_SBUS_CH;
    taskEXIT_CRITICAL();
}
_sbus_ch_struct SBUS_CH;
static _sbus_ch_struct published_raw;
void sbus_uart_error_isr(uint32_t error) {
    uav_sbus_rx_error_isr(error);
    SBUS_CH.Connect_State=0; published_raw.Connect_State=0; CAL_SBUS_CH.Connect_State=0;
}
void sbus_raw_snapshot(_sbus_ch_struct *out) {
    taskENTER_CRITICAL();
    *out = published_raw;
    taskEXIT_CRITICAL();
}

static int remote_parameters_valid(void) {
#define RANGE_OK(i)                                                                                \
    (SBUS_CH.CH##i##_MAX <= 2047 && SBUS_CH.CH##i##_MAX > SBUS_CH.CH##i##_MIN &&                   \
     SBUS_CH.CH##i##_MAX - SBUS_CH.CH##i##_MIN >= 100)
    return RANGE_OK(1) && RANGE_OK(2) && RANGE_OK(3) && RANGE_OK(4) && RANGE_OK(5) && RANGE_OK(6) &&
           RANGE_OK(7) && RANGE_OK(8);
#undef RANGE_OK
}
static u8 remoteCaliFlag = 0;

static u8 remoteCaliSaveFlashFlag = 0;
static uint8_t calibration_requests;
static uint32_t remote_save_ticket;
static _sbus_ch_struct original_calibration;
void sbus_cancel_calibration(void) {
    taskENTER_CRITICAL(); calibration_requests|=4u; taskEXIT_CRITICAL();
}
uint8_t sbus_calibration_saving(void) { return remote_save_ticket!=0; }
static uint8_t invalid_ranges(void) {
    uint8_t mask=0;
#define CHECK_RANGE(i) \
    if (!(SBUS_CH.CH##i##_MAX<=2047 && SBUS_CH.CH##i##_MAX>SBUS_CH.CH##i##_MIN && \
          SBUS_CH.CH##i##_MAX-SBUS_CH.CH##i##_MIN>=100)) mask|=(uint8_t)(1u<<((i)-1))
    CHECK_RANGE(1); CHECK_RANGE(2); CHECK_RANGE(3); CHECK_RANGE(4);
    CHECK_RANGE(5); CHECK_RANGE(6); CHECK_RANGE(7); CHECK_RANGE(8);
#undef CHECK_RANGE
    return mask;
}
void sbus_diagnostics_read(sbus_diagnostics_t *out) {
    uint32_t now=platform_millis();
    taskENTER_CRITICAL();
    *out=(sbus_diagnostics_t){.chunks=rx_chunks,.bytes=rx_bytes,.queue_drops=queue_drops,
        .frames=stream.frames,.bad_frames=stream.rejected,.skipped_bytes=stream.discarded,
        .frame_lost=frame_lost_count,.failsafe=failsafe_count,
        .frame_age_ms=receiver_seen ? (uint32_t)(now-valid_frame_ms):UINT32_MAX,
        .good_age_ms=frame_seen ? (uint32_t)(now-last_frame_ms):UINT32_MAX,
        .receiver_present=(uint8_t)(receiver_seen && (uint32_t)(now-valid_frame_ms)<=100u),
        .radio_ok=SBUS_CH.Connect_State,.calibrated=(uint8_t)remote_parameters_valid(),
        .invalid_ranges=invalid_ranges(),.flags=flags_seen,.uart_error=huart6.ErrorCode,
        .rx_running=uav_sbus_rx_running()};
    taskEXIT_CRITICAL();
}
static void received_frame(void *ctx, const uint8_t *frame, uint32_t ms) {
    (void)ctx; receiver_seen=1; valid_frame_ms=ms; flags_seen=frame[23];
    if (flags_seen&8u) { failsafe_count++; return; }
    if (flags_seen&4u) { frame_lost_count++; return; }
    memcpy(decode_buffer,frame,sizeof(decode_buffer)); Sbus_Channels_Proc();
    last_frame_ms=ms; frame_seen=1;
}
void sbus_request_calibration(uint8_t save) {
    taskENTER_CRITICAL();
    calibration_requests |= save ? 2u : 1u;
    taskEXIT_CRITICAL();
}
uint8_t sbus_calibration_active(void) { return remoteCaliFlag; }

void Sbus_Uart6_Task_Proc(void const *argument) {
    (void)argument;
    Channel_Param_Init();
    uint32_t report_ms=platform_millis();
    sbus_chunk_t chunk;
    for (;;) {
        int restarted=uav_sbus_rx_service(platform_millis());
        if (restarted) {
            stream.used=0; receiver_seen=frame_seen=0;
            SBUS_CH.Connect_State=0;
            taskENTER_CRITICAL(); published_raw.Connect_State=0; CAL_SBUS_CH.Connect_State=0; taskEXIT_CRITICAL();
            xQueueReset(sbus_frames);
            if (restarted>0) uav_logf("INFO","SBUS_RX","DMA restarted; awaiting fresh frame");
        }
        uint8_t request;
        taskENTER_CRITICAL();
        request = calibration_requests;
        calibration_requests = 0;
        taskEXIT_CRITICAL();
        flight_snapshot_t flight;
        flight_snapshot_read(&flight);
        if ((request&3u) && flight.state != 0) {
            uav_logf("WARN", "SBUS", "calibration rejected: flight_state=%u", (unsigned)flight.state);
            request &= 4u;
        }
        if ((request&1u) && (sensor_imu_calibrating() || sensors_mag_calibration_active())) {
            uav_logf("WARN","SBUS","start rejected: sensor calibration active"); request=0;
        }
        if ((request & 1u) && !remote_save_ticket) {
            original_calibration=SBUS_CH;
            remoteCaliFlag = 1;
            remoteCaliSaveFlashFlag = 0;
#define RESET_RANGE(i)                                                                             \
    SBUS_CH.CH##i##_MIN = 2047;                                                                    \
    SBUS_CH.CH##i##_MAX = 0
            RESET_RANGE(1);
            RESET_RANGE(2);
            RESET_RANGE(3);
            RESET_RANGE(4);
            RESET_RANGE(5);
            RESET_RANGE(6);
            RESET_RANGE(7);
            RESET_RANGE(8);
#undef RESET_RANGE
        }
        if (request & 2u)
            remoteCaliSaveFlashFlag = 1;
        if ((request&4u) && remoteCaliFlag && !remote_save_ticket) {
#define RESTORE_RANGE(i) SBUS_CH.CH##i##_MIN=original_calibration.CH##i##_MIN; SBUS_CH.CH##i##_MAX=original_calibration.CH##i##_MAX
            RESTORE_RANGE(1); RESTORE_RANGE(2); RESTORE_RANGE(3); RESTORE_RANGE(4);
            RESTORE_RANGE(5); RESTORE_RANGE(6); RESTORE_RANGE(7); RESTORE_RANGE(8);
#undef RESTORE_RANGE
            remoteCaliFlag=0; remoteCaliSaveFlashFlag=0;
            uav_logf("INFO","RC_CAL","cancelled; previous ranges restored");
        }
        if (remote_save_ticket) {
            int result=uav_storage_result(remote_save_ticket);
            if (result) {
                remote_save_ticket=0; remoteCaliSaveFlashFlag=0;
                if (result>0) remoteCaliFlag=0;
                uav_logf(result>0 ? "INFO":"ERROR","RC_CAL","save verified=%u result=%d retry_on_failure=1",
                         (unsigned)(result>0),result);
            }
        }
        if (xQueueReceive(sbus_frames,&chunk,pdMS_TO_TICKS(20))==pdPASS)
            uav_sbus_stream_feed(&stream,chunk.bytes,chunk.length,chunk.received_ms,received_frame,NULL);
        SBUS_CH.Connect_State=(uint8_t)(frame_seen && !uav_sbus_rx_pending() && uav_sbus_rx_running() &&
            !(flags_seen&8u) && (uint32_t)(platform_millis()-last_frame_ms)<=100u);
        if (SBUS_CH.Connect_State && !remoteCaliFlag && remote_parameters_valid()) {
            _sbus_ch_cal_struct next = {0};
            next.CAL_CH1 =
                (uint16_t)Sbus_To_Range(SBUS_CH.CH1, 1000, 2000, SBUS_CH.CH1_MIN, SBUS_CH.CH1_MAX);
            next.CAL_CH2 =
                (uint16_t)Sbus_To_Range(SBUS_CH.CH2, 1000, 2000, SBUS_CH.CH2_MIN, SBUS_CH.CH2_MAX);
            next.CAL_CH3 =
                (uint16_t)Sbus_To_Range(SBUS_CH.CH3, 1000, 2000, SBUS_CH.CH3_MIN, SBUS_CH.CH3_MAX);
            next.CAL_CH4 =
                (uint16_t)Sbus_To_Range(SBUS_CH.CH4, 1000, 2000, SBUS_CH.CH4_MIN, SBUS_CH.CH4_MAX);
            next.CAL_CH5 =
                (uint16_t)Sbus_To_Range(SBUS_CH.CH5, 1000, 2000, SBUS_CH.CH5_MIN, SBUS_CH.CH5_MAX);
            next.CAL_CH6 =
                (uint16_t)Sbus_To_Range(SBUS_CH.CH6, 1000, 2000, SBUS_CH.CH6_MIN, SBUS_CH.CH6_MAX);
            next.CAL_CH7 =
                (uint16_t)Sbus_To_Range(SBUS_CH.CH7, 1000, 2000, SBUS_CH.CH7_MIN, SBUS_CH.CH7_MAX);
            next.CAL_CH8 =
                (uint16_t)Sbus_To_Range(SBUS_CH.CH8, 1000, 2000, SBUS_CH.CH8_MIN, SBUS_CH.CH8_MAX);
            next.Connect_State = 1;
            taskENTER_CRITICAL();
            if (uav_sbus_rx_pending() || !uav_sbus_rx_running()) next.Connect_State=0;
            CAL_SBUS_CH = next;
            taskEXIT_CRITICAL();
        } else {
            taskENTER_CRITICAL();
            CAL_SBUS_CH.Connect_State = 0;
            taskEXIT_CRITICAL();
            if (remoteCaliFlag && SBUS_CH.Connect_State && !remote_save_ticket)
                Remote_Channel_Calibration();
        }
        taskENTER_CRITICAL();
        if (uav_sbus_rx_pending()) { SBUS_CH.Connect_State=0; CAL_SBUS_CH.Connect_State=0; }
        published_raw = SBUS_CH;
        taskEXIT_CRITICAL();
        if ((uint32_t)(platform_millis()-report_ms)>=5000u) {
            report_ms=platform_millis(); sbus_diagnostics_t d; sbus_diagnostics_read(&d);
            uav_logf("INFO","SBUS","chunks=%lu bytes=%lu frames=%lu bad=%lu drop=%lu lost=%lu fs=%lu flags=0x%02x RX=%u raw=%u cal=%u invalid=0x%02x age_ms=%lu uart_error=0x%lx",
                     (unsigned long)d.chunks,(unsigned long)d.bytes,(unsigned long)d.frames,(unsigned long)d.bad_frames,
                     (unsigned long)d.queue_drops,(unsigned long)d.frame_lost,(unsigned long)d.failsafe,d.flags,d.rx_running,
                     d.radio_ok,d.calibrated,d.invalid_ranges,(unsigned long)d.good_age_ms,(unsigned long)d.uart_error);
            uav_sbus_rx_stats_t rx; uav_sbus_rx_stats_read(&rx);
            uav_logf("INFO","SBUS_RX","errors=%lu PE=%lu NE=%lu FE=%lu ORE=%lu DMA=%lu attempts=%lu recovered=%lu failed=%lu last_error=0x%lx last_hal=%lu RX=%u pending=%u",
                     (unsigned long)rx.errors,(unsigned long)rx.parity,(unsigned long)rx.noise,(unsigned long)rx.framing,
                     (unsigned long)rx.overrun,(unsigned long)rx.dma_errors,(unsigned long)rx.attempts,
                     (unsigned long)rx.recovered,(unsigned long)rx.failed,(unsigned long)rx.last_error,
                     (unsigned long)rx.last_status,rx.running,rx.pending);
        }
    }
}

void Remote_Channel_Calibration() {
    SBUS_CH.CH1_MIN = SBUS_CH.CH1 < SBUS_CH.CH1_MIN ? SBUS_CH.CH1 : SBUS_CH.CH1_MIN;
    SBUS_CH.CH1_MAX = SBUS_CH.CH1 > SBUS_CH.CH1_MAX ? SBUS_CH.CH1 : SBUS_CH.CH1_MAX;

    SBUS_CH.CH2_MIN = SBUS_CH.CH2 < SBUS_CH.CH2_MIN ? SBUS_CH.CH2 : SBUS_CH.CH2_MIN;
    SBUS_CH.CH2_MAX = SBUS_CH.CH2 > SBUS_CH.CH2_MAX ? SBUS_CH.CH2 : SBUS_CH.CH2_MAX;

    SBUS_CH.CH3_MIN = SBUS_CH.CH3 < SBUS_CH.CH3_MIN ? SBUS_CH.CH3 : SBUS_CH.CH3_MIN;
    SBUS_CH.CH3_MAX = SBUS_CH.CH3 > SBUS_CH.CH3_MAX ? SBUS_CH.CH3 : SBUS_CH.CH3_MAX;

    SBUS_CH.CH4_MIN = SBUS_CH.CH4 < SBUS_CH.CH4_MIN ? SBUS_CH.CH4 : SBUS_CH.CH4_MIN;
    SBUS_CH.CH4_MAX = SBUS_CH.CH4 > SBUS_CH.CH4_MAX ? SBUS_CH.CH4 : SBUS_CH.CH4_MAX;

    SBUS_CH.CH5_MIN = SBUS_CH.CH5 < SBUS_CH.CH5_MIN ? SBUS_CH.CH5 : SBUS_CH.CH5_MIN;
    SBUS_CH.CH5_MAX = SBUS_CH.CH5 > SBUS_CH.CH5_MAX ? SBUS_CH.CH5 : SBUS_CH.CH5_MAX;

    SBUS_CH.CH6_MIN = SBUS_CH.CH6 < SBUS_CH.CH6_MIN ? SBUS_CH.CH6 : SBUS_CH.CH6_MIN;
    SBUS_CH.CH6_MAX = SBUS_CH.CH6 > SBUS_CH.CH6_MAX ? SBUS_CH.CH6 : SBUS_CH.CH6_MAX;

    SBUS_CH.CH7_MIN = SBUS_CH.CH7 < SBUS_CH.CH7_MIN ? SBUS_CH.CH7 : SBUS_CH.CH7_MIN;
    SBUS_CH.CH7_MAX = SBUS_CH.CH7 > SBUS_CH.CH7_MAX ? SBUS_CH.CH7 : SBUS_CH.CH7_MAX;

    SBUS_CH.CH8_MIN = SBUS_CH.CH8 < SBUS_CH.CH8_MIN ? SBUS_CH.CH8 : SBUS_CH.CH8_MIN;
    SBUS_CH.CH8_MAX = SBUS_CH.CH8 > SBUS_CH.CH8_MAX ? SBUS_CH.CH8 : SBUS_CH.CH8_MAX;

    if (remoteCaliSaveFlashFlag && !remote_save_ticket) {
        if (remote_parameters_valid() && UAV_Write_Param_Remote_Ticket(SBUS_CH, &remote_save_ticket) == 0) {
            remoteCaliSaveFlashFlag = 0;
        }
    }
}

void Channel_Param_Init() {

    UAV_Read_Param_Remote(&SBUS_CH);
    //  SBUS_CH.CH1_MIN = 353;
    //	SBUS_CH.CH1_MAX = 1697;
    //
    //	SBUS_CH.CH2_MIN = 353;
    //	SBUS_CH.CH2_MAX = 1697;
    //
    //	SBUS_CH.CH3_MIN = 353;
    //	SBUS_CH.CH3_MAX = 1697;
    //
    //
    //	SBUS_CH.CH4_MIN = 353;
    //	SBUS_CH.CH4_MAX = 1697;
    //
    //	SBUS_CH.CH5_MIN = 353;
    //	SBUS_CH.CH5_MAX = 1697;
    //
    //	SBUS_CH.CH6_MIN = 353;
    //	SBUS_CH.CH6_MAX = 1697;
    //
    //	SBUS_CH.CH7_MIN = 353;
    //	SBUS_CH.CH7_MAX = 1697;
    //
    //	SBUS_CH.CH8_MIN = 353;
    //	SBUS_CH.CH8_MAX = 1697;
}

void Sbus_Uart6_IDLE_Proc(uint16_t size) {
    BaseType_t wake = pdFALSE;
    rx_chunks++; rx_bytes+=size;
    if (size && size<=100 && sbus_frames) {
        sbus_chunk_t chunk={.received_ms=platform_millis(),.length=size};
        memcpy(chunk.bytes,SbusRxBuf,size);
        if (xQueueSendFromISR(sbus_frames,&chunk,&wake)!=pdPASS) queue_drops++;
    } else queue_drops++;
    portYIELD_FROM_ISR(wake);
}

void Sbus_Channels_Proc(void) // 解析sbus函数
{
    SbusChannels[0] = ((decode_buffer[1] | decode_buffer[2] << 8) & 0x07FF);
    SbusChannels[1] = ((decode_buffer[2] >> 3 | decode_buffer[3] << 5) & 0x07FF);
    SbusChannels[2] =
        ((decode_buffer[3] >> 6 | decode_buffer[4] << 2 | decode_buffer[5] << 10) & 0x07FF);
    SbusChannels[3] = ((decode_buffer[5] >> 1 | decode_buffer[6] << 7) & 0x07FF);
    SbusChannels[4] = ((decode_buffer[6] >> 4 | decode_buffer[7] << 4) & 0x07FF);
    SbusChannels[5] =
        ((decode_buffer[7] >> 7 | decode_buffer[8] << 1 | decode_buffer[9] << 9) & 0x07FF);
    SbusChannels[6] = ((decode_buffer[9] >> 2 | decode_buffer[10] << 6) & 0x07FF);
    SbusChannels[7] = ((decode_buffer[10] >> 5 | decode_buffer[11] << 3) & 0x07FF);
    SbusChannels[8] = ((decode_buffer[12] | decode_buffer[13] << 8) & 0x07FF);
    SbusChannels[9] = ((decode_buffer[13] >> 3 | decode_buffer[14] << 5) & 0x07FF);
    SbusChannels[10] =
        ((decode_buffer[14] >> 6 | decode_buffer[15] << 2 | decode_buffer[16] << 10) & 0x07FF);
    SbusChannels[11] = ((decode_buffer[16] >> 1 | decode_buffer[17] << 7) & 0x07FF);
    SbusChannels[12] = ((decode_buffer[17] >> 4 | decode_buffer[18] << 4) & 0x07FF);
    SbusChannels[13] =
        ((decode_buffer[18] >> 7 | decode_buffer[19] << 1 | decode_buffer[20] << 9) & 0x07FF);
    SbusChannels[14] = ((decode_buffer[20] >> 2 | decode_buffer[21] << 6) & 0x07FF);
    SbusChannels[15] = ((decode_buffer[21] >> 5 | decode_buffer[22] << 3) & 0x07FF);

    SBUS_CH.CH1 = SbusChannels[0];
    SBUS_CH.CH2 = SbusChannels[1];
    SBUS_CH.CH3 = SbusChannels[2];
    SBUS_CH.CH4 = SbusChannels[3];
    SBUS_CH.CH5 = SbusChannels[4];
    SBUS_CH.CH6 = SbusChannels[5];
    SBUS_CH.CH7 = SbusChannels[6];
    SBUS_CH.CH8 = SbusChannels[7];
    SBUS_CH.CH9 = SbusChannels[8];
    SBUS_CH.CH10 = SbusChannels[9];
    SBUS_CH.CH11 = SbusChannels[10];
    SBUS_CH.CH12 = SbusChannels[11];
    SBUS_CH.CH13 = SbusChannels[12];
    SBUS_CH.CH14 = SbusChannels[13];
    SBUS_CH.CH15 = SbusChannels[14];
    SBUS_CH.CH16 = SbusChannels[15];
}

float Sbus_To_Range(u16 sbus_value, float p_min, float p_max, u16 ch_min, u16 ch_max) {
    float p;
    if (ch_max <= ch_min)
        return p_min;
    p = p_min + (float)(sbus_value - ch_min) * (p_max - p_min) / (float)(ch_max - ch_min);
    if (p > p_max)
        p = p_max;
    if (p < p_min)
        p = p_min;
    return p;
}
