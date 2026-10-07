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
static uint8_t decode_buffer[25];
int sbus_transport_init(void) {
    sbus_frames = xQueueCreate(4, 25);
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
void sbus_request_calibration(uint8_t save) {
    taskENTER_CRITICAL();
    calibration_requests |= save ? 2u : 1u;
    taskEXIT_CRITICAL();
}
uint8_t sbus_calibration_active(void) { return remoteCaliFlag; }

void Sbus_Uart6_Task_Proc(void const *argument) {
    (void)argument;
    Channel_Param_Init();
    for (;;) {
        uint8_t request;
        taskENTER_CRITICAL();
        request = calibration_requests;
        calibration_requests = 0;
        taskEXIT_CRITICAL();
        flight_snapshot_t flight;
        flight_snapshot_read(&flight);
        if (request && flight.state != 0) {
            uav_logf("WARN", "SBUS", "calibration rejected: flight_state=%u", (unsigned)flight.state);
            request = 0;
        }
        if ((request & 1u) && !remote_save_ticket) {

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
        if (remote_save_ticket) {
            int result=uav_storage_result(remote_save_ticket);
            if (result) {
                remote_save_ticket=0; remoteCaliSaveFlashFlag=0;
                if (result>0) remoteCaliFlag=0;
                uav_logf(result>0 ? "INFO":"ERROR","RC_CAL","save verified=%u result=%d retry_on_failure=1",
                         (unsigned)(result>0),result);
            }
        }
        if (xQueueReceive(sbus_frames, decode_buffer, pdMS_TO_TICKS(20)) == pdPASS) {
            if (decode_buffer[0] == 0x0F && decode_buffer[24] == 0x00 &&
                !(decode_buffer[23] & 0x0C)) {
                Sbus_Channels_Proc();
                last_frame_ms = platform_millis();
                frame_seen = 1;
                SBUS_CH.Connect_State = 1;
            } else
                SBUS_CH.Connect_State = 0;
        }
        if (!frame_seen || (uint32_t)(platform_millis() - last_frame_ms) > 100)
            SBUS_CH.Connect_State = 0;
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
        published_raw = SBUS_CH;
        taskEXIT_CRITICAL();
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
    if (size == 25 && sbus_frames)
        xQueueSendFromISR(sbus_frames, SbusRxBuf, &wake);
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
