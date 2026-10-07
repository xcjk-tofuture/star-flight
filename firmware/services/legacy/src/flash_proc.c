#include "flash_proc.h"
#include "param_journal.h"
#include "calibration_record.h"
#include "star_protocol.h"
#include "flight_snapshot.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>
static SemaphoreHandle_t storage_mutex;
static QueueHandle_t writes;
static uint8_t flash_checked, flash_ready;
static volatile uint32_t pending_writes;
static uint32_t rejected_writes, storage_sequence[3];
typedef struct {
    uint8_t kind, length;
    uint8_t bytes[72];
} storage_request_t;
osThreadId FlashTaskHandle;
uint8_t uav_storage_busy(void) { return pending_writes != 0; }
void uav_storage_diagnostics(uint32_t *rejected, uint32_t sequence[3]) {
    taskENTER_CRITICAL();
    *rejected = rejected_writes;
    memcpy(sequence, storage_sequence, sizeof(storage_sequence));
    taskEXIT_CRITICAL();
}
int uav_storage_init(void) {
    storage_mutex = xSemaphoreCreateMutex();
    writes = xQueueCreate(2, sizeof(storage_request_t));
    return storage_mutex && writes ? 0 : -1;
}
static void storage_lock(void) {
    if (!storage_mutex || xSemaphoreTake(storage_mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        Error_Handler();
    if (!flash_checked) {
        uint16_t id = W25QXX_ReadID();
        flash_ready = id >= W25Q80 && id <= W25Q128;
        flash_checked = 1;
    }
}
static void storage_unlock(void) { xSemaphoreGive(storage_mutex); }
static uint32_t address(void *ctx, unsigned slot) {
    return (uint32_t)(uintptr_t)ctx + slot * 4096u;
}
static int valid(unsigned slot, unsigned offset, size_t n) {
    return slot < 2 && offset <= PJ_RECORD_BYTES && n <= PJ_RECORD_BYTES - offset;
}
static int read_bytes(void *ctx, unsigned slot, unsigned offset, uint8_t *b, size_t n) {
    if (!flash_ready || !valid(slot, offset, n))
        return -1;
    W25QXX_Read(b, address(ctx, slot) + offset, (uint16_t)n);
    return 0;
}
static int erase_slot(void *ctx, unsigned slot) {
    if (!flash_ready || slot > 1)
        return -1;
    W25QXX_Erase_Sector(address(ctx, slot) / 4096u);
    return 0;
}
static int program_bytes(void *ctx, unsigned slot, unsigned offset, const uint8_t *b, size_t n) {
    if (!flash_ready || !valid(slot, offset, n))
        return -1;
    W25QXX_Write_NoCheck((uint8_t *)b, address(ctx, slot) + offset, (uint16_t)n);
    return 0;
}
static pj_io_t io_for(unsigned kind) {
    pj_io_t io = {(void *)(uintptr_t)(4096u + (kind - 1u) * 8192u), read_bytes, erase_slot,
                  program_bytes};
    return io;
}
static int load(unsigned kind, uint8_t *bytes, size_t length) {
    pj_value_t value;
    pj_io_t io = io_for(kind);
    int ok;
    storage_lock();
    ok = pj_load(&io, (uint16_t)kind, 1, &value) == PJ_OK && value.length == length &&
         calibration_record_valid(kind, value.payload, length);
    if (ok) {
        memcpy(bytes, value.payload, length);
        taskENTER_CRITICAL();
        storage_sequence[kind - 1] = value.sequence;
        taskEXIT_CRITICAL();
    }
    storage_unlock();
    return ok;
}
void UAV_Read_Param_IMU(_imuData_all *d) {
    uint8_t b[72];
    d->magoffsetbias = (Vector3f_t){0, 0, 0};
    d->magscalebias = (Vector3f_t){1, 1, 1};
    if (!load(1, b, sizeof(b)))
        return;
    /* Startup gyro/accelerometer calibration retains ownership. Only magnetic
     * coefficients are restored, as in the refactor baseline. */
    d->magoffsetbias =
        (Vector3f_t){star_read_f32(b + 48), star_read_f32(b + 52), star_read_f32(b + 56)};
    d->magscalebias =
        (Vector3f_t){star_read_f32(b + 60), star_read_f32(b + 64), star_read_f32(b + 68)};
}
void UAV_Read_Param_Remote(_sbus_ch_struct *d) {
    uint8_t b[32];
    memset(b, 0, sizeof(b));
    (void)load(2, b, sizeof(b));
#define GET_CHANNEL(i)                                                                             \
    d->CH##i##_MAX = star_read_u16(b + ((i) - 1) * 4);                                             \
    d->CH##i##_MIN = star_read_u16(b + ((i) - 1) * 4 + 2)
    GET_CHANNEL(1);
    GET_CHANNEL(2);
    GET_CHANNEL(3);
    GET_CHANNEL(4);
    GET_CHANNEL(5);
    GET_CHANNEL(6);
    GET_CHANNEL(7);
    GET_CHANNEL(8);
#undef GET_CHANNEL
}
void UAV_Read_Param_Motor(_uav_control_data *d) {
    uint8_t b[60];
    if (!load(3, b, sizeof(b)))
        return;
    PID_DATA *p[5] = {&d->rollData, &d->pitchData, &d->yawData, &d->rollSpeedData,
                      &d->pitchSpeedData};
    for (unsigned i = 0; i < 5; i++) {
        p[i]->Kp = star_read_f32(b + i * 12);
        p[i]->Ki = star_read_f32(b + i * 12 + 4);
        p[i]->Kd = star_read_f32(b + i * 12 + 8);
    }
}
static int enqueue(storage_request_t *r) {
    int ok = 0;
    flight_snapshot_t state;
    if (!calibration_record_valid(r->kind, r->bytes, r->length)) {
        taskENTER_CRITICAL();
        rejected_writes++;
        taskEXIT_CRITICAL();
        return -2;
    }
    taskENTER_CRITICAL();
    flight_snapshot_read(&state);
    if (state.state == 0 && writes && xQueueSend(writes, r, 0) == pdPASS) {
        pending_writes++;
        ok = 1;
    } else
        rejected_writes++;
    taskEXIT_CRITICAL();
    return ok ? 0 : -1;
}
int UAV_Write_Param_IMU(_imuData_all d) {
    storage_request_t r = {.kind = 1, .length = 72};
    float v[18] = {d.accoffsetbias.x,  d.accoffsetbias.y, d.accoffsetbias.z,  d.accscalebias.x,
                   d.accscalebias.y,   d.accscalebias.z,  d.gyrooffsetbias.x, d.gyrooffsetbias.y,
                   d.gyrooffsetbias.z, d.gyroscalebias.x, d.gyroscalebias.y,  d.gyroscalebias.z,
                   d.magoffsetbias.x,  d.magoffsetbias.y, d.magoffsetbias.z,  d.magscalebias.x,
                   d.magscalebias.y,   d.magscalebias.z};
    for (unsigned i = 0; i < 18; i++)
        star_write_f32(r.bytes + i * 4, v[i]);
    return enqueue(&r);
}
int UAV_Write_Param_Remote(_sbus_ch_struct d) {
    storage_request_t r = {.kind = 2, .length = 32};
#define PUT_CHANNEL(i)                                                                             \
    star_write_u16(r.bytes + ((i) - 1) * 4, d.CH##i##_MAX);                                        \
    star_write_u16(r.bytes + ((i) - 1) * 4 + 2, d.CH##i##_MIN)
    PUT_CHANNEL(1);
    PUT_CHANNEL(2);
    PUT_CHANNEL(3);
    PUT_CHANNEL(4);
    PUT_CHANNEL(5);
    PUT_CHANNEL(6);
    PUT_CHANNEL(7);
    PUT_CHANNEL(8);
#undef PUT_CHANNEL
    return enqueue(&r);
}
int UAV_Write_Param_Motor(_uav_control_data d) {
    storage_request_t r = {.kind = 3, .length = 60};
    const PID_DATA *p[5] = {&d.rollData, &d.pitchData, &d.yawData, &d.rollSpeedData,
                            &d.pitchSpeedData};
    for (unsigned i = 0; i < 5; i++) {
        star_write_f32(r.bytes + i * 12, p[i]->Kp);
        star_write_f32(r.bytes + i * 12 + 4, p[i]->Ki);
        star_write_f32(r.bytes + i * 12 + 8, p[i]->Kd);
    }
    return enqueue(&r);
}
void Flash_Task_Proc(void const *arg) {
    (void)arg;
    storage_request_t r;
    for (;;) {
        if (xQueueReceive(writes, &r, portMAX_DELAY) != pdPASS)
            continue;
        pj_io_t io = io_for(r.kind);
        uint32_t sequence = 0;
        storage_lock();
        int status = pj_save(&io, r.kind, 1, r.bytes, r.length, &sequence);
        storage_unlock();
        taskENTER_CRITICAL();
        if (status == PJ_OK)
            storage_sequence[r.kind - 1] = sequence;
        else
            rejected_writes++;
        pending_writes--;
        taskEXIT_CRITICAL();
    }
}
