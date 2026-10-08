#include "flash_proc.h"
#include "internal_flash_port.h"
#include "calibration_record.h"
#include "mag_record.h"
#include "accel_record.h"
#include "star_protocol.h"
#include "flight_snapshot.h"
#include "uav_actuator.h"
#include "log_service.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>
static SemaphoreHandle_t storage_mutex;
static QueueHandle_t writes;
static nv_store_t store;
static volatile uint32_t pending_writes;
static volatile uint32_t storage_epoch;
static uint32_t rejected_writes, storage_sequence[3], next_ticket, completed_ticket;
typedef struct { uint8_t kind, length; uint32_t ticket; uint8_t bytes[NV_PAYLOAD_MAX]; } storage_request_t;
typedef struct { uint32_t ticket; int status; } storage_completion_t;
static storage_completion_t completed[8];
static unsigned completion_index;
static void update_sequence(unsigned kind, uint32_t seq) {
    if (kind!=UAV_PARAM_IMU_LEGACY && kind!=UAV_PARAM_MAG && kind!=UAV_PARAM_ACCEL &&
        kind!=UAV_PARAM_REMOTE && kind!=UAV_PARAM_PID) return;
    unsigned index=kind==UAV_PARAM_REMOTE ? 1u:kind==UAV_PARAM_PID ? 2u:0u;
    if (!storage_sequence[index] || (int32_t)(seq-storage_sequence[index])>0) storage_sequence[index]=seq;
}
osThreadId FlashTaskHandle;
uint8_t uav_storage_busy(void) { return pending_writes != 0; }
uint8_t uav_storage_ready(void) { return store.ready; }
uint32_t uav_storage_epoch(void) { return storage_epoch; }
int uav_storage_result(uint32_t ticket) {
    if (!ticket) return -2;
    int status=0;
    taskENTER_CRITICAL();
    for (unsigned i=0;i<8;i++) if (completed[i].ticket==ticket) status=completed[i].status;
    if (!status && completed_ticket && (int32_t)(completed_ticket-ticket)>=0) status=-2;
    taskEXIT_CRITICAL();
    return status;
}
void uav_storage_diagnostics(uint32_t *rejected, uint32_t sequence[3]) {
    taskENTER_CRITICAL();
    *rejected=rejected_writes; memcpy(sequence,storage_sequence,sizeof(storage_sequence));
    taskEXIT_CRITICAL();
}
int uav_storage_init(void) {
    storage_mutex=xSemaphoreCreateMutex(); writes=xQueueCreate(2,sizeof(storage_request_t));
    nv_io_t io=uav_internal_flash_io();
    int ready=uav_internal_flash_probe()==0 && nv_store_init(&store,&io)==NV_OK;
    uav_logf(ready ? "INFO":"ERROR","STORAGE",
             "backend=INTERNAL ready=%u banks=0x08040000/0x08060000 bank_KiB=128 generation=%lu seq=%lu erase_on_boot=0",
             (unsigned)ready,(unsigned long)store.generation,(unsigned long)store.sequence);
    return storage_mutex && writes ? 0:-1;
}
static void storage_lock(void) {
    if (!storage_mutex || xSemaphoreTake(storage_mutex,pdMS_TO_TICKS(1000))!=pdTRUE) Error_Handler();
}
static void storage_unlock(void) { xSemaphoreGive(storage_mutex); }
static int load(unsigned kind, uint8_t *bytes, size_t length) {
    uint32_t seq=0; storage_lock();
    int ok=nv_store_get(&store,kind,1,bytes,length,&seq)==NV_OK &&
           calibration_record_valid(kind,bytes,length);
    storage_unlock();
    if (ok) {
        taskENTER_CRITICAL();
        update_sequence(kind,seq);
        taskEXIT_CRITICAL();
    }
    return ok;
}
void UAV_Read_Param_IMU(_imuData_all *d) {
    uint8_t b[72];
    d->accoffsetbias=(Vector3f_t){0,0,0}; d->accscalebias=(Vector3f_t){1,1,1};
    d->magoffsetbias=(Vector3f_t){0,0,0}; d->magscalebias=(Vector3f_t){1,1,1};
    memset(d->magmatrix,0,sizeof(d->magmatrix));
    if (load(UAV_PARAM_IMU_LEGACY,b,72)) {
        d->magoffsetbias=(Vector3f_t){star_read_f32(b+48),star_read_f32(b+52),star_read_f32(b+56)};
        d->magscalebias=(Vector3f_t){star_read_f32(b+60),star_read_f32(b+64),star_read_f32(b+68)};
    }
    d->magmatrix[0]=d->magscalebias.x; d->magmatrix[4]=d->magscalebias.y; d->magmatrix[8]=d->magscalebias.z;
    if (load(UAV_PARAM_MAG,b,UAV_PARAM_MAG_BYTES)) {
        uav_mag_calibration_t c;
        if (uav_mag_record_decode(b,&c)) {
            d->magoffsetbias=(Vector3f_t){c.bias[0],c.bias[1],c.bias[2]};
            memcpy(d->magmatrix,c.matrix,sizeof(c.matrix));
            d->magscalebias=(Vector3f_t){c.matrix[0],c.matrix[4],c.matrix[8]};
            uav_logf("INFO","MAG_CAL","loaded full_matrix=1 samples=%lu coverage=0x%02lx rms_permille=%lu field_milli_uT=%lu",
                     (unsigned long)c.samples,(unsigned long)c.coverage,
                     (unsigned long)(c.rms_fraction*1000),(unsigned long)(c.field_ut*1000));
            for (unsigned i=0;i<3;i++)
                uav_logf("INFO","MAG_CAL","matrix_row=%u milli=%ld,%ld,%ld bias_milli_uT=%ld",
                         i,(long)(c.matrix[3*i]*1000),(long)(c.matrix[3*i+1]*1000),
                         (long)(c.matrix[3*i+2]*1000),(long)(c.bias[i]*1000));
        }
    } else uav_logf("WARN","MAG_CAL","no ellipsoid record; using legacy diagonal/identity; manual calibration recommended");
    if (load(UAV_PARAM_ACCEL,b,UAV_PARAM_ACCEL_BYTES)) {
        uav_accel_calibration_t c;
        if (uav_accel_record_decode(b,&c)) {
            d->accoffsetbias=(Vector3f_t){c.bias[0],c.bias[1],c.bias[2]};
            d->accscalebias=(Vector3f_t){c.scale[0],c.scale[1],c.scale[2]};
            uav_logf("INFO","ACC_CAL","loaded faces=6 samples=%lu rms_permille=%lu bias_milli_mps2=%ld,%ld,%ld scale_milli=%ld,%ld,%ld",
                     (unsigned long)c.samples,(unsigned long)(c.rms_fraction*1000),
                     (long)(c.bias[0]*1000),(long)(c.bias[1]*1000),(long)(c.bias[2]*1000),
                     (long)(c.scale[0]*1000),(long)(c.scale[1]*1000),(long)(c.scale[2]*1000));
        }
    } else uav_logf("WARN","ACC_CAL","no six-face record; using identity correction");
    /* Gyro startup calibration still owns its bias; old IMU fields are not imported. */
}
void UAV_Read_Param_Remote(_sbus_ch_struct *d) {
    uint8_t b[32]={0}; (void)load(UAV_PARAM_REMOTE,b,sizeof(b));
#define GET_CHANNEL(i) \
    d->CH##i##_MAX=star_read_u16(b+((i)-1)*4); \
    d->CH##i##_MIN=star_read_u16(b+((i)-1)*4+2)
    GET_CHANNEL(1); GET_CHANNEL(2); GET_CHANNEL(3); GET_CHANNEL(4);
    GET_CHANNEL(5); GET_CHANNEL(6); GET_CHANNEL(7); GET_CHANNEL(8);
#undef GET_CHANNEL
}
void UAV_Read_Param_Motor(_uav_control_data *d) {
    uint8_t b[60]; if (!load(UAV_PARAM_PID,b,sizeof(b))) return;
    PID_DATA *p[5]={&d->rollData,&d->pitchData,&d->yawData,&d->rollSpeedData,&d->pitchSpeedData};
    for (unsigned i=0;i<5;i++) {
        p[i]->Kp=star_read_f32(b+i*12); p[i]->Ki=star_read_f32(b+i*12+4); p[i]->Kd=star_read_f32(b+i*12+8);
    }
}
int UAV_Read_Param_Settings(uint8_t bytes[UAV_SETTINGS_BYTES]) {
    memset(bytes,0,UAV_SETTINGS_BYTES);
    if (load(UAV_PARAM_SETTINGS,bytes,UAV_SETTINGS_BYTES)) return 1;
    memset(bytes,0,UAV_SETTINGS_BYTES); return 0;
}
static int enqueue(storage_request_t *r, uint32_t *ticket) {
    if (ticket) *ticket=0;
    if (!calibration_record_valid(r->kind,r->bytes,r->length)) {
        taskENTER_CRITICAL(); rejected_writes++; taskEXIT_CRITICAL(); return -2;
    }
    int ok=0; flight_snapshot_t state;
    taskENTER_CRITICAL();
    flight_snapshot_read(&state);
    if (++next_ticket==0) next_ticket++;
    r->ticket=next_ticket;
    if (state.state==0 && store.ready && writes && xQueueSend(writes,r,0)==pdPASS) {
        pending_writes++; storage_epoch++; ok=1; if (ticket) *ticket=r->ticket;
    } else rejected_writes++;
    taskEXIT_CRITICAL();
    return ok ? 0:-1;
}
int UAV_Write_Param_IMU(_imuData_all d) {
    storage_request_t r={.kind=UAV_PARAM_IMU_LEGACY,.length=72};
    float v[18]={d.accoffsetbias.x,d.accoffsetbias.y,d.accoffsetbias.z,d.accscalebias.x,d.accscalebias.y,d.accscalebias.z,
        d.gyrooffsetbias.x,d.gyrooffsetbias.y,d.gyrooffsetbias.z,d.gyroscalebias.x,d.gyroscalebias.y,d.gyroscalebias.z,
        d.magoffsetbias.x,d.magoffsetbias.y,d.magoffsetbias.z,d.magscalebias.x,d.magscalebias.y,d.magscalebias.z};
    for (unsigned i=0;i<18;i++) star_write_f32(r.bytes+4*i,v[i]);
    return enqueue(&r,NULL);
}
int UAV_Write_Param_Settings(const uint8_t bytes[UAV_SETTINGS_BYTES], uint32_t *ticket) {
    storage_request_t r={.kind=UAV_PARAM_SETTINGS,.length=UAV_SETTINGS_BYTES};
    memcpy(r.bytes,bytes,UAV_SETTINGS_BYTES); return enqueue(&r,ticket);
}
int UAV_Write_Param_Mag(const uav_mag_calibration_t *c, uint32_t *ticket) {
    storage_request_t r={.kind=UAV_PARAM_MAG,.length=UAV_PARAM_MAG_BYTES};
    uav_mag_record_encode(r.bytes,c); return enqueue(&r,ticket);
}
int UAV_Write_Param_Accel(const uav_accel_calibration_t *c, uint32_t *ticket) {
    storage_request_t r={.kind=UAV_PARAM_ACCEL,.length=UAV_PARAM_ACCEL_BYTES};
    uav_accel_record_encode(r.bytes,c); return enqueue(&r,ticket);
}
int UAV_Write_Param_Remote_Ticket(_sbus_ch_struct d, uint32_t *ticket) {
    storage_request_t r={.kind=UAV_PARAM_REMOTE,.length=32};
#define PUT_CHANNEL(i) \
    star_write_u16(r.bytes+((i)-1)*4,d.CH##i##_MAX); \
    star_write_u16(r.bytes+((i)-1)*4+2,d.CH##i##_MIN)
    PUT_CHANNEL(1); PUT_CHANNEL(2); PUT_CHANNEL(3); PUT_CHANNEL(4);
    PUT_CHANNEL(5); PUT_CHANNEL(6); PUT_CHANNEL(7); PUT_CHANNEL(8);
#undef PUT_CHANNEL
    return enqueue(&r,ticket);
}
int UAV_Write_Param_Remote(_sbus_ch_struct d) { return UAV_Write_Param_Remote_Ticket(d,NULL); }
int UAV_Write_Param_Motor(_uav_control_data d) {
    storage_request_t r={.kind=UAV_PARAM_PID,.length=60};
    const PID_DATA *p[5]={&d.rollData,&d.pitchData,&d.yawData,&d.rollSpeedData,&d.pitchSpeedData};
    for (unsigned i=0;i<5;i++) {
        star_write_f32(r.bytes+i*12,p[i]->Kp); star_write_f32(r.bytes+i*12+4,p[i]->Ki); star_write_f32(r.bytes+i*12+8,p[i]->Kd);
    }
    return enqueue(&r,NULL);
}
void Flash_Task_Proc(void const *arg) {
    (void)arg; storage_request_t r;
    for (;;) {
        if (xQueueReceive(writes,&r,portMAX_DELAY)!=pdPASS) continue;
        uint32_t sequence=0; flight_snapshot_t flight; flight_snapshot_read(&flight);
        uav_actuator_stop(); flight_attitude_invalidate();
        storage_lock();
        int status=flight.state==0 ? nv_store_put(&store,r.kind,1,r.bytes,r.length,&sequence):NV_IO;
        if (status!=NV_OK && !store.ready) {
            nv_io_t io=uav_internal_flash_io(); (void)nv_store_init(&store,&io);
        }
        storage_unlock();
        taskENTER_CRITICAL();
        if (status==NV_OK) update_sequence(r.kind,sequence);
        else rejected_writes++;
        completed[completion_index]=(storage_completion_t){r.ticket,status==NV_OK ? 1:-1};
        completion_index=(completion_index+1u)%8u; completed_ticket=r.ticket; pending_writes--;
        taskEXIT_CRITICAL();
        uav_logf(status==NV_OK ? "INFO":"ERROR","STORAGE","key=%u ticket=%lu result=%d verified=%u seq=%lu generation=%lu",
                 (unsigned)r.kind,(unsigned long)r.ticket,status,(unsigned)(status==NV_OK),
                 (unsigned long)sequence,(unsigned long)store.generation);
    }
}
