#include "sensor_port.h"
#include "AHRS.h"
#include "attitude.h"
#include "flight_snapshot.h"
#include "log_service.h"
#include "platform_time.h"

#include "lowPassFilter.h"
#include "imu_calibration_config.h"
#include "imu_processing_config.h"
#include "flash_proc.h"
#include "pid.h"
#include "tim.h"
#include "stdio.h"
#include <string.h>

#define RAD_PER_DEG 0.017453293f
#define DEG_PER_RAD 57.29577951f

#define EXTERN_IMU 0

#define SENSORS_ENABLE_SPL06 1;
#define UPDATE_TIME 5
#define UPDATE_TIME_MAG 20


extern void UAV_Read_Param_IMU(_imuData_all *imu_data);
extern int UAV_Write_Param_IMU(_imuData_all imu_data);

extern u8 uart4RX[200];


osThreadId SensorDataTaskHandle;

// u16 FlashTest;
acc_raw_data_t test_acc;
gyro_raw_data_t test_gyro;
mag_raw_data_t test_mag;

_imuData_all imudata_all;
static _imuData_all published_sensors;
void sensor_snapshot_read(_imuData_all *out) {
    taskENTER_CRITICAL();
    *out = published_sensors;
    taskEXIT_CRITICAL();
}
_ahrs_data attitude_t;

PID_DATA imu_temperature_control_pid_data;
PID imu_temperature_control_pid;

u8 SensorError = 0;
static u8 GyroCalFlag = 1; // 传感器校准标准位
static uav_gyro_calibration_t startup_gyro;
static uav_imu_pipeline_t imu_pipeline;
static uav_imu_frame_t imu_frame;
static uav_fusion_t imu_fusion;
static uint32_t mag_sample_us, imu_read_errors;
static uint32_t mag_read_errors, mag_not_ready, mag_fresh_samples;
static uav_mag_calibrator_t mag_calibrator;
static uav_mag_calibration_t mag_candidate;
static uint8_t magCalistep, mag_fit_reason;
static uint32_t mag_started_ms, mag_fit_ms, mag_save_ticket, mag_finished_ms, mag_progress_ms;
static float mag_turn_rate;
static uint8_t mag_turn_seen;
static uint8_t mag_fit_attempted;
static uav_accel_calibrator_t accel_calibrator;
static uav_accel_calibration_t accel_candidate;
static uint8_t AccelCalFlag, accel_requests, accel_restart_gyro;
static uint32_t accel_started_ms, accel_save_ticket, accel_finished_ms, accel_progress_ms;
static void accel_calibration_update(uint32_t now);

static u8 MagCalFlag = 0;
uint8_t sensor_imu_calibrating(void) {
    taskENTER_CRITICAL();
    uint8_t active = GyroCalFlag || AccelCalFlag;
    taskEXIT_CRITICAL();
    return active;
}
uint8_t sensor_imu_calibration_failed(void) { return startup_gyro.failed; }
void sensor_processing_stats_read(sensor_processing_stats_t *out) {
    if (!out) return;
    taskENTER_CRITICAL();
    *out=(sensor_processing_stats_t){
        .samples=imu_pipeline.stats.samples,.invalid_samples=imu_pipeline.stats.invalid_samples,
        .read_errors=imu_read_errors,.timing_resets=imu_pipeline.stats.timing_resets,
        .filter_resets=imu_pipeline.stats.filter_resets,.dt_min_us=imu_pipeline.stats.dt_min_us,
        .dt_max_us=imu_pipeline.stats.dt_max_us,.sample_hz=imu_pipeline.stats.measured_hz,
        .accel_rejected=imu_fusion.accel_rejected,.mag_rejected=imu_fusion.mag_rejected,
        .cal_samples=startup_gyro.count,.cal_failed=startup_gyro.failed,.mag_used=imu_fusion.mag_used};
    taskEXIT_CRITICAL();
}
static uint8_t mag_request;
void sensors_request_mag_calibration(void) {
    taskENTER_CRITICAL();
    mag_request |= 1u;
    taskEXIT_CRITICAL();
}
void sensors_cancel_mag_calibration(void) {
    taskENTER_CRITICAL(); mag_request |= 2u; taskEXIT_CRITICAL();
}
void sensor_mag_calibration_stats_read(sensor_mag_calibration_stats_t *out) {
    taskENTER_CRITICAL();
    *out=(sensor_mag_calibration_stats_t){.samples=mag_calibrator.count,
        .rms_permille=(uint16_t)(isfinite(mag_candidate.rms_fraction) && mag_candidate.rms_fraction<65 ? mag_candidate.rms_fraction*1000:65535),
        .coverage=(uint8_t)mag_candidate.coverage,.reason=mag_fit_reason,.step=magCalistep,
        .quality_ready=(uint8_t)(mag_fit_attempted && mag_candidate.samples>0),
        .sphere_ready=mag_calibrator.sphere.ready,.sphere_fitted=mag_calibrator.sphere.fitted,
        .sphere_cursor_valid=mag_calibrator.sphere.cursor_valid,.sphere_covered=mag_calibrator.sphere.covered,
        .sphere_mask=mag_calibrator.sphere.mask};
    memcpy(out->sphere_cursor,mag_calibrator.sphere.cursor,sizeof(out->sphere_cursor));
    memcpy(out->sphere_goal,mag_calibrator.sphere.goal,sizeof(out->sphere_goal));
    if (!out->quality_ready) out->coverage=0;
    out->hint=mag_turn_rate>1.5f ? 1:mag_turn_rate<.1f ? 2:0;
    taskEXIT_CRITICAL();
}
void sensors_request_accel_calibration(void) {
    taskENTER_CRITICAL(); accel_requests|=1u; taskEXIT_CRITICAL();
}
void sensors_cancel_accel_calibration(void) {
    taskENTER_CRITICAL(); accel_requests|=4u; taskEXIT_CRITICAL();
}
uint8_t sensors_accel_calibration_active(void) { return AccelCalFlag; }
void sensor_accel_calibration_stats_read(sensor_accel_calibration_stats_t *out) {
    taskENTER_CRITICAL();
    *out=(sensor_accel_calibration_stats_t){accel_calibrator.count,
        (uint16_t)(isfinite(accel_candidate.rms_fraction) && accel_candidate.rms_fraction<65 ? accel_candidate.rms_fraction*1000:65535), accel_calibrator.mask, accel_calibrator.target,
        accel_calibrator.detected, accel_calibrator.phase, accel_calibrator.reason, AccelCalFlag,
        {imu_pipeline.sample.acc_uncalibrated[0],imu_pipeline.sample.acc_uncalibrated[1],imu_pipeline.sample.acc_uncalibrated[2]}};
    taskEXIT_CRITICAL();
}
static void mag_motion_update(void) {
    const float *g=imu_pipeline.sample.gyro_control;
    mag_turn_rate=sqrtf(g[0]*g[0]+g[1]*g[1]+g[2]*g[2]);
    mag_turn_seen=1;
}
uint8_t sensors_mag_calibration_active(void) { return MagCalFlag; } // 传感器校准标准位
u8 Bmi088Init_Flag = 1;
u8 AK8975Flag = 1;
u8 SPL06Flag = 1;
u8 IMUTemperatureFlag = 1;

u32 sensorTimeCount = 0;
static void publish_sensor_values(void) {
    taskENTER_CRITICAL();
    published_sensors=imudata_all;
    taskEXIT_CRITICAL();
}
static void copy_imu_sample(void) {
    const uav_imu_sample_t *s=&imu_pipeline.sample;
    imudata_all.acc=(acc_raw_data_t){s->acc[0],s->acc[1],s->acc[2]};
    imudata_all.gyro=(gyro_raw_data_t){s->gyro_control[0],s->gyro_control[1],s->gyro_control[2]};
    float raw[3]={test_mag.x,test_mag.y,test_mag.z};
    float bias[3]={imudata_all.magoffsetbias.x,imudata_all.magoffsetbias.y,imudata_all.magoffsetbias.z},corrected[3];
    uav_mag_correct(imudata_all.magmatrix,bias,raw,corrected);
    imudata_all.mag=(mag_raw_data_t){corrected[0],corrected[1],corrected[2]};
}
void Sensor_Data_Task_Proc(void const *argument) {
    (void)argument;
    osDelay(1000);
    Sensors_Init();
    UAV_Read_Param_IMU(&imudata_all);
    int configured=uav_imu_pipeline_init(&imu_pipeline,&uav_board_imu_processing)==0 &&
                   uav_fusion_init(&imu_fusion,&uav_board_fusion)==0 &&
                   uav_gyro_calibration_init(&startup_gyro,&uav_board_gyro_calibration)==0;
    float acc_bias[3]={imudata_all.accoffsetbias.x,imudata_all.accoffsetbias.y,imudata_all.accoffsetbias.z};
    float acc_scale[3]={imudata_all.accscalebias.x,imudata_all.accscalebias.y,imudata_all.accscalebias.z};
    if (configured && uav_imu_pipeline_set_accel_calibration(&imu_pipeline,acc_bias,acc_scale)) configured=0;
    if (!configured) { startup_gyro.failed=1; GyroCalFlag=0; SensorError=1; }
    uav_logf(configured ? "INFO":"ERROR","IMU_PIPE",
             "sample_target=500Hz burst=ACC8+GYRO7 fusion_target=200Hz gyro_LPF=40Hz acc_LPF=30Hz cal_LPF=5Hz max_gap=10ms configured=%u",
             (unsigned)configured);
    uav_logf("INFO","GYRO_CAL",
             "input=LPF5Hz warmup=%ums timeout=%lums window=%u max_rate=%ldmrad/s max_std=%ldmrad/s",
             (unsigned)UAV_GYRO_CAL_WARMUP_MS,(unsigned long)UAV_GYRO_CAL_TIMEOUT_MS,
             (unsigned)uav_board_gyro_calibration.samples,
             (long)(uav_board_gyro_calibration.max_rate_rad_s*1000),
             (long)(uav_board_gyro_calibration.max_rate_std_rad_s*1000));
    uint32_t cal_start=platform_millis(), cal_report=cal_start, warmup_start=cal_start;
    uint32_t last_mag=cal_start, last_baro=cal_start, last_temp=cal_start, last_heater=cal_start;
    uint32_t last_mag_retry=cal_start, last_report=cal_start, mag_retries=0;
    uint32_t next_fusion=platform_micros()+UAV_IMU_FUSION_PERIOD_US;
    uint32_t rejected[UAV_GYRO_REASON_COUNT]={0}, cal_timing_resets=0;
    TickType_t wake=xTaskGetTickCount();
    uint8_t storage_paused=0;
    uint32_t storage_seen=uav_storage_epoch();
    for (;;) {
        vTaskDelayUntil(&wake,pdMS_TO_TICKS(UAV_IMU_SAMPLE_PERIOD_MS));
        uint32_t now=platform_millis();
        if ((TickType_t)(xTaskGetTickCount()-wake)>=pdMS_TO_TICKS(UAV_IMU_SAMPLE_PERIOD_MS))
            wake=xTaskGetTickCount(); /* Drop catch-up bursts, retain real sample time. */
        sensorTimeCount++;
        /* F407 has one Flash bank: erase/program stalls instruction fetches.
         * Only disarmed saves are allowed. Drop the interval across the save. */
        uint32_t storage_current=uav_storage_epoch();
        if (storage_current!=storage_seen) {
            storage_seen=storage_current; storage_paused=1;
            uav_imu_pipeline_discard(&imu_pipeline);
            if (AccelCalFlag) uav_accel_calibrator_discard_window(&accel_calibrator);
            mag_turn_seen=0;
        }
        if (uav_storage_busy()) {
            if (!storage_paused) uav_imu_pipeline_discard(&imu_pipeline);
            storage_paused=1; flight_attitude_invalidate(); continue;
        }
        if (storage_paused) {
            storage_paused=0; next_fusion=platform_micros()+UAV_IMU_FUSION_PERIOD_US;
            if (GyroCalFlag) {
                uav_gyro_calibration_init(&startup_gyro,&uav_board_gyro_calibration);
                warmup_start=now; cal_timing_resets++;
            }
        }
        if (accel_restart_gyro) {
            accel_restart_gyro=0;
            uav_gyro_calibration_init(&startup_gyro,&uav_board_gyro_calibration);
            GyroCalFlag=1; cal_start=cal_report=warmup_start=now;
            memset(rejected,0,sizeof(rejected)); cal_timing_resets=0;
            SensorError=Bmi088Init_Flag || AK8975Flag;
        }
        if (GyroCalFlag && uav_gyro_calibration_expire(&startup_gyro,(uint32_t)(now-cal_start),UAV_GYRO_CAL_TIMEOUT_MS)) {
            GyroCalFlag=0; SensorError=1; flight_attitude_invalidate();
            uav_logf("ERROR","GYRO_CAL","FAILED timeout=%lums collected=%u/%u rate=%lu gravity=%lu variance=%lu invalid=%lu timing=%lu retry=RESET",
                     (unsigned long)UAV_GYRO_CAL_TIMEOUT_MS,(unsigned)startup_gyro.count,
                     (unsigned)startup_gyro.config.samples,(unsigned long)rejected[UAV_GYRO_REASON_RATE],
                     (unsigned long)rejected[UAV_GYRO_REASON_GRAVITY],(unsigned long)rejected[UAV_GYRO_REASON_VARIANCE],
                     (unsigned long)rejected[UAV_GYRO_REASON_INVALID],(unsigned long)cal_timing_resets);
        }
        float acc[3],gyro[3];
        uint32_t begin_us=platform_micros();
        int read_status=Bmi088Init_Flag ? -1 : uav_sensor_read_imu(acc,gyro);
        uint32_t end_us=platform_micros(), sample_us=begin_us+(uint32_t)(end_us-begin_us)/2;
        uint32_t sample_ms=platform_millis();
        uint32_t previous_resets=imu_pipeline.stats.filter_resets;
        int sample_good=configured && read_status==0 && uav_imu_pipeline_push(&imu_pipeline,acc,gyro,sample_us)==0;
        if (!sample_good) {
            if (read_status) { imu_read_errors++; uav_imu_pipeline_discard(&imu_pipeline); }
            flight_attitude_invalidate();
            if (AccelCalFlag) uav_accel_calibrator_discard_window(&accel_calibrator);
            mag_turn_seen=0;
            if (GyroCalFlag) {
                uav_gyro_calibration_init(&startup_gyro,&uav_board_gyro_calibration);
                cal_timing_resets++; warmup_start=now;
            }
        } else {
            test_acc=(acc_raw_data_t){acc[0],acc[1],acc[2]};
            test_gyro=(gyro_raw_data_t){gyro[0],gyro[1],gyro[2]};
            copy_imu_sample();
            if (MagCalFlag) mag_motion_update();
            if (AccelCalFlag && accel_calibrator.phase<=UAV_ACCEL_SAMPLE) {
                uint8_t before=accel_calibrator.mask,face=accel_calibrator.target;
                (void)uav_accel_calibrator_feed(&accel_calibrator,acc,imu_pipeline.sample.acc_uncalibrated,
                                               imu_pipeline.sample.gyro_control,now);
                if (accel_calibrator.mask!=before)
                    uav_logf("INFO","ACC_CAL","face=%u complete mask=0x%02x mean_milli_mps2=%ld,%ld,%ld",
                             face,accel_calibrator.mask,(long)(accel_calibrator.faces[face][0]*1000),
                             (long)(accel_calibrator.faces[face][1]*1000),(long)(accel_calibrator.faces[face][2]*1000));
            }
            if (GyroCalFlag && previous_resets!=imu_pipeline.stats.filter_resets) {
                uav_gyro_calibration_init(&startup_gyro,&uav_board_gyro_calibration);
                warmup_start=now; cal_timing_resets++;
            }
        }
        if ((uint32_t)(now-last_mag)>=UAV_IMU_MAG_PERIOD_MS) {
            last_mag=now;
            float field[3]; int status=uav_sensor_read_mag(field);
            if (status==1) {
                test_mag=(mag_raw_data_t){field[0],field[1],field[2]};
                mag_sample_us=platform_micros(); mag_fresh_samples++;
                if (MagCalFlag) uav_mag_calibrator_update_cursor(&mag_calibrator,field);
                if (MagCalFlag && sample_good && magCalistep==SENSOR_MAG_COLLECT && mag_turn_seen &&
                    mag_turn_rate>=.1f && mag_turn_rate<=1.5f)
                    (void)uav_mag_calibrator_feed(&mag_calibrator,field,now);
            } else if (status<0) mag_read_errors++;
            else mag_not_ready++;
            if (AK8975Flag && mag_retries<4 && (uint32_t)(now-last_mag_retry)>=500u) {
                last_mag_retry=now; mag_retries++; AK8975Flag=DrvAK8975Check();
                SensorError=Bmi088Init_Flag || AK8975Flag || startup_gyro.failed;
                uav_logf(AK8975Flag ? "WARN":"INFO","MAG_RECHECK","attempt=%u/4 rc=%u",
                         (unsigned)mag_retries,(unsigned)AK8975Flag);
            }
            if (sample_good) copy_imu_sample();
        }
        if ((uint32_t)(now-last_temp)>=UAV_IMU_TEMPERATURE_PERIOD_MS) {
            last_temp=now;
            if (!Bmi088Init_Flag) ReadAccTemperature(&imudata_all.f_temperature);
        }
        if ((uint32_t)(now-last_heater)>=50u) {
            last_heater=now;
            if (!Bmi088Init_Flag && isfinite(imudata_all.f_temperature) &&
                imudata_all.f_temperature>=-40 && imudata_all.f_temperature<=85)
                IMU_Temperature_Control(40);
            else uav_device_heater_write(0);
        }
        if (!SPL06Flag && (uint32_t)(now-last_baro)>=UAV_IMU_BARO_PERIOD_MS) {
            last_baro=now; imudata_all.Pressure=Drv_SPl0601_Read();
        }
        taskENTER_CRITICAL();
        uint8_t request=mag_request; mag_request=0;
        taskEXIT_CRITICAL();
        if ((request & 2u) && MagCalFlag && magCalistep<SENSOR_MAG_SAVE) {
            MagCalFlag=0; magCalistep=SENSOR_MAG_COLLECT;
            Cold_Start_ARHS(imudata_all,&attitude_t);
            uav_logf("INFO","MAG_CAL","cancelled; previous calibration retained");
        }
        if ((request & 1u) && !MagCalFlag) {
            flight_snapshot_t flight; flight_snapshot_read(&flight);
            if (flight.state==0 && !GyroCalFlag && !AccelCalFlag && !sbus_calibration_active() &&
                !startup_gyro.failed && !AK8975Flag && uav_storage_ready()) {
                uav_mag_calibrator_init(&mag_calibrator); memset(&mag_candidate,0,sizeof(mag_candidate));
                mag_started_ms=mag_fit_ms=mag_progress_ms=now; mag_save_ticket=0;
                magCalistep=SENSOR_MAG_COLLECT; mag_fit_reason=UAV_MAG_CAL_SAMPLES;
                MagCalFlag=1; flight_attitude_invalidate();
                mag_turn_seen=0; mag_turn_rate=0;
                float current[3]={test_mag.x,test_mag.y,test_mag.z};
                uav_mag_calibrator_refresh_sphere(&mag_calibrator,NULL,current,now);
                mag_fit_attempted=0;
                uav_logf("INFO","MAG_CAL","ellipsoid start min_samples=200 capacity=512 timeout=180s rotate=ALL_AXES raw_fresh_only=1");
            } else uav_logf("WARN","SENSOR","magnetic calibration rejected: state=%u gyro_cal=%u failed=%u mag_rc=%u",
                           (unsigned)flight.state,(unsigned)GyroCalFlag,(unsigned)startup_gyro.failed,(unsigned)AK8975Flag);
        }
        taskENTER_CRITICAL();
        uint8_t accel_request=accel_requests; accel_requests=0;
        taskEXIT_CRITICAL();
        if ((accel_request&4u) && AccelCalFlag && accel_calibrator.phase<UAV_ACCEL_SAVE) {
            AccelCalFlag=0; accel_calibrator.phase=UAV_ACCEL_PLACE;
            Cold_Start_ARHS(imudata_all,&attitude_t);
            uav_logf("INFO","ACC_CAL","cancelled; previous calibration retained");
        }
        if ((accel_request&1u) && !AccelCalFlag) {
            flight_snapshot_t flight; flight_snapshot_read(&flight);
            if (flight.state==0 && !GyroCalFlag && !MagCalFlag && !Bmi088Init_Flag &&
                !sbus_calibration_active() && uav_storage_ready()) {
                uav_accel_calibrator_init(&accel_calibrator); memset(&accel_candidate,0,sizeof(accel_candidate));
                accel_started_ms=accel_progress_ms=now; accel_save_ticket=0; AccelCalFlag=1;
                flight_attitude_invalidate();
                uav_logf("INFO","ACC_CAL","start faces=ANY_ORDER LPF=5Hz hold_ms=800 samples_per_face=200 sample_Hz=100 timeout_s=300 auto_detect=1");
            } else uav_logf("WARN","ACC_CAL","start rejected state=%u gyro=%u mag_cal=%u bmi=%u",
                           flight.state,GyroCalFlag,MagCalFlag,Bmi088Init_Flag);
        }
        if (AccelCalFlag) accel_calibration_update(now);
        if (MagCalFlag) Mag_Zero_Offset_Calibration(&imudata_all);
        if (sample_good && (int32_t)(sample_us-next_fusion)>=0) {
            next_fusion+=UAV_IMU_FUSION_PERIOD_US;
            if ((int32_t)(sample_us-next_fusion)>=0) next_fusion=sample_us+UAV_IMU_FUSION_PERIOD_US;
            if (uav_imu_pipeline_consume(&imu_pipeline,&imu_frame)==0) {
                if (GyroCalFlag && (uint32_t)(now-warmup_start)>=UAV_GYRO_CAL_WARMUP_MS) {
                    float bias[3];
                    int result=uav_gyro_calibration_feed(&startup_gyro,imu_frame.gyro_calibration,imu_frame.acc,bias);
                    if (result==UAV_GYRO_REJECTED) rejected[startup_gyro.reject_reason]++;
                    else if (result==UAV_GYRO_READY) {
                        imudata_all.gyrooffsetbias=(Vector3f_t){bias[0],bias[1],bias[2]};
                        uav_imu_pipeline_set_bias(&imu_pipeline,bias);
                        GyroCalFlag=0;
                        Cold_Start_ARHS(imudata_all,&attitude_t);
                        uav_logf("INFO","GYRO_CAL","complete elapsed=%lums bias_mrad/s=%ld,%ld,%ld",
                                 (unsigned long)(now-cal_start),(long)(bias[0]*1000),(long)(bias[1]*1000),(long)(bias[2]*1000));
                        /* This frame predates the new bias. Publish on the next fresh interval. */
                        imu_frame.dt_s=0;
                    }
                }
                if (!GyroCalFlag && !startup_gyro.failed && !MagCalFlag && !AccelCalFlag && imu_frame.dt_s>0) {
                    if (AHRS_Mahony_Update(imudata_all,&attitude_t)==0)
                        flight_attitude_publish_sample(attitude_t.roll,attitude_t.pitch,attitude_t.yaw,
                            attitude_t.rollSpeed,attitude_t.pitchSpeed,attitude_t.yawSpeed,sample_ms);
                    else flight_attitude_invalidate();
                } else if (MagCalFlag || AccelCalFlag) {
                    flight_attitude_invalidate();
                }
                publish_sensor_values();
            }
        }
        if (GyroCalFlag && (uint32_t)(now-cal_report)>=1000u) {
            cal_report=now;
            uav_logf("INFO","GYRO_CAL","progress=%u/%u elapsed=%lums rate=%lu gravity=%lu variance=%lu invalid=%lu timing=%lu",
                     (unsigned)startup_gyro.count,(unsigned)startup_gyro.config.samples,(unsigned long)(now-cal_start),
                     (unsigned long)rejected[UAV_GYRO_REASON_RATE],(unsigned long)rejected[UAV_GYRO_REASON_GRAVITY],
                     (unsigned long)rejected[UAV_GYRO_REASON_VARIANCE],(unsigned long)rejected[UAV_GYRO_REASON_INVALID],
                     (unsigned long)cal_timing_resets);
        }
        if ((uint32_t)(now-last_report)>=5000u) {
            last_report=now;
            uav_logf(startup_gyro.failed ? "ERROR":"INFO","IMU_PIPE","Hz=%lu cal=%u failed=%u dt_us=%lu..%lu bad=%lu read_errors=%lu timing=%lu filter_resets=%lu acc_weight=%u mag_used=%u acc_reject=%lu mag_reject=%lu",
                     (unsigned long)imu_pipeline.stats.measured_hz,(unsigned)GyroCalFlag,(unsigned)startup_gyro.failed,
                     (unsigned long)imu_pipeline.stats.dt_min_us,
                     (unsigned long)imu_pipeline.stats.dt_max_us,(unsigned long)imu_pipeline.stats.invalid_samples,
                     (unsigned long)imu_read_errors,(unsigned long)imu_pipeline.stats.timing_resets,
                     (unsigned long)imu_pipeline.stats.filter_resets,(unsigned)(imu_fusion.accel_weight*1000),
                     (unsigned)imu_fusion.mag_used,(unsigned long)imu_fusion.accel_rejected,(unsigned long)imu_fusion.mag_rejected);
            uint8_t st1,st2; uav_sensor_mag_status(&st1,&st2);
            uav_logf("INFO","HEADING","reason=%u innov_mdeg=%ld weight=%u norm_milli_uT=%ld horiz_milli_uT=%ld online_bias_mrad/s=%ld,%ld,%ld mag_fresh=%lu pending=%lu errors=%lu ST1=0x%02x ST2=0x%02x",
                     (unsigned)imu_fusion.mag_reason,(long)(imu_fusion.mag_innovation_rad*57295.77951f),
                     (unsigned)(imu_fusion.mag_weight*1000),(long)(imu_fusion.mag_norm_ut*1000),
                     (long)(imu_fusion.mag_horizontal_ut*1000),(long)(imu_fusion.gyro_bias[0]*1000),
                     (long)(imu_fusion.gyro_bias[1]*1000),(long)(imu_fusion.gyro_bias[2]*1000),
                     (unsigned long)mag_fresh_samples,(unsigned long)mag_not_ready,(unsigned long)mag_read_errors,
                     (unsigned)st1,(unsigned)st2);
        }
    }
}

void AHRS_Uart4_IDLE_Proc(u8 size) {
    if (size == 56 && uart4RX[0] == 0xFC && uart4RX[1] == 0x41) {
#if EXTERN_IMU
        attitude_t.rollSpeed =
            DATA_Trans(uart4RX[7], uart4RX[8], uart4RX[9], uart4RX[10]) * DEG_PER_RAD; // 横滚角速度
        attitude_t.pitchSpeed = DATA_Trans(uart4RX[11], uart4RX[12], uart4RX[13], uart4RX[14]) *
                                DEG_PER_RAD; // 俯仰角速度
        attitude_t.yawSpeed = DATA_Trans(uart4RX[15], uart4RX[16], uart4RX[17], uart4RX[18]) *
                              DEG_PER_RAD; // 偏航角速度

        attitude_t.roll =
            DATA_Trans(uart4RX[19], uart4RX[20], uart4RX[21], uart4RX[22]) * DEG_PER_RAD; // 横滚角
        attitude_t.pitch =
            DATA_Trans(uart4RX[23], uart4RX[24], uart4RX[25], uart4RX[26]) * DEG_PER_RAD; // 俯仰角
        attitude_t.yaw =
            DATA_Trans(uart4RX[27], uart4RX[28], uart4RX[29], uart4RX[30]) * DEG_PER_RAD; // 偏航角

        attitude_t.q0 = DATA_Trans(uart4RX[31], uart4RX[32], uart4RX[33], uart4RX[34]); // 四元数
        attitude_t.q1 = DATA_Trans(uart4RX[35], uart4RX[36], uart4RX[37], uart4RX[38]);
        attitude_t.q2 = DATA_Trans(uart4RX[39], uart4RX[40], uart4RX[41], uart4RX[42]);
        attitude_t.q3 = DATA_Trans(uart4RX[43], uart4RX[44], uart4RX[45], uart4RX[46]);
#endif
    }
}

float DATA_Trans(u8 Data_1, u8 Data_2, u8 Data_3, u8 Data_4) {
    u32 transition_32;
    float tmp = 0;
    int sign = 0;
    int exponent = 0;
    float mantissa = 0;
    transition_32 = 0;
    transition_32 |= Data_4 << 24;
    transition_32 |= Data_3 << 16;
    transition_32 |= Data_2 << 8;
    transition_32 |= Data_1;
    sign = (transition_32 & 0x80000000) ? -1 : 1; // 符号位
    // 先右移操作，再按位与计算，出来结果是30到23位对应的e
    exponent = ((transition_32 >> 23) & 0xff) - 127;
    // 将22~0转化为10进制，得到对应的x系数
    mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
    tmp = sign * mantissa * pow(2, exponent);
    return tmp;
}

void Sensors_Init() // 传感器初始化
{

    // UAV_Read_Param_IMU(&imudata_all);  //读传感器校准数据

    AK8975Flag = DrvAK8975Check();
    Bmi088Init_Flag = BMI088_INIT();
    if (!Bmi088Init_Flag) {
        uint8_t acc_conf, gyro_band, acc_range, gyro_range;
        WriteDataToAcc(ACC_CONF_ADDR,UAV_IMU_ACCEL_CONFIG);
        WriteDataToGyro(GYRO_BANDWIDTH_ADDR,UAV_IMU_GYRO_BANDWIDTH);
        ReadSingleDataFromAcc(ACC_CONF_ADDR,&acc_conf);
        ReadSingleDataFromGyro(GYRO_BANDWIDTH_ADDR,&gyro_band);
        ReadSingleDataFromAcc(ACC_RANGE_ADDR,&acc_range);
        ReadSingleDataFromGyro(GYRO_RANGE_ADDR,&gyro_range);
        if ((acc_conf&0x7fu)!=(UAV_IMU_ACCEL_CONFIG&0x7fu) || (gyro_band&7u)!=UAV_IMU_GYRO_BANDWIDTH ||
            (acc_range&3u)!=0 || (gyro_range&7u)!=2) Bmi088Init_Flag=1;
        uav_logf(Bmi088Init_Flag ? "ERROR":"INFO","IMU_PROFILE",
                 "acc_conf=0x%02x gyro_bw=0x%02x acc_range=%u gyro_range=%u expected=3g/500dps acc_ODR=800Hz gyro_ODR=1000Hz BW=116Hz",
                 (unsigned)acc_conf,(unsigned)gyro_band,(unsigned)acc_range,(unsigned)gyro_range);
    }

#ifdef SENSORS_ENABLE_SPL06
    SPL06Flag = Drv_Spl0601_Init();
#endif

    if (Bmi088Init_Flag || AK8975Flag) {
#ifdef SENSORS_ENABLE_SPL06
        if (SPL06Flag)
            SensorError = 1;
#endif
        SensorError = 1;
    }

    IMU_Temperature_Control_Init();
    uav_logf(Bmi088Init_Flag ? "ERROR" : (AK8975Flag || SPL06Flag) ? "WARN" : "INFO", "SENSOR",
             "init_complete BMI088_rc=%u MAG_rc=%u BARO_rc=%u attitude_mode=%s",
             (unsigned)Bmi088Init_Flag, (unsigned)AK8975Flag, (unsigned)SPL06Flag,
             Bmi088Init_Flag ? "BLOCKED" : AK8975Flag ? "6AXIS" : "9AXIS");
    if (!Bmi088Init_Flag && AK8975Flag)
        uav_logf("WARN", "AHRS", "magnetometer unavailable; six-axis attitude; yaw has no magnetic reference and may drift");
}

uint8_t sensor_calibration_step(void) { return magCalistep; }
static void mag_calibration_fail(uint8_t reason) {
    mag_fit_reason=reason; magCalistep=SENSOR_MAG_FAILED; MagCalFlag=0;
    Cold_Start_ARHS(imudata_all,&attitude_t);
    uav_logf("ERROR","MAG_CAL","FAILED reason=%u samples=%u coverage=0x%02lx previous_calibration_retained=1",
             (unsigned)reason,(unsigned)mag_calibrator.count,(unsigned long)mag_candidate.coverage);
}
void Mag_Zero_Offset_Calibration(_imuData_all *imu) {
    uint32_t now=platform_millis();
    if (magCalistep==SENSOR_MAG_COLLECT) {
        if ((uint32_t)(now-mag_started_ms)>=UAV_MAG_CAL_TIMEOUT_MS) {
            mag_calibration_fail(UAV_MAG_CAL_TIMEOUT); return;
        }
        if ((uint32_t)(now-mag_calibrator.sphere_ms)>=500u) {
            float current[3]={test_mag.x,test_mag.y,test_mag.z};
            uav_mag_calibrator_refresh_sphere(&mag_calibrator,&mag_candidate,current,now);
        }
        if (mag_calibrator.count>=UAV_MAG_CAL_MIN_SAMPLES &&
            (uint32_t)(now-mag_fit_ms)>=5000u) {
            mag_fit_ms=now; magCalistep=SENSOR_MAG_FIT;
            mag_fit_attempted=1;
            mag_candidate.samples=0;
            mag_fit_reason=(uint8_t)uav_mag_calibrator_fit(&mag_calibrator,&mag_candidate);
            float current[3]={test_mag.x,test_mag.y,test_mag.z};
            uav_mag_calibrator_refresh_sphere(&mag_calibrator,&mag_candidate,current,now);
            if (mag_fit_reason==UAV_MAG_CAL_OK) {
                if (UAV_Write_Param_Mag(&mag_candidate,&mag_save_ticket)!=0) {
                    mag_calibration_fail(UAV_MAG_CAL_STORAGE); return;
                }
                magCalistep=SENSOR_MAG_SAVE;
                uav_logf("INFO","MAG_CAL","fit accepted samples=%u coverage=0x%02lx rms_permille=%lu field_milli_uT=%lu ticket=%lu awaiting_readback=1",
                         (unsigned)mag_calibrator.count,(unsigned long)mag_candidate.coverage,
                         (unsigned long)(mag_candidate.rms_fraction*1000),(unsigned long)(mag_candidate.field_ut*1000),
                         (unsigned long)mag_save_ticket);
            } else magCalistep=SENSOR_MAG_COLLECT;
        }
        if ((uint32_t)(now-mag_progress_ms)>=2000u) {
            mag_progress_ms=now;
            uav_logf("INFO","MAG_CAL","collect elapsed_ms=%lu samples=%u/200 coverage=0x%02lx reason=%u rotate_all_axes=1",
                     (unsigned long)(now-mag_started_ms),(unsigned)mag_calibrator.count,
                     (unsigned long)mag_candidate.coverage,(unsigned)mag_fit_reason);
        }
    } else if (magCalistep==SENSOR_MAG_SAVE) {
        int result=uav_storage_result(mag_save_ticket);
        if (!result) return;
        if (result<0) { mag_calibration_fail(UAV_MAG_CAL_STORAGE); return; }
        imu->magoffsetbias=(Vector3f_t){mag_candidate.bias[0],mag_candidate.bias[1],mag_candidate.bias[2]};
        memcpy(imu->magmatrix,mag_candidate.matrix,sizeof(imu->magmatrix));
        imu->magscalebias=(Vector3f_t){mag_candidate.matrix[0],mag_candidate.matrix[4],mag_candidate.matrix[8]};
        copy_imu_sample(); Cold_Start_ARHS(*imu,&attitude_t);
        mag_finished_ms=now; magCalistep=SENSOR_MAG_DONE;
        uav_logf("INFO","MAG_CAL","COMPLETE saved=1 readback=1 full_matrix=1 fusion_input=CORRECTED bias_milli_uT=%ld,%ld,%ld",
                 (long)(mag_candidate.bias[0]*1000),(long)(mag_candidate.bias[1]*1000),(long)(mag_candidate.bias[2]*1000));
    } else if (magCalistep==SENSOR_MAG_DONE && (uint32_t)(now-mag_finished_ms)>=1000u) {
        MagCalFlag=0;
    }
}
static void accel_calibration_fail(uint8_t reason) {
    accel_calibrator.reason=reason; accel_calibrator.phase=UAV_ACCEL_FAILED; AccelCalFlag=0;
    Cold_Start_ARHS(imudata_all,&attitude_t);
    uav_logf("ERROR","ACC_CAL","FAILED reason=%u mask=0x%02x previous_calibration_retained=1",
             reason,accel_calibrator.mask);
}
static void accel_calibration_update(uint32_t now) {
    if (accel_calibrator.phase<UAV_ACCEL_SAVE && (uint32_t)(now-accel_started_ms)>=UAV_ACCEL_CAL_TIMEOUT_MS) {
        accel_calibration_fail(UAV_ACCEL_REASON_TIMEOUT); return;
    }
    if (accel_calibrator.phase==UAV_ACCEL_FIT) {
        int reason=uav_accel_calibrator_fit(&accel_calibrator,&accel_candidate);
        if (reason) {
            if (accel_calibrator.phase==UAV_ACCEL_PLACE)
                uav_logf("WARN","ACC_CAL","recapture face=%u mask=0x%02x reason=unstable hold_still=1",
                         accel_calibrator.target,accel_calibrator.mask);
            else accel_calibration_fail((uint8_t)reason);
            return;
        }
        if (UAV_Write_Param_Accel(&accel_candidate,&accel_save_ticket)) {
            accel_calibration_fail(UAV_ACCEL_REASON_STORAGE); return;
        }
        accel_calibrator.phase=UAV_ACCEL_SAVE;
        uav_logf("INFO","ACC_CAL","fit accepted rms_permille=%lu ticket=%lu awaiting_readback=1",
                 (unsigned long)(accel_candidate.rms_fraction*1000),(unsigned long)accel_save_ticket);
    } else if (accel_calibrator.phase==UAV_ACCEL_SAVE) {
        int result=uav_storage_result(accel_save_ticket);
        if (!result) return;
        if (result<0 || uav_imu_pipeline_set_accel_calibration(&imu_pipeline,accel_candidate.bias,accel_candidate.scale)) {
            accel_calibration_fail(UAV_ACCEL_REASON_STORAGE); return;
        }
        imudata_all.accoffsetbias=(Vector3f_t){accel_candidate.bias[0],accel_candidate.bias[1],accel_candidate.bias[2]};
        imudata_all.accscalebias=(Vector3f_t){accel_candidate.scale[0],accel_candidate.scale[1],accel_candidate.scale[2]};
        imu_frame.dt_s=0; Cold_Start_ARHS(imudata_all,&attitude_t);
        accel_finished_ms=now; accel_calibrator.phase=UAV_ACCEL_DONE;
        if (startup_gyro.failed) accel_restart_gyro=1;
        uav_logf("INFO","ACC_CAL","COMPLETE saved=1 readback=1 applied_before_LPF=1 bias_milli_mps2=%ld,%ld,%ld scale_milli=%ld,%ld,%ld",
                 (long)(accel_candidate.bias[0]*1000),(long)(accel_candidate.bias[1]*1000),(long)(accel_candidate.bias[2]*1000),
                 (long)(accel_candidate.scale[0]*1000),(long)(accel_candidate.scale[1]*1000),(long)(accel_candidate.scale[2]*1000));
    } else if (accel_calibrator.phase==UAV_ACCEL_DONE && (uint32_t)(now-accel_finished_ms)>=1200u) {
        AccelCalFlag=0;
    }
    if ((uint32_t)(now-accel_progress_ms)>=1000u && accel_calibrator.phase<UAV_ACCEL_SAVE) {
        accel_progress_ms=now;
        uav_logf("INFO","ACC_CAL","phase=%u target=%u detected=%u mask=0x%02x collected=%u/200 reason=%u elapsed_ms=%lu",
                 accel_calibrator.phase,accel_calibrator.target,accel_calibrator.detected,accel_calibrator.mask,
                 accel_calibrator.count,accel_calibrator.reason,(unsigned long)(now-accel_started_ms));
    }
}
void IMU_Temperature_Control_Init() // IMU恒温控制初始化
{
    uav_device_heater_init();

    imu_temperature_control_pid_data.ErrorMax = 20;
    imu_temperature_control_pid_data.DifferentialMax = 70;
    imu_temperature_control_pid_data.IntegrateMax = 90;

    imu_temperature_control_pid_data.Kf = 0; // 前馈控制

    imu_temperature_control_pid_data.Kp = 0.01;
    imu_temperature_control_pid_data.Ki = 0;
    imu_temperature_control_pid_data.Kd = 0;
}

void IMU_Temperature_Control(float target) // IMU恒温控制  输入温度
{
    s16 out;
    out = (s16)PID_Control(&imu_temperature_control_pid, &imu_temperature_control_pid_data, 0.05f,
                           0, target, imudata_all.f_temperature, 1000);
    out = out > 999 ? 999 : out;
    out = out < 0 ? 0 : out;
    uav_device_heater_write((uint16_t)out);
    // printf("out:%d\r\n", out);
}

/****************************************************************************************************
 * 函  数：static float invSqrt(float x)
 * 功　能: 快速计算 1/Sqrt(x)
 * 参  数：要计算的值
 * 返回值：计算的结果
 * 备  注：比普通Sqrt()函数要快四倍See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
 *****************************************************************************************************/
float invSqrt(float x) { return isfinite(x) && x > 0.0f ? 1.0f / sqrtf(x) : 0.0f; }

#define Kp                                                                                         \
    6.f // proportional gain governs rate of convergence to accelerometer/magnetometer
        // 比例增益控制加速度计，磁力计的收敛速率
#define Ki                                                                                         \
    0.05f // integral gain governs rate of convergence of gyroscope biases
          // 积分增益控制陀螺偏差的收敛速度
#define Kp_Mag 6.f

#define halfT UPDATE_TIME / 2000.f // half the sample period 采样周期的一半

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0; // scaled integral error


int AHRS_Mahony_Update(_imuData_all imu, _ahrs_data *attitude) {
    const float mag[3]={imu.mag.x,imu.mag.y,imu.mag.z};
    flight_snapshot_t flight; flight_snapshot_read(&flight);
    int mag_permissions=!AK8975Flag ? UAV_FUSION_MAG_AVAILABLE:0;
    if (flight.state==0) mag_permissions|=UAV_FUSION_ALLOW_MAG_RECOVERY;
    if (uav_fusion_step(&imu_fusion,imu_frame.acc,imu_frame.gyro_average,mag,
                        platform_micros(),mag_sample_us,mag_permissions,imu_frame.dt_s)!=0)
        return -1;
    attitude->q0=imu_fusion.q[0]; attitude->q1=imu_fusion.q[1];
    attitude->q2=imu_fusion.q[2]; attitude->q3=imu_fusion.q[3];
    attitude->roll=imu_fusion.roll_deg;
    attitude->pitch=imu_fusion.pitch_deg;
    attitude->yaw=imu_fusion.yaw_deg;
    attitude->rollSpeed=(imu_frame.gyro_control[0]+imu_fusion.gyro_bias[0])*DEG_PER_RAD;
    attitude->pitchSpeed=(imu_frame.gyro_control[1]+imu_fusion.gyro_bias[1])*DEG_PER_RAD;
    attitude->yawSpeed=(imu_frame.gyro_control[2]+imu_fusion.gyro_bias[2])*DEG_PER_RAD;
    return 0;
}

#define allT UPDATE_TIME / 1000.f // half the sample period 采样周期的一半

void AHRS_Kalman_Update(_imuData_all imu, _ahrs_data *attitude) {

    float ax = imu.acc.x;
    float ay = imu.acc.y;
    float az = imu.acc.z;

    float gx = imu.gyro.roll;
    float gy = imu.gyro.pitch;
    float gz = imu.gyro.yaw;

    float v_roll, v_pitch, v_yaw = 0;

    float mbx = imu.mag.x;
    float mby = imu.mag.y;
    float mbz = imu.mag.z;

    float mZx, mZy, mZz = 0;

    float roll_z = atan2f(ay, az), pitch_z = atan2f(-ax, sqrtf(ay * ay + az * az)), yaw_z = 0;

    static float roll_k, pitch_k, yaw_k = 0;
    static float roll_k_, pitch_k_, yaw_k_ = 0;
    static float roll_k_1, pitch_k_1, yaw_k_1 = 0;

    static float p_k_[9] = {0};
    static float p_k_1[9] = {1.f, 0.0f, 0.0f, 0.0f, 1.f, 0.0f, 0.0f, 0.0f, 1.f};
    static float p_k[9] = {1.f, 0.0f, 0.0f, 0.0f, 1.f, 0.0f, 0.0f, 0.0f, 1.f};

    float Kk[9] = {0};
    float Q[9] = {0.0025, 0.0f, 0.0f, 0.0f, 0.0025, 0.0f, 0.0f, 0.0f, 0.0025};
    float R[9] = {0.3, 0.0f, 0.0f, 0.0f, 0.3, 0.0f, 0.0f, 0.0f, 0.3}; // 观测噪声协方差矩阵
    // step1 - system input
    mZx = cos(pitch_z) * mbx + sin(pitch_z) * sin(roll_z) * mby +
          sin(pitch_z) * cos(roll_z) * mbz; // 先绕roll 再绕pitch
    mZy = cos(roll_z) * mby - sin(roll_z) * mbz;
    //  mZz = -sin(pitch_k) * mbx + cos(pitch_k) * sin(roll_k) * mby +
    //								cos(pitch_k) * cos(roll_k) * mbz;

    v_roll = gx - ((sin(pitch_k) * sin(roll_k)) / cos(pitch_k)) * gy +
             ((cos(roll_k) * sin(pitch_k)) / cos(pitch_k)) * gz;
    v_pitch = gy * cos(roll_k) - gz * sin(roll_k);
    v_yaw = gy * sin(roll_k) / cos(pitch_k) + gz * cos(roll_k) / cos(pitch_k);

    roll_k_ = roll_k_1 + allT * v_roll;
    pitch_k_ = pitch_k_1 + allT * v_pitch;
    yaw_k_ = yaw_k_1 + allT * v_yaw;

    if (yaw_k_ > PI)
        yaw_k_ -= 2.0f * PI;
    else if (yaw_k_ < -PI)
        yaw_k_ += 2.0f * PI;

    // step2 - Prior estimation
    p_k_[0] = p_k_1[0] + Q[0];
    p_k_[4] = p_k_1[4] + Q[4];
    p_k_[8] = p_k_1[8] + Q[8];

    // step3 - Prior estimation error covariance
    // step4 - kalman gain
    Kk[0] = p_k_[0] / (p_k_[0] + R[0]);
    Kk[4] = p_k_[4] / (p_k_[4] + R[4]);
    Kk[8] = p_k_[8] / (p_k_[8] + R[8]);

    //	roll_z = atan((ay) / (az));
    //  pitch_z = -1 * atan((ax) / sqrt(ay * ay  + az * az));
    roll_z = atan2(ay, az);
    pitch_z = atan2(-ax, sqrt(ay * ay + az * az));
    yaw_z = atan2(mZy, mZx);

    roll_k = roll_k_ + Kk[0] * (roll_z - roll_k_);
    pitch_k = pitch_k_ + Kk[4] * (pitch_z - pitch_k_);
    yaw_k = yaw_k_ + Kk[8] * (yaw_z - yaw_k_);
    // step5 - measure data
    // printf("%f % f %f \r\n", Zk[0] ,Zk[1],  Zk[2] );
    p_k[0] = (1 - Kk[0]) * p_k_[0];
    p_k[4] = (1 - Kk[4]) * p_k_[4];
    p_k[8] = (1 - Kk[8]) * p_k_[8];

    p_k_1[0] = p_k[0];
    p_k_1[4] = p_k[4];
    p_k_1[8] = p_k[8];

    roll_k_1 = roll_k;
    pitch_k_1 = pitch_k;
    yaw_k_1 = yaw_k;

    // step6 - Posterior estimation
    // step7 - Posteriori estimation error covariance

    attitude->yaw = yaw_k * SEC2DEG;
    attitude->pitch = pitch_k * SEC2DEG; // T13
    attitude->roll = roll_k * SEC2DEG;   // T23/T33

    // calculate the angle,unit: degree
}

void Cold_Start_ARHS(_imuData_all imu, _ahrs_data *attitude) {
    (void)imu;
    uav_fusion_init(&imu_fusion,&uav_board_fusion);
    attitude->q0=1; attitude->q1=attitude->q2=attitude->q3=0;
}
