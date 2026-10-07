#ifndef __AHRS_H__
#define __AHRS_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "timers.h"
#include "parameter.h"
#include "SPL06.h"
#include "AK8975.h"
#include "BMI088_1.h"
#include "math.h"
//==引用

//==定义

typedef struct {
    //
    u8 data_sta; // 0不可用  1可用
    float IEM[3][3];
    //
    vec3_f f_gyrRaw;
    vec3_f f_accRaw;
    //
    vec3_f f_gyr_dps;
    vec3_f f_acc_cmpss;
    //
    vec3_f f_gyr_dps_nb;
    vec3_f f_gyr_radps_nb;
    vec3_f f_acc_cmpss_nb;
    //
    float f_temperature;
    vec3_f gyrSensitivity;
    vec3_f accSensitivity;
    //

} _imuData_st;

typedef struct {
    acc_raw_data_t acc;
    Vector3f_t accoffsetbias; // 加速度计零偏误差
    Vector3f_t accscalebias;  // 加速度计刻度误差

    gyro_raw_data_t gyro;
    Vector3f_t gyrooffsetbias; // 陀螺仪零偏误差
    Vector3f_t gyroscalebias;  // 陀螺仪刻度误差

    mag_raw_data_t mag;
    Vector3f_t magoffsetbias; // 磁力计零偏误差
    Vector3f_t magscalebias;  // 磁力计刻度误差

    float f_temperature;
    float Pressure;
    float Hight;
} _imuData_all;

typedef struct _AHRS_DATA {
    float roll;
    float pitch;
    float yaw;

    float rollSpeed;
    float pitchSpeed;
    float yawSpeed;

    float q0;
    float q1;
    float q2;
    float q3;

} _ahrs_data;

typedef struct {
    u8 test_u8[4];
    float test[3];
} __attribute__((__packed__)) _test_st;
extern _test_st test_st;

//==数据声明
extern _imuData_st st_imuData;

//==函数声明

// static

// public

void AHRS_Uart4_IDLE_Proc(u8 size);

void Sensors_Update();
void Sensors_Init();
/*IMU传感器灵敏度初始化*/
void ImuSensitivityInit(u8 ins_calibrated, vec3_f accRefValue);
/*IMU传感器数据获取*/
void ImuDataGet(vec3_s16 gyrRaw, vec3_s16 accRaw);
/*IMU温度获取*/
void ImuTemperatureGet(float f_temperature);
/*IMU数据计算处理*/
void ImuDataCalcu(u8 ins_calibrated, vec3_f gyrOffset, vec3_f accOffset, float IEM[3][3]);

int AHRS_Mahony_Update(_imuData_all imu, _ahrs_data *attitude);
void AHRS_Kalman_Update(_imuData_all imu, _ahrs_data *attitude);

void Mag_Zero_Offset_Calibration(_imuData_all *imu);

void SensorData_Task_Proc(void const *argument);

void IMU_Temperature_Control_Init(); // IMU恒温控制初始化

void IMU_Temperature_Control(float target); // IMU恒温控制  输入温度

float invSqrt(float x);
float DATA_Trans(u8 Data_1, u8 Data_2, u8 Data_3, u8 Data_4);

void Cold_Start_ARHS(_imuData_all imu, _ahrs_data *attitude);

void sensors_request_mag_calibration(void);
uint8_t sensors_mag_calibration_active(void);
void sensor_snapshot_read(_imuData_all *out);
uint8_t sensor_calibration_step(void);
uint8_t sensor_imu_calibrating(void);
uint8_t sensor_imu_calibration_failed(void);
typedef struct {
    uint32_t samples, invalid_samples, read_errors, timing_resets, filter_resets;
    uint32_t dt_min_us, dt_max_us, accel_rejected, mag_rejected;
    float sample_hz;
    uint16_t cal_samples;
    uint8_t cal_failed, mag_used;
} sensor_processing_stats_t;
void sensor_processing_stats_read(sensor_processing_stats_t *out);
#endif
