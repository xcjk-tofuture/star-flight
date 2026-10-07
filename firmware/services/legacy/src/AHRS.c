#include "sensor_port.h"
#include "AHRS.h"
#include "attitude.h"
#include "flight_snapshot.h"
#include "log_service.h"
#include "platform_time.h"

#include "lowPassFilter.h"
#include "matrix6.h"
#include "pid.h"
#include "tim.h"
#include "stdio.h"

#define RAD_PER_DEG 0.017453293f
#define DEG_PER_RAD 57.29577951f

#define EXTERN_IMU 0

#define SENSORS_ENABLE_SPL06 1;
#define UPDATE_TIME 5
#define UPDATE_TIME_MAG 20

#define CALIBRATION_COUNT 500 // 决定用多少个值去做校准

extern void UAV_Read_Param_IMU(_imuData_all *imu_data);
extern void UAV_Write_Param_IMU(_imuData_all imu_data);

extern u8 uart4RX[200];

float gyroCalibration[CALIBRATION_COUNT];

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
static u8 AccCalFlag = 1;  // 传感器校准标准位
static u8 GyroCalFlag = 1; // 传感器校准标准位

static u8 MagCalFlag = 0;
uint8_t sensor_imu_calibrating(void) {
    taskENTER_CRITICAL();
    uint8_t active = AccCalFlag || GyroCalFlag;
    taskEXIT_CRITICAL();
    return active;
}
static uint8_t mag_request;
void sensors_request_mag_calibration(void) {
    taskENTER_CRITICAL();
    mag_request = 1;
    taskEXIT_CRITICAL();
}
uint8_t sensors_mag_calibration_active(void) { return MagCalFlag; } // 传感器校准标准位
u8 Bmi088Init_Flag = 1;
u8 AK8975Flag = 1;
u8 SPL06Flag = 1;
u8 IMUTemperatureFlag = 1;

u32 sensorTimeCount = 0;
void Sensor_Data_Task_Proc(void const *argument) {
    osDelay(1000);
#if !EXTERN_IMU
    Sensors_Init(); // 传感器初始化
#else
#ifdef SENSORS_ENABLE_SPL06
    SPL06Flag = Drv_Spl0601_Init();
#endif
#endif

    UAV_Read_Param_IMU(&imudata_all);
    static TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    uint32_t last_mag_id_retry = platform_millis();
    uint8_t mag_id_retries = 0;

    for (;;) {

        if ((TickType_t)(xTaskGetTickCount() - xLastWakeTime) > pdMS_TO_TICKS(5)) {
            xLastWakeTime = xTaskGetTickCount();
            flight_attitude_invalidate();
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1)); // 绝对延时
        sensorTimeCount++;
        taskENTER_CRITICAL();
        uint8_t request = mag_request;
        mag_request = 0;
        taskEXIT_CRITICAL();
        if (request && !AK8975Flag) {
            MagCalFlag = 1;
            flight_attitude_invalidate();
        } else if (request)
            uav_logf("WARN", "SENSOR", "magnetic calibration rejected: magnetometer initialization failed");
#if !EXTERN_IMU

        if (sensorTimeCount % 2 == 0) {
            ReadAccTemperature(&imudata_all.f_temperature);
            ReadAccData(&test_acc);
            ReadGyroData(&test_gyro);
        }
#endif
        if (sensorTimeCount % UPDATE_TIME == 0) {
            IMU_Temperature_Control(40);
            imudata_all.Pressure = Drv_SPl0601_Read();
            // Spl0601Get(&imudata_all.Hight);
#if !EXTERN_IMU

            IMU_Update(test_acc, test_gyro, test_mag, &imudata_all);
            taskENTER_CRITICAL();
            published_sensors = imudata_all;
            taskEXIT_CRITICAL();
            /* BMI088 is required for attitude. Optional sensor failures retain
             * their diagnostic flags without blocking six-axis operation. */
            if (!(AccCalFlag || GyroCalFlag || MagCalFlag || Bmi088Init_Flag)) {
                // AHRS_Kalman_Update(imudata_all, &attitude_t);
                if (AHRS_Mahony_Update(imudata_all, &attitude_t) == 0)
                    flight_attitude_publish(attitude_t.roll, attitude_t.pitch, attitude_t.yaw,
                                            attitude_t.rollSpeed, attitude_t.pitchSpeed,
                                            attitude_t.yawSpeed);
                else
                    flight_attitude_invalidate();
            } else if (MagCalFlag == 1) {
                Mag_Zero_Offset_Calibration(&imudata_all);
            } else {
                // printf("CALLING... \r\n");
                flight_attitude_invalidate();
                Sensor_Calibration(&imudata_all);
                if (sensorTimeCount == CALIBRATION_COUNT * UPDATE_TIME)
                    Cold_Start_ARHS(imudata_all, &attitude_t);
            }
#endif
        }

        if (sensorTimeCount % UPDATE_TIME_MAG == 0) {
#if !EXTERN_IMU
            ReadMagData(&test_mag);
            /* The first boot ID read can fail while later conversions work.
             * Recheck on this SPI2 owner after normal read/trigger cycles have
             * started. Never permanently latch a recoverable first-read error. */
            uint32_t mag_now = platform_millis();
            if (AK8975Flag && mag_id_retries < 4 &&
                (uint32_t)(mag_now - last_mag_id_retry) >= 500u) {
                last_mag_id_retry = mag_now;
                mag_id_retries++;
                AK8975Flag = DrvAK8975Check();
                SensorError = Bmi088Init_Flag || AK8975Flag;
                uav_logf(AK8975Flag ? "WARN" : "INFO", "MAG_RECHECK",
                         "attempt=%u/4 rc=%u attitude_mode=%s",
                         (unsigned)mag_id_retries, (unsigned)AK8975Flag,
                         Bmi088Init_Flag ? "BLOCKED" : AK8975Flag ? "6AXIS" : "9AXIS");
                if (!AK8975Flag)
                    uav_logf("INFO", "AHRS", "magnetometer ID recovered; magnetic reference enabled");
            }
#endif
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

void Sensor_Calibration(_imuData_all *imu) // 传感器校准
{
    if (GyroCalFlag)
        Simple_Zero_Offset_Calibration(imu, &(imu->gyrooffsetbias)); // 简单零偏误差校准
    if (AccCalFlag)
        Acc_LMS_Calibration(imu, &(imu->accoffsetbias), &(imu->accscalebias));
}

static u8 magCalistep = 0;
uint8_t sensor_calibration_step(void) {
    taskENTER_CRITICAL();
    uint8_t step = magCalistep;
    taskEXIT_CRITICAL();
    return step;
}
void Mag_Zero_Offset_Calibration(_imuData_all *imu) {
    static float gyroRoll, gyroPitch, gyroYaw;
    static float magXMax, magYMax, magZMax;
    static float magXMin, magYMin, magZMin;
    switch (magCalistep) {
    case 0:
        gyroRoll += imu->gyro.roll * (UPDATE_TIME / 1000.f);
        magZMax = magZMax > imu->mag.z ? magZMax : imu->mag.z;
        magZMin = magZMin < imu->mag.z ? magZMin : imu->mag.z;

        if (gyroRoll >= PI * 2.2f || gyroRoll <= PI * -2.2f)
            magCalistep = 1;
        break;
    case 1:
        imu->magoffsetbias.z = (magZMax + magZMin) / 2;
        gyroPitch += imu->gyro.pitch * (UPDATE_TIME / 1000.f);
        magZMax = magZMax > imu->mag.z ? magZMax : imu->mag.z;
        magZMin = magZMin < imu->mag.z ? magZMin : imu->mag.z;

        if (gyroPitch >= PI * 2.2f || gyroPitch <= PI * -2.2f)
            magCalistep = 2;
        break;
    case 2:
        imu->magoffsetbias.z = (magZMax + magZMin) / 2;
        gyroYaw += imu->gyro.yaw * (UPDATE_TIME / 1000.f);
        magXMax = magXMax > imu->mag.x ? magXMax : imu->mag.x;
        magXMin = magXMin < imu->mag.x ? magXMin : imu->mag.x;
        magYMax = magYMax > imu->mag.y ? magYMax : imu->mag.y;
        magYMin = magYMin < imu->mag.y ? magYMin : imu->mag.y;
        if ((gyroYaw) >= PI * 2.2f || (gyroYaw) <= PI * -2.2f)
            magCalistep = 3;
        break;
    case 3:
        imu->magoffsetbias.x = (magXMax + magXMin) / 2;
        imu->magoffsetbias.y = (magYMax + magYMin) / 2;
        gyroRoll = 0;
        gyroPitch = 0;
        gyroYaw = 0;
        UAV_Write_Param_IMU(*imu); // 写入数据
        osDelay(1000);
        magCalistep = 4;
        break;
    case 4:
        osDelay(1000);
        magCalistep = 0;
        MagCalFlag = 0;
        break;
    }
}

void Simple_Zero_Offset_Calibration(_imuData_all *imu, Vector3f_t *offset) // 陀螺仪零偏校准
{
    static float gyroBias[3] = {0.0f};
    static int i = 0;

    gyroBias[0] += imu->gyro.roll;
    gyroBias[1] += imu->gyro.pitch;
    gyroBias[2] += imu->gyro.yaw;

    i++;
    if (i >= CALIBRATION_COUNT) {

        if (gyroBias[0] >= 500 || gyroBias[1] >= 500 || gyroBias[2] >= 500) // 陀螺仪存在运动状态
        {
            i = 0;
            gyroBias[0] = 0;
            gyroBias[1] = 0;
            gyroBias[2] = 0;
        } else {
            gyroBias[0] /= i;
            gyroBias[1] /= i;
            gyroBias[2] /= i;

            offset->x = gyroBias[0];
            offset->y = gyroBias[1];
            offset->z = gyroBias[2];
            // printf("%d\r\n",i);
            GyroCalFlag = 0;
        }
    }
}

float raw[6][3];
void Acc_LMS_Calibration(_imuData_all *imu, Vector3f_t *offset,
                         Vector3f_t *scale) // 传感器广义椭球校准
{
    osDelay(200);
    static int i = 0;
    raw[i][0] = imu->acc.x;
    raw[i][1] = imu->acc.y;
    raw[i][2] = imu->acc.z;
    i++;
    if (i == 6) {
        // LMS_Fitting(raw, offset, scale);
        AccCalFlag = 0;
    }
}

void LMS_Fitting(float raw[6][3], Vector3f_t *offset, Vector3f_t *scale) // 用于椭球拟合加速度拟合
{
    float x[6], y[6], z[6];
    float m[6][6];
    float m_t[6][6];
    float m_txm_inv[6][6];
    float m_txm_invxm_t[6][6];
    float m_txm[6][6];
    float p[6];
    double v[7] = {0};
    float x0, y0, z0, A, B, C;

    for (int i = 0; i <= 5; i++) {
        x[i] = raw[i][0];
        y[i] = raw[i][1];
        z[i] = raw[i][2];
    }
    for (int i = 0; i < 6; i++)
        p[i] = -(x[i] * x[i]); // p矩阵
    for (int i = 0; i <= 5; i++) {
        m[i][0] = y[i] * y[i];
        m[i][1] = z[i] * z[i];
        m[i][2] = x[i];
        m[i][3] = y[i];
        m[i][4] = z[i];
        m[i][5] = 1.0f; // m矩阵
    }

    Matrix6_Tran(m, m_t); // 求m的转置
    Matrix6_Mul(m_t, m, m_txm);
    if (!Matrix6_Det(m_txm, m_txm_inv))
        return;
    Matrix6_Mul(m_txm_inv, m_t, m_txm_invxm_t);

    for (int i = 0; i < 6; i++) {
        v[0] += (m_txm_invxm_t[0][i] * p[i]);
        v[1] += (m_txm_invxm_t[1][i] * p[i]);
        v[2] += (m_txm_invxm_t[2][i] * p[i]);
        v[3] += (m_txm_invxm_t[3][i] * p[i]);
        v[4] += (m_txm_invxm_t[4][i] * p[i]);
        v[5] += (m_txm_invxm_t[5][i] * p[i]);

        // printf("%f\n",m_txm_invxm_t[5][i] * p[i]);
    }
    x0 = -v[2] / 2;
    y0 = -v[3] / (2 * v[0]);
    z0 = -v[4] / (2 * v[1]);
    A = sqrt(x0 * x0 + v[1] * y0 * y0 + v[1] * z0 * z0 - v[5]);
    B = A * invSqrt(v[0]);
    C = A * invSqrt(v[1]);

    offset->x = x0;
    offset->y = y0;
    offset->z = z0;

    scale->x = A;
    scale->y = B;
    scale->z = C;
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
    out = (s16)PID_Control(&imu_temperature_control_pid, &imu_temperature_control_pid_data, 0.005f,
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

void IMU_Update(acc_raw_data_t acc, gyro_raw_data_t gyro, mag_raw_data_t mag, _imuData_all *imu) {
    //	imu->acc.x =imu->acc.x * (1 - 0.9) + acc.x * 0.9;
    //	imu->acc.y =imu->acc.y * (1 - 0.9) + acc.y * 0.9;
    //	imu->acc.z =imu->acc.z * (1 - 0.9) + acc.z * 0.9; //一阶低通滤波33

    //	gyro.roll = gyro.roll - imu->gyrooffsetbias.x;
    //	gyro.pitch = gyro.pitch - imu->gyrooffsetbias.y;
    //	gyro.yaw = gyro.yaw - imu->gyrooffsetbias.z;

    //	imu->gyro.pitch =imu->gyro.pitch * (1 - 0.3) +  gyro.pitch * 0.3;
    //	imu-> gyro.roll =imu-> gyro.roll * (1 - 0.3) +  gyro.roll * 0.3;
    //	imu-> gyro.yaw =imu-> gyro.yaw * (1 - 0.3) +  gyro.yaw * 0.3; //一阶低通滤波33

    imu->acc.x = acc.x;
    imu->acc.y = acc.y;
    imu->acc.z = acc.z;

    static LPF2ndData_t LPF2_GYRO;
    Vector3f_t LPF2_GYRO_Data;

    static LPF2ndData_t LPF2_ACC;
    Vector3f_t LPF2_ACC_Data;

    static LPF2ndData_t LPF2_MAG;
    Vector3f_t LPF2_MAG_Data;

    LPF2_GYRO_Data.x = gyro.roll - imu->gyrooffsetbias.x;
    LPF2_GYRO_Data.y = gyro.pitch - imu->gyrooffsetbias.y;
    LPF2_GYRO_Data.z = gyro.yaw - imu->gyrooffsetbias.z;

    LPF2_ACC_Data.x = acc.x;
    LPF2_ACC_Data.y = acc.y;
    LPF2_ACC_Data.z = acc.z;

    LowPassFilter2ndFactorCal(UPDATE_TIME / 1000.0f, 51, &LPF2_GYRO);
    LPF2_GYRO_Data = LowPassFilter2nd(&LPF2_GYRO, LPF2_GYRO_Data); // 陀螺仪二阶低通滤波
                                                                   // 截止频率50HZ

    LowPassFilter2ndFactorCal(UPDATE_TIME / 1000.0f, 51, &LPF2_ACC);
    LPF2_ACC_Data = LowPassFilter2nd(&LPF2_ACC, LPF2_ACC_Data); // 陀螺仪二阶低通滤波 截止频率50HZ

    imu->gyro.pitch = LPF2_GYRO_Data.y;
    imu->gyro.roll = LPF2_GYRO_Data.x;
    imu->gyro.yaw = LPF2_GYRO_Data.z; // 减去零偏误差

    imu->acc.x = LPF2_ACC_Data.x;
    imu->acc.y = LPF2_ACC_Data.y;
    imu->acc.z = LPF2_ACC_Data.z; // 减去零偏误差

    imu->mag.x = mag.x - imu->magoffsetbias.x;
    imu->mag.y = mag.y - imu->magoffsetbias.y;
    imu->mag.z = mag.z - imu->magoffsetbias.z;
}

int AHRS_Mahony_Update(_imuData_all imu, _ahrs_data *attitude) {
    static uav_attitude_t filter;
    static uint8_t initialized;
    const float acc[3] = {imu.acc.x, imu.acc.y, imu.acc.z};
    const float gyro[3] = {imu.gyro.roll, imu.gyro.pitch, imu.gyro.yaw};
    const float mag[3] = {AK8975Flag ? 0.0f : imu.mag.x,
                         AK8975Flag ? 0.0f : imu.mag.y,
                         AK8975Flag ? 0.0f : imu.mag.z};
    if (!initialized) {
        uav_attitude_init(&filter);
        initialized = 1;
    }
    filter.q[0] = attitude->q0;
    filter.q[1] = attitude->q1;
    filter.q[2] = attitude->q2;
    filter.q[3] = attitude->q3;
    if (filter.q[0] * filter.q[0] + filter.q[1] * filter.q[1] + filter.q[2] * filter.q[2] +
            filter.q[3] * filter.q[3] <
        1e-12f)
        uav_attitude_init(&filter);
    if (uav_attitude_step(&filter, acc, gyro, mag, UPDATE_TIME / 1000.0f) != 0)
        return -1;
    attitude->q0 = filter.q[0];
    attitude->q1 = filter.q[1];
    attitude->q2 = filter.q[2];
    attitude->q3 = filter.q[3];
    attitude->roll = filter.roll_deg;
    attitude->pitch = filter.pitch_deg;
    attitude->yaw = filter.yaw_deg;
    attitude->rollSpeed = imu.gyro.roll * DEG_PER_RAD;
    attitude->pitchSpeed = imu.gyro.pitch * DEG_PER_RAD;
    attitude->yawSpeed = imu.gyro.yaw * DEG_PER_RAD;
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
    float roll, pitch, yaw = 0;

    float ax = imu.acc.x;
    float ay = imu.acc.y;
    float az = imu.acc.z;

    float gx = imu.gyro.roll;
    float gy = imu.gyro.pitch;
    float gz = imu.gyro.yaw;

    float mbx = imu.mag.x;
    float mby = imu.mag.y;
    float mbz = imu.mag.z;

    float mZx, mZy, mZz = 0;

    roll = atan((ay) / (az));
    pitch = -1 * atan((ax) / sqrt(ay * ay + az * az));

    mZx = cos(pitch) * mbx + sin(pitch) * sin(roll) * mby +
          sin(pitch) * cos(roll) * mbz; // 先绕roll 再绕pitch
    mZy = cos(roll) * mby - sin(roll) * mbz;

    yaw = atan(mZy / mZx);

    //	attitude->yaw = -yaw * SEC2DEG;
    //	attitude->pitch = pitch * SEC2DEG;   //T13
    //	attitude->roll = roll * SEC2DEG;  // T23/T33

    // 将角度转换为弧度
    double half_roll = roll * 0.5;
    double half_pitch = pitch * 0.5;
    double half_yaw = yaw * 0.5;

    // 计算三角函数值
    double sin_r = sin(half_roll);
    double cos_r = cos(half_roll);
    double sin_p = sin(half_pitch);
    double cos_p = cos(half_pitch);
    double sin_y = sin(half_yaw);
    double cos_y = cos(half_yaw);

    // 计算四元数
    //	attitude->q0 = cos_r * cos_p * cos_y - sin_r * sin_p * sin_y;
    //	attitude->q1 = sin_r * cos_p * cos_y + cos_r * sin_p * sin_y;
    //	attitude->q2 = cos_r * sin_p * cos_y - sin_r * cos_p * sin_y;
    //	attitude->q3 = cos_r * cos_p * sin_y + sin_r * sin_p * cos_y;
    attitude->q0 = 1;
    attitude->q1 = 0;
    attitude->q2 = 0;
    attitude->q3 = 0;

    //	// 计算四元数
    //	attitude->q0 = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y;
    //	attitude->q1  = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y;
    //	attitude->q2  = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y;
    //	attitude->q3  = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y;
}
