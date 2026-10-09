#ifndef UAV_SENSOR_PORT_H
#define UAV_SENSOR_PORT_H
#include <stdint.h>
/* SPI2 is exclusively owned by the sampling task. Blocking I/O bounds: 2ms/call.
 * Device index: 0 BMI accel, 1 BMI gyro, 2 magnetometer, 3 pressure. */
void uav_sensor_select(unsigned device, int selected);
/* IDs observed during the existing driver transactions; no extra SPI traffic. */
void uav_sensor_id_snapshot(uint8_t ids[4], uint8_t *seen);
/* Read each XYZ vector as one coherent burst; buffers are owned until return.
 * Application profile must remain +/-3g and +/-500deg/s. No repeated range reads. */
int uav_sensor_read_imu(float acc[3], float gyro[3]);
int uav_sensor_read_temperature(float *temperature);
typedef struct {
    uint32_t reads, errors;
    int16_t raw_signed;
    uint8_t id, msb, lsb, hal_status, valid;
} uav_temperature_io_t;
/* Cached bytes from the normal temperature transaction; adds no SPI traffic. */
void uav_sensor_temperature_io(uav_temperature_io_t *out);
typedef struct {
    uint32_t cr1, ccmr1, ccer;
    uint16_t ccr, arr, psc;
    uint8_t pb8_high, gpio_mode, gpio_af;
} uav_heater_io_t;
/* Read-only timer/GPIO snapshot. Pin level is digital, not a current measurement. */
void uav_device_heater_io(uav_heater_io_t *out);
/* AK8975 single conversion: 1=fresh, 0=pending, -1=bus/status failure. */
int uav_sensor_read_mag(float mag[3]);
void uav_sensor_mag_status(uint8_t *st1, uint8_t *st2);
void uav_sensor_tx(const uint8_t *bytes, uint16_t size);
void uav_sensor_rx(uint8_t *bytes, uint16_t size);
uint8_t uav_sensor_byte(uint8_t byte);
void uav_sensor_delay_ms(uint32_t ms);
void uav_device_key_scan(uint8_t *key);
void uav_device_led_write(uint8_t bits);
void uav_device_heater_init(void);
void uav_device_heater_write(uint16_t value);
void uav_device_delay_us(uint32_t us);
#endif
