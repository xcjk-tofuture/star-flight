#include "sensor_port.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "uav_board.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log_service.h"
#include "imu_sample_decode.h"
#include <string.h>
static uint8_t observed_ids[4], observed_mask;
static unsigned selected_device = 4;
static uint8_t transaction_command, transaction_id, transaction_has_id;
static uint16_t transaction_position;
/* Observe the driver's real read rather than inserting a second ID request.
 * BMI088 accel has one dummy byte; the other three checks do not. */
static void observe_byte(uint8_t tx, uint8_t rx, int received) {
    if (selected_device >= 4)
        return;
    if (!transaction_position)
        transaction_command = tx;
    else if (received && transaction_command == (selected_device == 3 ? 0x8du : 0x80u) &&
             transaction_position == (selected_device == 0 ? 2u : 1u)) {
        transaction_id = rx;
        transaction_has_id = 1;
    }
    transaction_position++;
}
void uav_sensor_id_snapshot(uint8_t ids[4], uint8_t *seen) {
    if (!ids || !seen)
        return;
    taskENTER_CRITICAL();
    memcpy(ids, observed_ids, sizeof(observed_ids));
    *seen = observed_mask;
    taskEXIT_CRITICAL();
}
void uav_sensor_select(unsigned device, int selected) {
    if (device >= 4)
        return;
    GPIO_TypeDef *port = device == 0   ? BMI088_ACC_GPIOx
                         : device == 1 ? BMI088_GYRO_GPIOx
                         : device == 2 ? SPI2_CS3_GPIO_Port
                                       : SPI2_CS2_GPIO_Port;
    uint16_t pin = device == 0   ? BMI088_ACC_GPIOp
                   : device == 1 ? BMI088_GYRO_GPIOp
                   : device == 2 ? SPI2_CS3_Pin
                                 : SPI2_CS2_Pin;
    if (selected) {
        selected_device = device;
        transaction_position = 0;
        transaction_has_id = 0;
    }
    HAL_GPIO_WritePin(port, pin, selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
    if (!selected && selected_device == device) {
        selected_device = 4;
        if (transaction_has_id) {
            static const char *const names[] = {"BMI088_ACC", "BMI088_GYRO", "AK8975", "SPL06"};
            static const char *const pins[] = {"PE15", "PD8", "PD10", "PD9"};
            static const uint8_t expected[] = {0x1e, 0x0f, 0x48, 0x10};
            int changed = !(observed_mask & (1u << device)) || observed_ids[device] != transaction_id;
            observed_ids[device] = transaction_id;
            observed_mask |= (uint8_t)(1u << device);
            if (changed)
                uav_logf(transaction_id == expected[device] ? "INFO" : "WARN", "SENSOR_ID",
                         "%s ID=0x%02x expected=0x%02x CS=%s transfer=OK", names[device],
                         (unsigned)transaction_id, (unsigned)expected[device], pins[device]);
        }
    }
}
void uav_sensor_tx(const uint8_t *bytes, uint16_t size) {
    if (HAL_SPI_Transmit(&hspi2, (uint8_t *)bytes, size, 2) != HAL_OK)
        Error_Handler();
    for (uint16_t i = 0; i < size; i++)
        observe_byte(bytes[i], 0, 0);
}
void uav_sensor_rx(uint8_t *bytes, uint16_t size) {
    if (HAL_SPI_Receive(&hspi2, bytes, size, 2) != HAL_OK)
        Error_Handler();
    for (uint16_t i = 0; i < size; i++)
        observe_byte(0xff, bytes[i], 1);
}
uint8_t uav_sensor_byte(uint8_t byte) {
    uint8_t response;
    if (HAL_SPI_TransmitReceive(&hspi2, &byte, &response, 1, 2) != HAL_OK)
        Error_Handler();
    observe_byte(byte, response, 1);
    return response;
}
void uav_sensor_delay_ms(uint32_t ms) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
        HAL_Delay(ms);
    else
        osDelay(ms);
}
int uav_sensor_read_imu(float acc[3], float gyro[3]) {
    if (!acc || !gyro) return -1;
    uint8_t acc_tx[8]={0x92}, acc_rx[8], gyro_tx[7]={0x82}, gyro_rx[7];
    uav_sensor_select(0,1);
    HAL_StatusTypeDef status=HAL_SPI_TransmitReceive(&hspi2,acc_tx,acc_rx,sizeof(acc_rx),2);
    uav_sensor_select(0,0);
    if (status!=HAL_OK) return -1;
    uav_sensor_select(1,1);
    status=HAL_SPI_TransmitReceive(&hspi2,gyro_tx,gyro_rx,sizeof(gyro_rx),2);
    uav_sensor_select(1,0);
    if (status!=HAL_OK) return -1;
    return uav_imu_decode_bmi088(acc_rx,gyro_rx,acc,gyro);
}
void uav_device_key_scan(uint8_t *key) {
    *key = HAL_GPIO_ReadPin(UAV_KEY_PORT, UAV_KEY1_PIN) == GPIO_PIN_RESET   ? 1
           : HAL_GPIO_ReadPin(UAV_KEY_PORT, UAV_KEY2_PIN) == GPIO_PIN_RESET ? 2
                                                                            : 0;
}
void uav_device_led_write(uint8_t bits) {
    HAL_GPIO_WritePin(UAV_LED_PORT, RGB_R_Pin, (bits & 4) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(UAV_LED_PORT, RGB_G_Pin, (bits & 2) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(UAV_LED_PORT, RGB_B_Pin, (bits & 1) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
void uav_device_heater_init(void) {
    if (HAL_TIM_PWM_Start(&UAV_HEATER_TIMER, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();
    __HAL_TIM_SET_AUTORELOAD(&UAV_HEATER_TIMER, 999);
    __HAL_TIM_SET_COMPARE(&UAV_HEATER_TIMER, TIM_CHANNEL_1, 0);
}
void uav_device_heater_write(uint16_t value) {
    __HAL_TIM_SET_COMPARE(&UAV_HEATER_TIMER, TIM_CHANNEL_1, value > 999 ? 999 : value);
}
void uav_device_delay_us(uint32_t us) {
    volatile uint32_t count = (HAL_RCC_GetHCLKFreq() / 4000000) * us;
    while (count--) {
    } /* Same short device wake delays as the original driver. */
}
