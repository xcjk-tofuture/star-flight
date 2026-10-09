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
static uint8_t mag_pending, mag_st1, mag_st2;
static uint32_t mag_trigger_ms;
static uav_temperature_io_t temperature_io;
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
static HAL_StatusTypeDef temperature_register_read(uint8_t address, uint8_t *value) {
    /* Same ACC read protocol as the legacy driver: command, dummy, data.
     * Bounded HAL transactions avoid the legacy fatal-error byte wrapper. */
    uint8_t tx[3]={(uint8_t)(address|0x80u),0xff,0xff},rx[3]={0};
    uav_sensor_select(0,1);
    HAL_StatusTypeDef status=HAL_SPI_TransmitReceive(&hspi2,tx,rx,sizeof(rx),2);
    uav_sensor_select(0,0);
    if (status==HAL_OK) *value=rx[2];
    return status;
}
int uav_sensor_read_temperature(float *temperature) {
    if (!temperature) return -1;
    uav_temperature_io_t observed=temperature_io;
    observed.reads++; observed.valid=0; observed.id=observed.msb=observed.lsb=0; observed.raw_signed=0;
    uint8_t id_tx[3]={0x80},id_rx[3]={0},tx[4]={0xa2},rx[4]={0};
    uav_sensor_select(0,1);
    HAL_StatusTypeDef status=HAL_SPI_TransmitReceive(&hspi2,id_tx,id_rx,sizeof(id_rx),2);
    uav_sensor_select(0,0);
    observed.id=id_rx[2];
    if (status!=HAL_OK || id_rx[2]!=0x1e) goto complete;
    uav_sensor_select(0,1);
    status=HAL_SPI_TransmitReceive(&hspi2,tx,rx,sizeof(rx),2);
    uav_sensor_select(0,0);
    observed.msb=rx[2]; observed.lsb=rx[3];
    if (status!=HAL_OK) goto complete;
    int raw=(rx[2]<<3)|(rx[3]>>5);
    if (raw>1023) raw-=2048;
    observed.raw_signed=(int16_t)raw;
    *temperature=raw*.125f+23.0f;
    observed.valid=(uint8_t)(*temperature>=-40 && *temperature<=85);
    if (observed.valid && observed.reads%10u==0) {
        /* Compare once per second, not on the 500Hz inertial sampling path.
         * Re-read MSB to reject a pair torn by the device's temperature update. */
        uint8_t msb=0,lsb=0,msb_again=0;
        HAL_StatusTypeDef check=temperature_register_read(0x22,&msb);
        if (check==HAL_OK) check=temperature_register_read(0x23,&lsb);
        if (check==HAL_OK) check=temperature_register_read(0x22,&msb_again);
        observed.check_read=observed.reads; observed.check_msb=msb; observed.check_lsb=lsb;
        observed.check_burst_raw_signed=observed.raw_signed;
        observed.check_hal=(uint8_t)check;
        observed.check_valid=(uint8_t)(check==HAL_OK && msb==msb_again);
        int alternate=(msb<<3)|(lsb>>5); if (alternate>1023) alternate-=2048;
        observed.check_raw_signed=(int16_t)alternate;
    }
complete:
    observed.hal_status=(uint8_t)status;
    if (!observed.valid) observed.errors++;
    taskENTER_CRITICAL(); temperature_io=observed; taskEXIT_CRITICAL();
    return observed.valid ? 0:-1;
}
void uav_sensor_temperature_io(uav_temperature_io_t *out) {
    taskENTER_CRITICAL(); *out=temperature_io; taskEXIT_CRITICAL();
}
static int magnetic_transaction(uint8_t *tx, uint8_t *rx, uint16_t count) {
    uav_sensor_select(2,1);
    HAL_StatusTypeDef status=HAL_SPI_TransmitReceive(&hspi2,tx,rx,count,2);
    uav_sensor_select(2,0);
    return status==HAL_OK ? 0:-1;
}
static int magnetic_trigger(void) {
    uint8_t tx[2]={0x0a,0x01},rx[2];
    if (magnetic_transaction(tx,rx,2)) { mag_pending=0; return -1; }
    mag_trigger_ms=HAL_GetTick(); mag_pending=1;
    return 0;
}
void uav_sensor_mag_status(uint8_t *st1, uint8_t *st2) {
    if (st1) *st1=mag_st1;
    if (st2) *st2=mag_st2;
}
int uav_sensor_read_mag(float mag[3]) {
    if (!mag) return -1;
    if (!mag_pending) return magnetic_trigger() ? -1:0;
    uint8_t status_tx[2]={0x82,0},status_rx[2];
    if (magnetic_transaction(status_tx,status_rx,2)) { mag_pending=0; return -1; }
    mag_st1=status_rx[1];
    if (!(mag_st1&1u)) {
        /* Conversion is polled without blocking the inertial loop. */
        if ((uint32_t)(HAL_GetTick()-mag_trigger_ms)>100u) {
            magnetic_trigger(); return -1;
        }
        return 0;
    }
    uint8_t tx[8]={0x83},rx[8];
    if (magnetic_transaction(tx,rx,8)) { mag_pending=0; return -1; }
    mag_st2=rx[7]; /* Final status read also releases the sensor data latch. */
    float field[3];
    int result=uav_imu_decode_ak8975(rx+1,field);
    mag_pending=0;
    if (magnetic_trigger()) return -1;
    if (result) return -1;
    for (unsigned i=0;i<3;i++) mag[i]=field[i];
    return 1;
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
    __HAL_TIM_SET_COMPARE(&UAV_HEATER_TIMER, TIM_CHANNEL_1, 0);
    uint32_t clock=HAL_RCC_GetPCLK2Freq();
    if (clock!=HAL_RCC_GetHCLKFreq()) clock*=2;
    __HAL_TIM_SET_PRESCALER(&UAV_HEATER_TIMER,clock/1000000u-1);
    __HAL_TIM_SET_AUTORELOAD(&UAV_HEATER_TIMER,999);
    UAV_HEATER_TIMER.Instance->EGR=TIM_EGR_UG;
    if (HAL_TIM_PWM_Start(&UAV_HEATER_TIMER, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();
}
void uav_device_heater_write(uint16_t value) {
    __HAL_TIM_SET_COMPARE(&UAV_HEATER_TIMER, TIM_CHANNEL_1, value > 1000 ? 1000 : value);
}
void uav_device_heater_io(uav_heater_io_t *out) {
    taskENTER_CRITICAL();
    *out=(uav_heater_io_t){UAV_HEATER_TIMER.Instance->CR1,UAV_HEATER_TIMER.Instance->CCMR1,
        UAV_HEATER_TIMER.Instance->CCER,(uint16_t)UAV_HEATER_TIMER.Instance->CCR1,
        (uint16_t)UAV_HEATER_TIMER.Instance->ARR,(uint16_t)UAV_HEATER_TIMER.Instance->PSC,
        (uint8_t)!!(GPIOB->IDR & GPIO_PIN_8),(uint8_t)((GPIOB->MODER>>16)&3u),
        (uint8_t)(GPIOB->AFR[1]&15u)};
    taskEXIT_CRITICAL();
}
void uav_device_delay_us(uint32_t us) {
    volatile uint32_t count = (HAL_RCC_GetHCLKFreq() / 4000000) * us;
    while (count--) {
    } /* Same short device wake delays as the original driver. */
}
