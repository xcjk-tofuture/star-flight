#include "boot_log.h"
#include "log_service.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "w25qxx_device.h"
#include "AHRS.h"
#include "flight_snapshot.h"
#include "sensor_port.h"
#include "serial_port.h"
#include "display_service.h"
#include "gui_display_port.h"
#include <math.h>

extern uint8_t SensorError, Bmi088Init_Flag, AK8975Flag, SPL06Flag;
extern void *OLEDTaskHandle, *PCTaskHandle, *SensorDataTaskHandle;
extern char _heap_start, _heap_end, _estack;
static uint8_t flash_ready;
int app_boot_flash_ready(void) { return flash_ready; }

void app_boot_log_hal(void) {
#ifdef NDEBUG
    const char *build = "Release";
#else
    const char *build = "Debug";
#endif
    uav_logf("INFO", "BOOT", "StarFlight %s built=%s %s compiler=%s", build, __DATE__,
             __TIME__, __VERSION__);
    uav_logf("INFO", "MCU", "target=STM32F407 dev=0x%03lx rev=0x%04lx cpuid=0x%08lx flash=%uKiB",
             (unsigned long)HAL_GetDEVID(), (unsigned long)HAL_GetREVID(),
             (unsigned long)SCB->CPUID, (unsigned)*(volatile uint16_t *)0x1fff7a22u);
    uint32_t reset = RCC->CSR;
    uav_logf("INFO", "RESET", "CSR=0x%08lx POR=%u PIN=%u SW=%u IWDG=%u WWDG=%u LOWPOWER=%u",
             (unsigned long)reset, !!(reset & RCC_CSR_PORRSTF), !!(reset & RCC_CSR_PINRSTF),
             !!(reset & RCC_CSR_SFTRSTF), !!(reset & RCC_CSR_IWDGRSTF),
             !!(reset & RCC_CSR_WWDGRSTF), !!(reset & RCC_CSR_LPWRRSTF));
    uav_logf("INFO", "HAL", "init=OK version=0x%08lx tick=%luHz", (unsigned long)HAL_GetHalVersion(),
             (unsigned long)(1000u / (uint32_t)HAL_GetTickFreq()));
}
void app_boot_log_clocks(void) {
    uint32_t pll = RCC->PLLCFGR;
    uav_logf("INFO", "CLOCK", "SYS=%luHz HCLK=%luHz PCLK1=%luHz PCLK2=%luHz",
             (unsigned long)HAL_RCC_GetSysClockFreq(), (unsigned long)HAL_RCC_GetHCLKFreq(),
             (unsigned long)HAL_RCC_GetPCLK1Freq(), (unsigned long)HAL_RCC_GetPCLK2Freq());
    uav_logf("INFO", "PLL", "source=%s HSE_VALUE=%luHz M=%lu N=%lu P=%lu Q=%lu CR=0x%08lx",
             (pll & RCC_PLLCFGR_PLLSRC) ? "HSE" : "HSI", (unsigned long)HSE_VALUE,
             (unsigned long)(pll & 0x3fu), (unsigned long)((pll >> 6) & 0x1ffu),
             (unsigned long)((((pll >> 16) & 3u) + 1u) * 2u),
             (unsigned long)((pll >> 24) & 15u), (unsigned long)RCC->CR);
}
static unsigned spi2_cs_levels(void) {
    return (HAL_GPIO_ReadPin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin) == GPIO_PIN_SET ? 1u : 0u) |
           (HAL_GPIO_ReadPin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin) == GPIO_PIN_SET ? 2u : 0u) |
           (HAL_GPIO_ReadPin(SPI2_CS3_GPIO_Port, SPI2_CS3_Pin) == GPIO_PIN_SET ? 4u : 0u) |
           (HAL_GPIO_ReadPin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin) == GPIO_PIN_SET ? 8u : 0u);
}
void app_boot_spi_idle(void) {
    unsigned before = spi2_cs_levels();
    /* All active-low selects must be idle before the first device command.
     * Cube GPIO defaults select all devices; change only application startup. */
    HAL_GPIO_WritePin(GPIOC, FLASH_CS_Pin | OLED_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, SPI2_CS1_Pin | SPI2_CS2_Pin | SPI2_CS3_Pin, GPIO_PIN_SET);
    uav_logf("INFO", "SPI_CS", "SPI2 high_mask before=0x%x after=0x%x bit0=ACC_PE15 bit1=GYRO_PD8 bit2=MAG_PD10 bit3=BARO_PD9",
             before, spi2_cs_levels());
    uav_logf("INFO", "SPI_CS", "SPI1 FLASH_PC0=%u OLED_PC4=%u idle=HIGH",
             (unsigned)HAL_GPIO_ReadPin(FLASH_CS_GPIO_Port, FLASH_CS_Pin),
             (unsigned)HAL_GPIO_ReadPin(OLED_CS_GPIO_Port, OLED_CS_Pin));
}
static void uart_parameters(const char *name, UART_HandleTypeDef *uart, const char *pins) {
    unsigned data = uart->Init.WordLength == UART_WORDLENGTH_9B ? 9u : 8u;
    const char *parity = uart->Init.Parity == UART_PARITY_NONE ? "NONE"
                         : uart->Init.Parity == UART_PARITY_EVEN ? "EVEN" : "ODD";
    if (uart->Init.Parity != UART_PARITY_NONE)
        data--;
    uav_logf("INFO", "UART", "%s baud=%lu data=%u parity=%s stop=%s mode=0x%lx BRR=0x%lx %s",
             name, (unsigned long)uart->Init.BaudRate, data, parity,
             uart->Init.StopBits == UART_STOPBITS_1 ? "1" : "2",
             (unsigned long)uart->Init.Mode, (unsigned long)uart->Instance->BRR, pins);
}
static void spi_parameters(const char *name, SPI_HandleTypeDef *spi, uint32_t clock,
                           const char *pins) {
    unsigned divider = 1u << (((spi->Init.BaudRatePrescaler >> 3) & 7u) + 1u);
    unsigned mode = (spi->Init.CLKPolarity == SPI_POLARITY_HIGH ? 2u : 0u) |
                    (spi->Init.CLKPhase == SPI_PHASE_2EDGE ? 1u : 0u);
    uav_logf("INFO", "SPI", "%s clock=%luHz div=%u bus=%luHz mode=%u bits=%u CR1=0x%lx %s",
             name, (unsigned long)clock, divider, (unsigned long)(clock / divider), mode,
             spi->Init.DataSize == SPI_DATASIZE_16BIT ? 16u : 8u,
             (unsigned long)spi->Instance->CR1, pins);
}
void app_boot_log_peripherals(void) {
    uav_logf("INFO", "PERIPH", "GPIO DMA I2C SPI TIM UART USB initialization returned");
    uart_parameters("USART1", &huart1, "TX=PA9 RX=PA10 telemetry+LOG");
    uart_parameters("USART2", &huart2, "TX=PA2 RX=PA3 flow");
    uart_parameters("USART3", &huart3, "TX=PB10 RX=PB11 LOG moved to USART1");
    uart_parameters("UART4", &huart4, "TX=PA0 RX=PA1");
    uart_parameters("UART5", &huart5, "TX=PC12 RX=PD2");
    uart_parameters("USART6", &huart6, "TX=PC6 RX=PC7 SBUS");
    spi_parameters("SPI1", &hspi1, HAL_RCC_GetPCLK2Freq(), "SCK=PA5 MISO=PA6 MOSI=PA7");
    spi_parameters("SPI2", &hspi2, HAL_RCC_GetPCLK1Freq(), "SCK=PB13 MISO=PB14 MOSI=PB15");
    uav_logf("INFO", "BOARD", "FLASH_CS=PC0 OLED_CS=PC4 OLED_RESET=PB1 OLED_DC=PC5");
    uav_logf("INFO", "MEMORY", "RAM=0x20000000..0x%08lx C_heap=0x%08lx..0x%08lx RTOS_heap=%luB",
             (unsigned long)(uintptr_t)&_estack, (unsigned long)(uintptr_t)&_heap_start,
             (unsigned long)(uintptr_t)&_heap_end, (unsigned long)configTOTAL_HEAP_SIZE);
}
void app_boot_log_flash(int result) {
    flash_ready = result == 0;
    uint16_t id = W25QXX_ReadID();
    uav_logf(result ? "WARN" : "INFO", "FLASH", "W25Q32JVSSIQ ID=0x%04x expected=0xEF15 rc=%d CS=PC0",
             (unsigned)id, result);
    if (result)
        uav_logf("WARN", "BOOT", "Flash unavailable; continuing diagnostic boot; parameter storage unresolved");
}
void app_boot_log_task(const char *name, void *handle, uint32_t stack_words) {
    uav_logf("INFO", "TASK", "%s handle=0x%08lx priority=%lu stack=%luwords/%luB heap_free=%luB",
             name, (unsigned long)(uintptr_t)handle,
             (unsigned long)uxTaskPriorityGet((TaskHandle_t)handle), (unsigned long)stack_words,
             (unsigned long)(stack_words * sizeof(StackType_t)),
             (unsigned long)xPortGetFreeHeapSize());
}
void app_boot_resource_init(const char *name, int (*initialize)(void), const char *parameters) {
    uav_logf("INFO", "RESOURCE", "%s begin %s", name, parameters);
    int result = initialize();
    uav_logf(result ? "ERROR" : "INFO", "RESOURCE", "%s rc=%d heap_free=%luB", name, result,
             (unsigned long)xPortGetFreeHeapSize());
    if (result)
        Error_Handler();
}
void app_boot_log_scheduler(void) {
    uav_logf("INFO", "RTOS", "tasks_created=%lu tick=%luHz stack_check=%u heap_free=%luB kernel_start_requested",
             (unsigned long)uxTaskGetNumberOfTasks(), (unsigned long)configTICK_RATE_HZ,
             (unsigned)configCHECK_FOR_STACK_OVERFLOW, (unsigned long)xPortGetFreeHeapSize());
}
void app_boot_log_running(uint16_t period) {
    uav_logf("INFO", "RTOS", "scheduler=RUNNING tasks=%lu heap_free=%luB min_heap=%luB",
             (unsigned long)uxTaskGetNumberOfTasks(), (unsigned long)xPortGetFreeHeapSize(),
             (unsigned long)xPortGetMinimumEverFreeHeapSize());
    uav_logf("INFO", "UART1", "telemetry=0x0004 period=%ums LOG=0x20F0 boot_cache=%uB queue=%uB",
             (unsigned)period, (unsigned)UAV_LOG_BOOT_BYTES, (unsigned)UAV_LOG_QUEUE_BYTES);
    uav_logf("INFO", "UART1", "TX=DMA2_Stream7 completion_timeout=50ms failed_log_chunk=RETRY");
}
static unsigned long stack_free_words(void *handle) {
    TaskStatus_t status;
    vTaskGetInfo((TaskHandle_t)handle, &status, pdTRUE, eInvalid);
    return (unsigned long)status.usStackHighWaterMark;
}
static void log_sensor_sample(void) {
    _imuData_all sample;
    sensor_snapshot_read(&sample);
    const float values[] = {sample.acc.x, sample.acc.y, sample.acc.z,
                            sample.gyro.roll, sample.gyro.pitch, sample.gyro.yaw,
                            sample.mag.x, sample.mag.y, sample.mag.z};
    long scaled[9];
    unsigned bad = 0;
    for (unsigned i = 0; i < 9; i++) {
        if (!isfinite(values[i]) || fabsf(values[i]) > 1000000.0f) {
            bad |= 1u << i;
            scaled[i] = 0;
        } else
            scaled[i] = (long)(values[i] * 1000.0f);
    }
    /* Integer output avoids newlib float formatting on the communication task. */
    uav_logf(bad ? "WARN" : "INFO", "SENSOR_DATA",
             "scale=1000 acc=%ld,%ld,%ld gyro=%ld,%ld,%ld mag=%ld,%ld,%ld bad_fields=0x%03x",
             scaled[0], scaled[1], scaled[2], scaled[3], scaled[4], scaled[5],
             scaled[6], scaled[7], scaled[8], bad);
}
void app_boot_log_health(void) {
    flight_snapshot_t snapshot;
    flight_snapshot_read(&snapshot);
    uav_logf(SensorError ? "WARN" : "INFO", "SENSOR", "init_flags BMI088=%u AK8975=%u SPL06=%u error=%u calibrating=%u attitude_valid=%u mode=%s",
             (unsigned)Bmi088Init_Flag, (unsigned)AK8975Flag, (unsigned)SPL06Flag,
             (unsigned)SensorError, (unsigned)sensor_imu_calibrating(), (unsigned)snapshot.valid,
             Bmi088Init_Flag ? "BLOCKED" : AK8975Flag ? "6AXIS" : "9AXIS");
    uint8_t ids[4], seen;
    uav_sensor_id_snapshot(ids, &seen);
    uav_logf("INFO", "SENSOR_ID", "seen_mask=0x%x ACC=0x%02x GYRO=0x%02x MAG=0x%02x BARO=0x%02x",
             (unsigned)seen, (unsigned)ids[0], (unsigned)ids[1], (unsigned)ids[2], (unsigned)ids[3]);
    log_sensor_sample();
    serial_port_stats_t tx;
    serial_port_get_stats(&tx);
    uav_logf("INFO", "UART_TX", "ok=%lu start_errors=%lu timeouts=%lu dma_errors=%lu last_hal=%lu",
             (unsigned long)tx.frames_ok, (unsigned long)tx.start_errors,
             (unsigned long)tx.timeouts, (unsigned long)tx.dma_errors,
             (unsigned long)tx.last_hal_status);
    uav_display_stats_t display;
    uav_display_get_stats(&display);
    uav_logf(display.errors ? "WARN" : "INFO", "GUI",
             "frames=%lu spans=%lu bytes=%lu errors=%lu last_bytes=%u frame_ms=%u",
             (unsigned long)display.frames, (unsigned long)display.spans,
             (unsigned long)display.bytes, (unsigned long)display.errors,
             (unsigned)display.last_bytes, (unsigned)display.render_ms);
    gui_display_port_stats_t oled;
    gui_display_port_get_stats(&oled);
    uav_logf(oled.dma_errors || oled.timeouts ? "WARN" : "INFO", "OLED",
             "configured=%u on=%u init=%lu SCK=%luHz dma_errors=%lu timeouts=%lu last_hal=%lu",
             (unsigned)oled.configured, (unsigned)oled.enabled, (unsigned long)oled.initializations,
             (unsigned long)oled.clock_hz, (unsigned long)oled.dma_errors,
             (unsigned long)oled.timeouts, (unsigned long)oled.last_hal_status);
    uav_logf("INFO", "STACK", "min_free_words OLED=%lu PC=%lu Sensor=%lu",
             stack_free_words(OLEDTaskHandle), stack_free_words(PCTaskHandle),
             stack_free_words(SensorDataTaskHandle));
    uav_logf("INFO", "HEALTH", "heap_free=%luB min_heap=%luB log_drop_bytes=%lu flight_state=%u",
             (unsigned long)xPortGetFreeHeapSize(), (unsigned long)xPortGetMinimumEverFreeHeapSize(),
             (unsigned long)uav_log_dropped(), (unsigned)snapshot.state);
}
