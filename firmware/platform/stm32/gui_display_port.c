#include "gui_display_port.h"
#include "main.h"
#include "spi.h"
#include "shared_spi.h"
#include "gui_oled_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "log_service.h"
#include <string.h>
extern DMA_HandleTypeDef hdma_spi1_tx;
static volatile uint8_t dma_active, dma_done, dma_failed;
static uint8_t dma_bytes[128];
static gui_display_port_stats_t stats;
typedef struct { uint32_t cr1, direction; } spi_profile_t;

void DMA2_Stream3_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_spi1_tx); }
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *spi) {
    if (spi == &hspi1 && dma_active) dma_done = 1;
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi) {
    if (spi == &hspi1 && dma_active) dma_failed = 1;
}
void gui_display_port_get_stats(gui_display_port_stats_t *out) {
    if (!out) return;
    taskENTER_CRITICAL(); *out = stats; taskEXIT_CRITICAL();
}
static void signal_guard(void) {
    /* A few microseconds of setup/hold margin; no precise upper delay needed. */
    volatile uint32_t count = SystemCoreClock / 2000000u;
    while (count--) { __NOP(); }
}
static int begin_profile(spi_profile_t *saved) {
    uav_spi1_lock();
    if (hspi1.State != HAL_SPI_STATE_READY || __HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY)) {
        stats.last_hal_status = HAL_BUSY;
        uav_spi1_unlock(); return -1;
    }
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
    saved->cr1 = READ_REG(hspi1.Instance->CR1);
    saved->direction = hspi1.Init.Direction;
    __HAL_SPI_DISABLE(&hspi1);
    /* One-line TX prevents RX overrun IRQs during OLED DMA. This is local to
     * the locked transaction; Flash gets its original duplex and clock back. */
    WRITE_REG(hspi1.Instance->CR1,
              (saved->cr1 & ~(SPI_CR1_SPE | SPI_CR1_BR | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE)) |
              GUI_OLED_SPI_PRESCALER | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);
    hspi1.Init.Direction = SPI_DIRECTION_1LINE;
    signal_guard();
    return 0;
}
static void end_profile(const spi_profile_t *saved) {
    signal_guard();
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
    signal_guard();
    __HAL_SPI_DISABLE(&hspi1);
    hspi1.Init.Direction = saved->direction;
    WRITE_REG(hspi1.Instance->CR1, saved->cr1);
    uav_spi1_unlock();
}
static HAL_StatusTypeDef send_commands(const uint8_t *commands, uint16_t length) {
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);
    signal_guard();
    HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi1, (uint8_t *)commands, length, 30);
    signal_guard();
    stats.last_hal_status = (uint32_t)result;
    return result;
}
static HAL_StatusTypeDef send_data(const uint8_t *bytes, uint8_t count) {
    memcpy(dma_bytes, bytes, count);
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
    signal_guard();
    dma_done = dma_failed = 0; dma_active = 1;
    HAL_StatusTypeDef result = HAL_SPI_Transmit_DMA(&hspi1, dma_bytes, count);
    if (result == HAL_OK) {
        uint32_t start = HAL_GetTick();
        while (!dma_done && !dma_failed && (uint32_t)(HAL_GetTick()-start) < 50u)
            vTaskDelay(pdMS_TO_TICKS(1));
        if (dma_failed) { result = HAL_ERROR; stats.dma_errors++; }
        else if (!dma_done) { result = HAL_TIMEOUT; stats.timeouts++; }
    }
    dma_active = 0;
    if (result != HAL_OK) HAL_SPI_Abort(&hspi1); /* Finish before reusing DMA SRAM. */
    stats.last_hal_status = (uint32_t)result;
    return result;
}
static int command_transaction(const uint8_t *bytes, uint16_t count) {
    spi_profile_t saved;
    if (begin_profile(&saved)) return -1;
    HAL_StatusTypeDef result = send_commands(bytes, count);
    end_profile(&saved);
    return result == HAL_OK ? 0 : -1;
}
int gui_display_port_write(void *context, uint8_t page, uint8_t x,
                            const uint8_t *bytes, uint8_t count) {
    (void)context;
    if (!bytes || !count || page >= 8 || (unsigned)x + count > 128) return -1;
    unsigned column = x + GUI_OLED_COLUMN_OFFSET;
    uint8_t commands[] = {(uint8_t)(0xb0u + page),
                           (uint8_t)(column & 15u), (uint8_t)(0x10u | (column >> 4))};
    spi_profile_t saved;
    if (begin_profile(&saved)) return -1;
    HAL_StatusTypeDef result = send_commands(commands, sizeof(commands));
    if (result == HAL_OK) result = send_data(bytes, count);
    end_profile(&saved);
    return result == HAL_OK ? 0 : -1;
}
int gui_display_port_init(void) {
    stats.initializations++; stats.configured = stats.enabled = 0;
    stats.clock_hz = HAL_RCC_GetPCLK2Freq() / GUI_OLED_SPI_DIVIDER;
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream3_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    /* Keep the panel deselected while its supply settles, then issue a fresh
     * reset pulse. Do not keep the shared SPI mutex during these delays. */
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET);
    osDelay(GUI_OLED_POWER_WAIT_MS);
    HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_RESET);
    osDelay(GUI_OLED_RESET_LOW_MS);
    HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, GPIO_PIN_SET);
    osDelay(GUI_OLED_RESET_SETTLE_MS);
    static const uint8_t initialize[] = {
        0xae, 0xd5, 0x80, 0xa8, 0x3f, 0xd3, 0x00, 0x40, 0xa1, 0xc8,
        0xda, 0x12, 0x81, GUI_OLED_CONTRAST, 0xd9, 0xf1, 0xdb, 0x40, 0xa4, 0xa6,
#if GUI_OLED_CONTROLLER == GUI_OLED_SH1106
        0xad, 0x8b
#else
        0x2e, 0x20, 0x02, 0x8d, 0x14 /* Scroll off, page mode, charge pump. */
#endif
    };
    if (command_transaction(initialize, sizeof(initialize))) return -1;
    osDelay(30); /* Charge pump/supply settling while display remains OFF. */
    static const uint8_t black[128] = {0};
    for (uint8_t page = 0; page < 8; page++)
        if (gui_display_port_write(NULL, page, 0, black, sizeof(black))) return -1;
    stats.configured = 1;
    uav_logf("INFO", "OLED", "controller_profile=%s SCK=%luHz TX=DMA div=64 reset=20ms settle=30ms black_before_on=1",
             GUI_OLED_CONTROLLER == GUI_OLED_SH1106 ? "SH1106" : "SSD1306",
             (unsigned long)stats.clock_hz);
    return 0; /* Display ON is deferred until a complete GUI frame is written. */
}
int gui_display_port_enable(void) {
    static const uint8_t enable[] = {0xaf};
    if (!stats.configured) return -1;
    if (command_transaction(enable, sizeof(enable))) return -1;
    stats.enabled = 1;
    return 0;
}
int gui_display_port_reassert(void) {
    static const uint8_t mapping[] = {
#if GUI_OLED_CONTROLLER != GUI_OLED_SH1106
        0x2e, 0x20, 0x02,
#endif
        0x40, 0xd3, 0x00, 0xa1, 0xc8, 0xa4, 0xa6
    };
    return command_transaction(mapping, sizeof(mapping));
}
