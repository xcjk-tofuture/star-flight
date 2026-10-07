#include "serial_port.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
static uint8_t transmit_bytes[140];
static volatile uint8_t transmit_complete;
static serial_port_stats_t stats;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart) {
    if (uart == &huart1)
        transmit_complete = 1;
}
void serial_port_get_stats(serial_port_stats_t *out) {
    if (out)
    {
        taskENTER_CRITICAL(); *out = stats; taskEXIT_CRITICAL();
    }
}
int serial_port_send(const uint8_t *bytes, size_t length) {
    if (!bytes || !length || length > sizeof(transmit_bytes))
        return -1;
    if (huart1.gState != HAL_UART_STATE_READY) {
        stats.last_hal_status = HAL_BUSY;
        stats.start_errors++;
        return -1;
    }
    memcpy(transmit_bytes, bytes, length);
    transmit_complete = 0;
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1, transmit_bytes, (uint16_t)length);
    stats.last_hal_status = (uint32_t)status;
    if (status != HAL_OK) {
        stats.start_errors++;
        return -1;
    }
    uint32_t started = HAL_GetTick();
    while (!transmit_complete) {
        if (huart1.hdmatx->ErrorCode != HAL_DMA_ERROR_NONE) {
            HAL_UART_AbortTransmit(&huart1);
            stats.last_hal_status = HAL_ERROR;
            stats.dma_errors++;
            return -1;
        }
        if ((uint32_t)(HAL_GetTick() - started) >= 50u) {
            /* Stop DMA before the static buffer can be reused; preserve RX. */
            HAL_UART_AbortTransmit(&huart1);
            stats.last_hal_status = HAL_TIMEOUT;
            stats.timeouts++;
            return -1;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    stats.frames_ok++;
    return 0;
}
