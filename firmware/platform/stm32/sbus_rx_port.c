#include "sbus_rx_port.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
extern uint8_t SbusRxBuf[100];
static volatile uint8_t recovery_pending;
static volatile uint32_t errors, parity, noise, framing, overrun, dma_errors, last_error;
static uint32_t attempts, recovered, failed, last_status, last_attempt_ms;
static uint8_t attempted, consecutive_failures;
void uav_sbus_rx_error_isr(uint32_t error) {
    errors++; last_error=error; recovery_pending=1;
    if (error & HAL_UART_ERROR_PE) parity++;
    if (error & HAL_UART_ERROR_NE) noise++;
    if (error & HAL_UART_ERROR_FE) framing++;
    if (error & HAL_UART_ERROR_ORE) overrun++;
    if (error & HAL_UART_ERROR_DMA) dma_errors++;
}
uint8_t uav_sbus_rx_pending(void) { return recovery_pending; }
uint8_t uav_sbus_rx_running(void) {
    return huart6.RxState==HAL_UART_STATE_BUSY_RX && huart6.hdmarx &&
        (huart6.Instance->CR3 & USART_CR3_DMAR) && (huart6.hdmarx->Instance->CR & DMA_SxCR_EN) &&
        huart6.ReceptionType==HAL_UART_RECEPTION_TOIDLE && (huart6.Instance->CR1 & USART_CR1_IDLEIE);
}
int uav_sbus_rx_service(uint32_t now) {
    if (uav_sbus_rx_running() && (!recovery_pending || huart6.ErrorCode==HAL_UART_ERROR_NONE)) {
        recovery_pending=0; return 0;
    }
    if (attempted && (uint32_t)(now-last_attempt_ms)<20u) return 0;
    attempted=1; last_attempt_ms=now; attempts++;
    /* IRQ reports the fault only. Restart after DMA abort callbacks have left,
     * clear stale SR/DR flags, and check the actual stream instead of HAL_OK alone. */
    uint32_t error=huart6.ErrorCode;
    if (error) last_error=error;
    HAL_StatusTypeDef status=HAL_UART_AbortReceive(&huart6);
    if (status==HAL_OK && huart6.hdmarx && huart6.hdmarx->State==HAL_DMA_STATE_BUSY)
        status=HAL_DMA_Abort(huart6.hdmarx);
    if (status==HAL_OK && huart6.hdmarx && huart6.hdmarx->State!=HAL_DMA_STATE_READY) {
        status=HAL_DMA_DeInit(huart6.hdmarx);
        if (status==HAL_OK) status=HAL_DMA_Init(huart6.hdmarx);
    }
    /* A stale HAL lock or abort state must not leave the receiver dead forever.
     * Escalate through public HAL APIs, only for USART6, without MCU reset. */
    if ((status!=HAL_OK || huart6.Lock==HAL_LOCKED || consecutive_failures>=3)) {
        status=HAL_UART_DeInit(&huart6);
        if (status==HAL_OK) status=HAL_UART_Init(&huart6);
    }
    if (status==HAL_OK) {
        __HAL_UART_CLEAR_OREFLAG(&huart6);
        status=HAL_UARTEx_ReceiveToIdle_DMA(&huart6,SbusRxBuf,sizeof(SbusRxBuf));
        if (status==HAL_OK) __HAL_DMA_DISABLE_IT(huart6.hdmarx,DMA_IT_HT);
    }
    last_status=(uint32_t)status;
    if (status==HAL_OK && uav_sbus_rx_running()) {
        recovery_pending=0; consecutive_failures=0; recovered++; return 1;
    }
    recovery_pending=1; failed++;
    if (consecutive_failures<3) consecutive_failures++;
    return -1;
}
void uav_sbus_rx_stats_read(uav_sbus_rx_stats_t *out) {
    taskENTER_CRITICAL();
    *out=(uav_sbus_rx_stats_t){errors,parity,noise,framing,overrun,dma_errors,
        attempts,recovered,failed,last_error,last_status,uav_sbus_rx_running(),recovery_pending};
    taskEXIT_CRITICAL();
}
