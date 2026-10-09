#include "flow_rx_port.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
extern uint8_t uart2RX[200];
static volatile uint8_t pending;
static volatile uint32_t errors,last_error;
static uint32_t attempts,recovered,failed,last_attempt;
static uint8_t attempted,failures;
void uav_flow_rx_error_isr(uint32_t error) { pending=1; errors++; last_error=error; }
uint8_t uav_flow_rx_running(void) {
    return huart2.RxState==HAL_UART_STATE_BUSY_RX && huart2.hdmarx &&
        (huart2.Instance->CR3 & USART_CR3_DMAR) && (huart2.hdmarx->Instance->CR & DMA_SxCR_EN) &&
        huart2.ReceptionType==HAL_UART_RECEPTION_TOIDLE && (huart2.Instance->CR1 & USART_CR1_IDLEIE);
}
int uav_flow_rx_service(uint32_t now) {
    if (uav_flow_rx_running() && (!pending || huart2.ErrorCode==HAL_UART_ERROR_NONE)) { pending=0; return 0; }
    if (attempted && (uint32_t)(now-last_attempt)<20u) return 0;
    attempted=1; last_attempt=now; attempts++;
    HAL_StatusTypeDef status=HAL_UART_AbortReceive(&huart2);
    if (status==HAL_OK && huart2.hdmarx && huart2.hdmarx->State==HAL_DMA_STATE_BUSY) status=HAL_DMA_Abort(huart2.hdmarx);
    if (status==HAL_OK && huart2.hdmarx && huart2.hdmarx->State!=HAL_DMA_STATE_READY) {
        status=HAL_DMA_DeInit(huart2.hdmarx); if (status==HAL_OK) status=HAL_DMA_Init(huart2.hdmarx);
    }
    if (status!=HAL_OK || huart2.Lock==HAL_LOCKED || failures>=3) {
        status=HAL_UART_DeInit(&huart2); if (status==HAL_OK) status=HAL_UART_Init(&huart2);
    }
    if (status==HAL_OK) {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        status=HAL_UARTEx_ReceiveToIdle_DMA(&huart2,uart2RX,sizeof(uart2RX));
        if (status==HAL_OK) __HAL_DMA_DISABLE_IT(huart2.hdmarx,DMA_IT_HT);
    }
    if (status==HAL_OK && uav_flow_rx_running()) { pending=0; failures=0; recovered++; return 1; }
    pending=1; failed++; if (failures<3) failures++; return -1;
}
void uav_flow_rx_stats_read(uav_flow_rx_stats_t *out) {
    taskENTER_CRITICAL(); *out=(uav_flow_rx_stats_t){errors,attempts,recovered,failed,last_error,uav_flow_rx_running(),pending}; taskEXIT_CRITICAL();
}
