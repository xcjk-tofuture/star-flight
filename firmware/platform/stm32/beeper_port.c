#include "beeper_port.h"
#include "tim.h"
static uint16_t current_hz;
int uav_beeper_port_init(void) {
    uint32_t timer_clock=HAL_RCC_GetPCLK1Freq();
    if (timer_clock!=HAL_RCC_GetHCLKFreq()) timer_clock*=2;
    if (!htim3.Instance || timer_clock<1000000u) return -1;
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
    __HAL_TIM_SET_PRESCALER(&htim3,timer_clock/1000000u-1);
    __HAL_TIM_SET_AUTORELOAD(&htim3,499);
    htim3.Instance->EGR=TIM_EGR_UG;
    return HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1)==HAL_OK ? 0:-1;
}
void uav_beeper_port_tone(uint16_t hz) {
    if (hz==current_hz) return;
    current_hz=hz;
    if (!hz) { __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0); return; }
    if (hz<1000) hz=1000;
    if (hz>4000) hz=4000;
    uint32_t period=1000000u/hz;
    __HAL_TIM_SET_AUTORELOAD(&htim3,period-1);
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,period/2);
    __HAL_TIM_SET_COUNTER(&htim3,0);
}
