#include "platform_time.h"
#include "main.h"
uint32_t platform_millis(void) { return HAL_GetTick(); }
uint32_t platform_micros(void) {
    static uint32_t previous_cycles, micros, remainder;
    static uint8_t initialized;
    uint32_t cycles_per_us=SystemCoreClock/1000000u;
    if (!initialized) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        previous_cycles=DWT->CYCCNT;
        micros=HAL_GetTick()*1000u;
        initialized=1;
    }
    uint32_t cycles=DWT->CYCCNT;
    uint32_t delta=(uint32_t)(cycles-previous_cycles);
    previous_cycles=cycles;
    micros+=delta/cycles_per_us;
    remainder+=delta%cycles_per_us;
    micros+=remainder/cycles_per_us;
    remainder%=cycles_per_us;
    return micros;
}
