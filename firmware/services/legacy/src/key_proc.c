#include "sensor_port.h"
#include "display_service.h"
#include "key_proc.h"

#include "platform_time.h"
#include "beeper.h"

u8 keyUp, keyDown, keyOld, keyValue;

osThreadId KeyTaskHandle;

void Key_Task_Proc(void const *argument) {
    (void)argument;
    uint8_t candidate = 0, stable = 0, samples = 0;
    uint8_t key2_long_emitted=0;
    uint32_t pressed_ms = 0;
    for (;;) {
        keyDown = keyUp = 0;
        uint8_t raw = Key_Scan();
        if (raw != candidate) { candidate = raw; samples = 1; }
        else if (samples < 4) samples++;
        if (samples == 4 && candidate != stable) {
            uint8_t previous = stable;
            uint32_t now = platform_millis();
            stable = candidate;
            keyValue = stable;
            keyDown = stable & (previous ^ stable);
            keyUp = (uint8_t)(~stable) & (previous ^ stable);
            keyOld = stable;
            if (previous && !(previous==2 && key2_long_emitted)) {
                int long_press = (uint32_t)(now-pressed_ms) >= 600u;
                uav_display_input(previous == 1 ? (long_press ? UAV_DISPLAY_BACK : UAV_DISPLAY_NEXT)
                                   : (long_press ? UAV_DISPLAY_CAL_MENU : UAV_DISPLAY_CONFIRM));
            }
            if (stable) pressed_ms = now;
            if (stable==2) key2_long_emitted=0;
        }
        if (stable==2 && !key2_long_emitted && (uint32_t)(platform_millis()-pressed_ms)>=600u) {
            uav_display_input(UAV_DISPLAY_CAL_MENU); key2_long_emitted=1;
        }
        osDelay(5);
        uav_beeper_tick(platform_millis());
    }
}

u8 Key_Scan(void) {
    uint8_t key;
    uav_device_key_scan(&key);
    return key;
}
