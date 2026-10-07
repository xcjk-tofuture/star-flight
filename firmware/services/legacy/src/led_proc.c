#include "sensor_port.h"
#include "led_proc.h"
#include "flight_snapshot.h"
#include "AHRS.h"

#include "motor_proc.h"

osThreadId RGBTaskHandle;

u8 ucLed;

void RGB_Task_Proc(void const *argument) // RGB进程主程序
{
    /* USER CODE BEGIN RGB_Task_Proc */
    /* Infinite loop */
    ucLed = 0x00;
    for (;;) {

        if (sensor_imu_calibration_failed())
            uav_device_led_write(4); /* Latched red; RESET starts a fresh attempt. */
        else if (sensor_imu_calibrating())
            RGB_Calibration(ucLed);
        else {
            flight_snapshot_t flight;
            flight_snapshot_read(&flight);
            if (flight.state == LOCKED)
                RGB_Normal(ucLed);
            else if (flight.state == UNLOCKED)
                RGB_Unlocked(ucLed);
            else if (flight.state == EMERGENCY)
                RGB_Emergency(ucLed);
            else if (flight.state == FLYING)
                RGB_Flying(ucLed);
        }

        osDelay(100);
    }
    /* USER CODE END RGB_Task_Proc */
}

void RGB_Calibration(u8 ucled) {
    static u8 ledTemp;
    ledTemp ^= 0x01;
    ucLed = ledTemp;
    RGB_Show_Proc(ucLed);
}

void RGB_Normal(u8 ucled) {
    // if(ucLed > 0x04) ucLed = 0x01;

    static u8 ledTemp;
    ledTemp ^= 0x07;
    ucLed = ledTemp;
    osDelay(400);
    RGB_Show_Proc(ucLed);
}

void RGB_Emergency(u8 ucled) {
    // if(ucLed > 0x04) ucLed = 0x01;
    static u8 ledTemp;
    ledTemp ^= 0x02;
    ucLed = ledTemp;
    RGB_Show_Proc(ucLed);
}

void RGB_Unlocked(u8 ucled) {
    // if(ucLed > 0x04) ucLed = 0x01;
    static u8 ledTemp;
    ledTemp ^= 0x06;
    ucLed = ledTemp;
    osDelay(200);
    RGB_Show_Proc(ucLed);
}

void RGB_Flying(u8 ucled) {
    // if(ucLed > 0x04) ucLed = 0x01;
    static u8 ledTemp;
    ledTemp ^= 0x04;
    ucLed = ledTemp;
    osDelay(400);
    RGB_Show_Proc(ucLed);
}

void RGB_Show_Proc(u8 ucled) // 0b000 后三位赋1 灯亮
{
    uav_device_led_write(ucled);
};
