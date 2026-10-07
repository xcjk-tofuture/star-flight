#ifndef __MOTOR_PROC_H__
#define __MOTOR_PROC_H__
#include <stdint.h>

#include "pid.h"

typedef struct _UAV_CONTROL_DATA {
    PID rollPid;
    PID_DATA rollData;

    PID pitchPid;
    PID_DATA pitchData;

    PID yawPid;
    PID_DATA yawData;

    PID rollSpeedPid;
    PID_DATA rollSpeedData;

    PID pitchSpeedPid;
    PID_DATA pitchSpeedData;

    float rollSpeedOut;
    float pitchSpeedOut;

    float rollOut;
    float pitchOut;
    float yawOut;

    float xVel;
    float yVel;
    float zVel;

    float height;
} _uav_control_data;

typedef enum { LOCKED = 0, UNLOCKED, FLYING, EMERGENCY } UAV_STA;

void Motor_Task_Proc(void const *argument);
void UAV_Control_Init(_uav_control_data *uav_data);
void mag_cail_proc(void);
// void pwm_timer_callback(TimerHandle_t xTimer);
// void pwm_init(uint16_t frequency, uint8_t duty_cycle);
// void pwm_set(uint16_t frequency, uint8_t duty_cycle);

#endif