#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "sbus_proc.h"
#include "AHRS.h"
#include "display_service.h"
#include "motor_proc.h"
#include "uav_actuator.h"
#include "flight_snapshot.h"
#include "platform_time.h"
#include "flight_machine.h"
#include "flash_proc.h"
#include "beeper.h"
#include <string.h>

static _sbus_ch_cal_struct control_channels;
static _ahrs_data control_attitude;

static _uav_control_data uav_control_data;

osThreadId MotorTaskHandle;

// 传感器校准标准位
// CH5 low=ground/UI, mid=preparation, high=stabilize; CH6 low=safety/kill.

static u8 uavSafeFlag = LOCKED;
static flight_machine_t machine;

void Motor_Task_Proc(void const *argument) {
    uav_actuator_write(0, 1000);
    uav_actuator_write(1, 1000);
    uav_actuator_write(2, 1000);
    uav_actuator_write(3, 1000);
    UAV_Control_Init(&uav_control_data);
    flight_machine_init(&machine);
    TickType_t wake;
    osDelay(5000);
    UAV_Read_Param_Motor(&uav_control_data);
    wake = xTaskGetTickCount();
    for (;;) {
        flight_snapshot_t sensed;
        sbus_snapshot(&control_channels);
        flight_snapshot_read(&sensed);
        control_attitude.roll = sensed.roll_rad * 57.29577951f;
        control_attitude.pitch = sensed.pitch_rad * 57.29577951f;
        control_attitude.yaw = sensed.yaw_rad * 57.29577951f;
        control_attitude.rollSpeed = sensed.roll_rate_radps * 57.29577951f;
        control_attitude.pitchSpeed = sensed.pitch_rate_radps * 57.29577951f;
        control_attitude.yawSpeed = sensed.yaw_rate_radps * 57.29577951f;
        flight_inputs_t inputs = {0};
        TickType_t tick = xTaskGetTickCount();
        inputs.timing_fault = (TickType_t)(tick - wake) > pdMS_TO_TICKS(10);
        if (inputs.timing_fault)
            wake = tick;
        inputs.now_ms = platform_millis();
        inputs.attitude_ms = sensed.attitude_ms;
        inputs.connected = control_channels.Connect_State;
        inputs.attitude_valid = sensed.valid;
        inputs.calibrating = sbus_calibration_active() || sensor_imu_calibrating() ||
                             sensors_mag_calibration_active();
        inputs.storage_busy = uav_storage_busy();
        inputs.channels[0] = control_channels.CAL_CH1;
        inputs.channels[1] = control_channels.CAL_CH2;
        inputs.channels[2] = control_channels.CAL_CH3;
        inputs.channels[3] = control_channels.CAL_CH4;
        inputs.channels[4] = control_channels.CAL_CH5;
        inputs.channels[5] = control_channels.CAL_CH6;
        inputs.channels[6] = control_channels.CAL_CH7;
        inputs.channels[7] = control_channels.CAL_CH8;
        uint8_t before=machine.state;
        flight_machine_step(&machine, &inputs);
        uav_beeper_alarm(machine.state==FM_EMERGENCY);
        if (before!=machine.state && machine.state!=FM_EMERGENCY) {
            uav_beeper_request(machine.state==FM_LOCKED ? UAV_BEEP_LOCK
                : machine.state==FM_ARMED && before==FM_LOCKED ? UAV_BEEP_ARM:UAV_BEEP_ACCEPT);
        }
        uavSafeFlag = machine.state;
        if (uavSafeFlag != FLYING) {
            memset(&uav_control_data.rollPid, 0, sizeof(PID));
            memset(&uav_control_data.pitchPid, 0, sizeof(PID));
            memset(&uav_control_data.yawPid, 0, sizeof(PID));
            memset(&uav_control_data.rollSpeedPid, 0, sizeof(PID));
            memset(&uav_control_data.pitchSpeedPid, 0, sizeof(PID));
        }
        {
            // printf("State :%d\r\n", uavSafeFlag);
            switch (uavSafeFlag) {
            case LOCKED:
                uav_actuator_write(0, 1000);
                uav_actuator_write(1, 1000);
                uav_actuator_write(2, 1000);
                uav_actuator_write(3, 1000);
                //				 uav_actuator_write(0, control_channels.CAL_CH3);
                //				 uav_actuator_write(1, control_channels.CAL_CH3);
                //				 uav_actuator_write(2, control_channels.CAL_CH3);
                //				 uav_actuator_write(3, control_channels.CAL_CH3);
                break;
            case UNLOCKED:
                uav_actuator_write(0, 1050);
                uav_actuator_write(1, 1050);
                uav_actuator_write(2, 1050);
                uav_actuator_write(3, 1050);
                break;
            case FLYING:

                uav_control_data.rollSpeedOut = (s16)PID_Control(
                    &uav_control_data.rollPid, &uav_control_data.rollData, 0.005f, 0,
                    (control_channels.CAL_CH1 - 1500) / 12.0f, control_attitude.roll, 1000);
                uav_control_data.pitchSpeedOut = (s16)PID_Control(
                    &uav_control_data.pitchPid, &uav_control_data.pitchData, 0.005f, 0,
                    (control_channels.CAL_CH2 - 1500) / 12.0f, control_attitude.pitch, 1000);

                uav_control_data.rollOut = (s16)PID_Control(
                    &uav_control_data.rollSpeedPid, &uav_control_data.rollSpeedData, 0.005f, 0,
                    uav_control_data.rollSpeedOut, control_attitude.rollSpeed, 1000);
                uav_control_data.pitchOut = (s16)PID_Control(
                    &uav_control_data.pitchSpeedPid, &uav_control_data.pitchSpeedData, 0.005f, 0,
                    uav_control_data.pitchSpeedOut, control_attitude.pitchSpeed, 1000);

                uav_control_data.yawOut =
                    (s16)PID_Control(&uav_control_data.yawPid, &uav_control_data.yawData, 0.005f, 0,
                                     ((control_channels.CAL_CH4 - 1500) / 12.0f) * 0.01,
                                     control_attitude.yawSpeed, 1000);

                if (control_channels.CAL_CH3 > 1100) {
                    uav_actuator_write(0,
                                       control_channels.CAL_CH3 - uav_control_data.rollOut -
                                           uav_control_data.pitchOut); //+ uav_control_data.yawOut;
                    uav_actuator_write(1,
                                       control_channels.CAL_CH3 + uav_control_data.rollOut +
                                           uav_control_data.pitchOut); // + uav_control_data.yawOut;
                    uav_actuator_write(2,
                                       control_channels.CAL_CH3 + uav_control_data.rollOut -
                                           uav_control_data.pitchOut); // - uav_control_data.yawOut;
                    uav_actuator_write(3,
                                       control_channels.CAL_CH3 - uav_control_data.rollOut +
                                           uav_control_data.pitchOut); //- uav_control_data.yawOut;
                } else {
                    uav_actuator_stop();
                }
                break;
            case EMERGENCY:
                uav_actuator_write(0, 1000);
                uav_actuator_write(1, 1000);
                uav_actuator_write(2, 1000);
                uav_actuator_write(3, 1000);
                break;
            }

        }
        flight_fault_publish(machine.reason, machine.transitions);
        flight_state_publish(uavSafeFlag);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
    }
}

void UAV_Control_Init(_uav_control_data *uav_data) {
    uav_data->rollData.Kp = 0.1;
    uav_data->rollData.Ki = 0;
    uav_data->rollData.Kd = 0;
    uav_data->rollData.ErrorMax = 70;
    uav_data->rollData.DifferentialMax = 200;
    uav_data->rollData.IntegrateMax = 1000;

    uav_data->pitchData.Kp = 0.1;
    uav_data->pitchData.Ki = 0;
    uav_data->pitchData.Kd = 0;
    uav_data->pitchData.ErrorMax = 70;
    uav_data->pitchData.DifferentialMax = 200;
    uav_data->pitchData.IntegrateMax = 1000;

    uav_data->yawData.Kp = 0.1;
    uav_data->yawData.Ki = 0;
    uav_data->yawData.Kd = 0;
    uav_data->yawData.ErrorMax = 70;
    uav_data->yawData.DifferentialMax = 200;
    uav_data->yawData.IntegrateMax = 1000;

    uav_data->rollSpeedData.Kp = 0.1;
    uav_data->rollSpeedData.Ki = 0;
    uav_data->rollSpeedData.Kd = 0;
    uav_data->rollSpeedData.ErrorMax = 100;
    uav_data->rollSpeedData.DifferentialMax = 200;
    uav_data->rollSpeedData.IntegrateMax = 1000;

    uav_data->pitchSpeedData.Kp = 0.1;
    uav_data->pitchSpeedData.Ki = 0;
    uav_data->pitchSpeedData.Kd = 100;
    uav_data->pitchSpeedData.ErrorMax = 100;
    uav_data->pitchSpeedData.DifferentialMax = 200;
    uav_data->pitchSpeedData.IntegrateMax = 1000;
}
