
#ifndef __FLASH_PROC_H__
#define __FLASH_PROC_H__
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include "cmsis_os.h"
#include "timers.h"
#include "parameter.h"
#include "spi.h"
#include "stdio.h"
#include "string.h"

#include "AHRS.h"
#include "motor_proc.h"
#include "sbus_proc.h"
#include "mag_calibration.h"
#include "accel_calibration.h"

int uav_storage_init(void);
uint8_t uav_storage_busy(void);
uint8_t uav_storage_ready(void);
uint32_t uav_storage_epoch(void);
/* 0 pending, 1 verified success, -1 failed, -2 expired/unknown ticket. */
int uav_storage_result(uint32_t ticket);
void uav_storage_diagnostics(uint32_t *rejected, uint32_t sequence[3]);
void Flash_Task_Proc(void const *argument);
void W25q32_Task_Proc(void const *argument);

void UAV_Read_Param_IMU(_imuData_all *imu_data);
void UAV_Read_Param_Remote(_sbus_ch_struct *channe_data);
void UAV_Read_Param_Motor(_uav_control_data *motor_data);
int UAV_Write_Param_Motor(_uav_control_data motor_data);
int UAV_Write_Param_Remote(_sbus_ch_struct channe_data);
int UAV_Write_Param_IMU(_imuData_all imu_data);
int UAV_Write_Param_Mag(const uav_mag_calibration_t *calibration, uint32_t *ticket);
int UAV_Write_Param_Accel(const uav_accel_calibration_t *calibration, uint32_t *ticket);
int UAV_Write_Param_Remote_Ticket(_sbus_ch_struct data, uint32_t *ticket);

#include "w25qxx_device.h"
#endif
