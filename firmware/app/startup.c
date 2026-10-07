#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include "pc_proc.h"
#include "sbus_proc.h"
#include "log_service.h"
#include "shared_spi.h"
#include "flash_proc.h"
#include "flow_proc.h"
extern osThreadId RGBTaskHandle;
extern void RGB_Task_Proc(void const *argument);
extern osThreadId KeyTaskHandle;
extern void Key_Task_Proc(void const *argument);
extern osThreadId SbusUart6TaskHandle;
extern void Sbus_Uart6_Task_Proc(void const *argument);
extern osThreadId OLEDTaskHandle;
extern void OLED_Task_Proc(void const *argument);
extern osThreadId SensorDataTaskHandle;
extern void Sensor_Data_Task_Proc(void const *argument);
extern osThreadId FlashTaskHandle;
extern void Flash_Task_Proc(void const *argument);
extern osThreadId FlowTaskHandle;
extern void Flow_Task_Proc(void const *argument);
extern osThreadId PCTaskHandle;
extern void PC_Task_Proc(void const *argument);
extern osThreadId MotorTaskHandle;
extern void Motor_Task_Proc(void const *argument);
void app_tasks_init(void) {
    if (PC_Init() != 0 || sbus_transport_init() != 0 || uav_log_init() != 0 ||
        flow_transport_init() != 0 || uav_spi1_init() != 0 || uav_storage_init() != 0)
        Error_Handler();
    osThreadDef(RGB, RGB_Task_Proc, osPriorityIdle, 0, 128);
    RGBTaskHandle = osThreadCreate(osThread(RGB), NULL);
    if (!RGBTaskHandle)
        Error_Handler();
    osThreadDef(Key, Key_Task_Proc, osPriorityIdle, 0, 128);
    KeyTaskHandle = osThreadCreate(osThread(Key), NULL);
    if (!KeyTaskHandle)
        Error_Handler();
    osThreadDef(Sbus, Sbus_Uart6_Task_Proc, osPriorityAboveNormal, 0, 256);
    SbusUart6TaskHandle = osThreadCreate(osThread(Sbus), NULL);
    if (!SbusUart6TaskHandle)
        Error_Handler();
    /* Keep the known working stack budget for rendering and the FPU context. */
    osThreadDef(OLED, OLED_Task_Proc, osPriorityIdle, 0, 1024);
    OLEDTaskHandle = osThreadCreate(osThread(OLED), NULL);
    if (!OLEDTaskHandle)
        Error_Handler();
    osThreadDef(Sensor, Sensor_Data_Task_Proc, osPriorityRealtime, 0, 768);
    SensorDataTaskHandle = osThreadCreate(osThread(Sensor), NULL);
    if (!SensorDataTaskHandle)
        Error_Handler();
    osThreadDef(Flash, Flash_Task_Proc, osPriorityIdle, 0, 768);
    FlashTaskHandle = osThreadCreate(osThread(Flash), NULL);
    if (!FlashTaskHandle)
        Error_Handler();
    osThreadDef(Flow, Flow_Task_Proc, osPriorityIdle, 0, 128);
    FlowTaskHandle = osThreadCreate(osThread(Flow), NULL);
    if (!FlowTaskHandle)
        Error_Handler();
    osThreadDef(PC, PC_Task_Proc, osPriorityNormal, 0, 768);
    PCTaskHandle = osThreadCreate(osThread(PC), NULL);
    if (!PCTaskHandle)
        Error_Handler();
    osThreadDef(Control, Motor_Task_Proc, osPriorityHigh, 0, 512);
    MotorTaskHandle = osThreadCreate(osThread(Control), NULL);
    if (!MotorTaskHandle)
        Error_Handler();
}
