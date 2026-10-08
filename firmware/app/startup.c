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
#include "boot_log.h"
#include "beeper.h"
#include "app_settings.h"
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
    app_boot_resource_init("PC_RX", PC_Init, "depth=4 max_rx=100B telemetry=50ms");
    app_boot_resource_init("SBUS_RX", sbus_transport_init, "chunks=4x100B parser=25B RX=100k8E1 legacy_profile timeout=100ms auto_rearm=1");
    app_boot_resource_init("LOG", uav_log_init, "boot=8192B runtime=1024B chunk=64B");
    app_boot_resource_init("FLOW_RX", flow_transport_init, "depth=4 frame=14B timeout=100ms");
    app_boot_resource_init("SPI1_MUTEX", uav_spi1_init, "recursive priority_inheritance=1");
    app_boot_resource_init("STORAGE", uav_storage_init, "write_depth=2");
    uav_settings_init();
    uav_beeper_init();
    osThreadDef(RGB, RGB_Task_Proc, osPriorityIdle, 0, 128);
    RGBTaskHandle = osThreadCreate(osThread(RGB), NULL);
    if (!RGBTaskHandle)
        Error_Handler();
    app_boot_log_task("RGB", RGBTaskHandle, 128);
    osThreadDef(Key, Key_Task_Proc, osPriorityIdle, 0, 128);
    KeyTaskHandle = osThreadCreate(osThread(Key), NULL);
    if (!KeyTaskHandle)
        Error_Handler();
    app_boot_log_task("Key", KeyTaskHandle, 128);
    osThreadDef(Sbus, Sbus_Uart6_Task_Proc, osPriorityAboveNormal, 0, 384);
    SbusUart6TaskHandle = osThreadCreate(osThread(Sbus), NULL);
    if (!SbusUart6TaskHandle)
        Error_Handler();
    app_boot_log_task("Sbus", SbusUart6TaskHandle, 384);
    /* Keep the known working stack budget for rendering and the FPU context. */
    osThreadDef(OLED, OLED_Task_Proc, osPriorityIdle, 0, 1024);
    OLEDTaskHandle = osThreadCreate(osThread(OLED), NULL);
    if (!OLEDTaskHandle)
        Error_Handler();
    app_boot_log_task("OLED", OLEDTaskHandle, 1024);
    osThreadDef(Sensor, Sensor_Data_Task_Proc, osPriorityRealtime, 0, 768);
    SensorDataTaskHandle = osThreadCreate(osThread(Sensor), NULL);
    if (!SensorDataTaskHandle)
        Error_Handler();
    app_boot_log_task("Sensor", SensorDataTaskHandle, 768);
    osThreadDef(Flash, Flash_Task_Proc, osPriorityIdle, 0, 768);
    FlashTaskHandle = osThreadCreate(osThread(Flash), NULL);
    if (!FlashTaskHandle)
        Error_Handler();
    app_boot_log_task("Flash", FlashTaskHandle, 768);
    osThreadDef(Flow, Flow_Task_Proc, osPriorityIdle, 0, 128);
    FlowTaskHandle = osThreadCreate(osThread(Flow), NULL);
    if (!FlowTaskHandle)
        Error_Handler();
    app_boot_log_task("Flow", FlowTaskHandle, 128);
    osThreadDef(PC, PC_Task_Proc, osPriorityNormal, 0, 768);
    PCTaskHandle = osThreadCreate(osThread(PC), NULL);
    if (!PCTaskHandle)
        Error_Handler();
    app_boot_log_task("PC", PCTaskHandle, 768);
    osThreadDef(Control, Motor_Task_Proc, osPriorityHigh, 0, 512);
    MotorTaskHandle = osThreadCreate(osThread(Control), NULL);
    if (!MotorTaskHandle)
        Error_Handler();
    app_boot_log_task("Control", MotorTaskHandle, 512);
    app_boot_log_scheduler();
}
