#ifndef APP_BOOT_LOG_H
#define APP_BOOT_LOG_H
#include <stdint.h>
void app_boot_log_hal(void);
void app_boot_log_clocks(void);
void app_boot_log_peripherals(void);
void app_boot_log_flash(int result);
int app_boot_flash_ready(void);
void app_boot_resource_init(const char *name, int (*initialize)(void), const char *parameters);
void app_boot_log_task(const char *name, void *handle, uint32_t stack_words);
void app_boot_log_scheduler(void);
void app_boot_log_running(uint16_t telemetry_period_ms);
void app_boot_log_health(void);
#endif
