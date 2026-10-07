#include "log_service.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "platform_time.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
static QueueHandle_t bytes;
static uint8_t boot_bytes[UAV_LOG_BOOT_BYTES];
static size_t boot_count, boot_read;
static uint32_t dropped;
int uav_log_init(void) {
    bytes = xQueueCreate(UAV_LOG_QUEUE_BYTES, sizeof(uint8_t));
    return bytes ? 0 : -1;
}
void uav_log_write(const uint8_t *data, size_t length) {
    if (!data || !length)
        return;
    if (!bytes || xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        if (length > sizeof(boot_bytes) - boot_count)
            dropped += (uint32_t)length;
        else {
            memcpy(boot_bytes + boot_count, data, length);
            boot_count += length;
        }
        return;
    }
    taskENTER_CRITICAL();
    if (length > uxQueueSpacesAvailable(bytes))
        dropped += (uint32_t)length;
    else
        for (size_t i = 0; i < length; i++)
            xQueueSend(bytes, data + i, 0);
    taskEXIT_CRITICAL();
}
void uav_log_byte(uint8_t byte) { uav_log_write(&byte, 1); }
uint32_t uav_log_dropped(void) { return dropped; }
void uav_logf(const char *level, const char *module, const char *format, ...) {
    char line[256];
    if (!level || !module || !format)
        return;
    int prefix = snprintf(line, sizeof(line), "[%010lu ms][%s][%s] ",
                          (unsigned long)platform_millis(), level, module);
    if (prefix < 0 || (size_t)prefix >= sizeof(line) - 3)
        return;
    va_list args;
    va_start(args, format);
    int message = vsnprintf(line + prefix, sizeof(line) - (size_t)prefix - 2, format, args);
    va_end(args);
    if (message < 0)
        return;
    size_t length = strlen(line);
    line[length++] = '\r';
    line[length++] = '\n';
    uav_log_write((const uint8_t *)line, length);
}
size_t uav_log_receive(uint8_t *out, size_t capacity, uint32_t timeout_ms) {
    if (!out || !capacity)
        return 0;
    size_t count = 0;
    while (count < capacity && boot_read < boot_count)
        out[count++] = boot_bytes[boot_read++];
    if (!bytes || count == capacity)
        return count;
    uint32_t wait = count ? 0 : timeout_ms;
    if (xQueueReceive(bytes, out + count, pdMS_TO_TICKS(wait)) == pdPASS) {
        count++;
        while (count < capacity && xQueueReceive(bytes, out + count, 0) == pdPASS)
            count++;
    }
    return count;
}
