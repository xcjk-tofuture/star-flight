#include "gui_display_port.h"
#include "oled_device.h"
#include "main.h"
#include "spi.h"
#include "shared_spi.h"
#ifndef GUI_OLED_COLUMN_OFFSET
#define GUI_OLED_COLUMN_OFFSET 0
#endif
void gui_display_port_init(void) { OLED_Init(); }
int gui_display_port_write(void *context, uint8_t page, uint8_t x,
                            const uint8_t *bytes, uint8_t count) {
    (void)context;
    if (!bytes || !count || page >= 8 || (unsigned)x + count > 128) return -1;
    unsigned column = x + GUI_OLED_COLUMN_OFFSET;
    uint8_t commands[] = {(uint8_t)(0xb0u + page),
                           (uint8_t)(column & 15u), (uint8_t)(0x10u | (column >> 4))};
    uav_spi1_lock();
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi1, commands, sizeof(commands), 10);
    if (result == HAL_OK) {
        HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
        result = HAL_SPI_Transmit(&hspi1, (uint8_t *)bytes, count, 10);
    }
    HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
    uav_spi1_unlock();
    return result == HAL_OK ? 0 : -1;
}
