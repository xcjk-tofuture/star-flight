#ifndef STAR_GUI_OLED_CONFIG_H
#define STAR_GUI_OLED_CONFIG_H
/* Board-specific display settings. Default matches the original SSD1306 init.
 * Set GUI_OLED_CONTROLLER to 2 for a confirmed SH1106 module (column offset 2). */
#define GUI_OLED_SSD1306 1
#define GUI_OLED_SH1106 2
#ifndef GUI_OLED_CONTROLLER
#define GUI_OLED_CONTROLLER GUI_OLED_SSD1306
#endif
#if GUI_OLED_CONTROLLER != GUI_OLED_SSD1306 && GUI_OLED_CONTROLLER != GUI_OLED_SH1106
#error Unsupported GUI OLED controller
#endif
#ifndef GUI_OLED_COLUMN_OFFSET
#define GUI_OLED_COLUMN_OFFSET (GUI_OLED_CONTROLLER == GUI_OLED_SH1106 ? 2 : 0)
#endif
/* PCLK2=84MHz -> 1.3125MHz, with margin for both controller families. */
#define GUI_OLED_SPI_DIVIDER 64u
#define GUI_OLED_SPI_PRESCALER SPI_BAUDRATEPRESCALER_64
#define GUI_OLED_POWER_WAIT_MS 300u
#define GUI_OLED_RESET_LOW_MS 20u
#define GUI_OLED_RESET_SETTLE_MS 30u
#define GUI_OLED_REPAINT_MS 2000u
#define GUI_OLED_CONTRAST 0x7fu
#endif
