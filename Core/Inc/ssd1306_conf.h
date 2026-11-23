
#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

// Микроконтроллер STM32F411CEU6
#define STM32F4

// Интерфейс I2C
#define SSD1306_USE_I2C

// Используем I2C1
#define SSD1306_I2C_PORT hi2c1

// Адрес OLED, который мы обнаружили
#define SSD1306_I2C_ADDR (0x3C << 1)

// Размер дисплея
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

// Шрифты
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10

#endif
