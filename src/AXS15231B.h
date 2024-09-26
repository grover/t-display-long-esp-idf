#pragma once

#include "stdint.h"
#include "pins_config.h"
#include "lvgl.h"/* https://github.com/lvgl/lvgl.git */

#define TFT_MADCTL 0x36
#define TFT_MAD_MY 0x80
#define TFT_MAD_MX 0x40
#define TFT_MAD_MV 0x20
#define TFT_MAD_ML 0x10
#define TFT_MAD_BGR 0x08
#define TFT_MAD_MH 0x04
#define TFT_MAD_RGB 0x00

#define TFT_CS_H gpio_set_level((gpio_num_t)TFT_QSPI_CS, 1);
#define TFT_CS_L gpio_set_level((gpio_num_t)TFT_QSPI_CS, 0);

typedef struct
{
    uint8_t cmd;
    uint8_t data[36];
    uint8_t len;
    uint32_t sleepTimeInMilliseconds;
} lcd_cmd_t;

void axs15231_init(void);

// Set the display window size
void lcd_address_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void lcd_setRotation(uint8_t r);
void lcd_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void lcd_fill(uint16_t xsta,
              uint16_t ysta,
              uint16_t xend,
              uint16_t yend,
              uint16_t color);
void lcd_PushColors(uint16_t x,
                    uint16_t y,
                    uint16_t width,
                    uint16_t high,
                    uint16_t *data);
void lcd_PushColors(uint16_t *data, uint32_t len);
void lcd_sleep();

bool get_lcd_spi_dma_write(void);
