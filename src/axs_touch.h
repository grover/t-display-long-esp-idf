
#pragma once

#include <stdint.h>

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct touch_input {
    uint8_t gesture;
    uint16_t x;
    uint16_t y;
} touch_input_t;

void touch_init(void);
esp_err_t touch_read_input(touch_input_t *input);

#ifdef __cplusplus
}
#endif