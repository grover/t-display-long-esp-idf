
#include "backlight.h"

#include "pins_config.h"

#include <driver/gpio.h>

void _backlight_init_pins() {
    const uint64_t outputPins 
        = (1ULL << TFT_BL);

    gpio_config_t conf = {
        .pin_bit_mask = outputPins,              /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
        .mode = GPIO_MODE_OUTPUT,                  /*!< GPIO mode: set input/output mode                     */
        .pull_up_en = GPIO_PULLUP_DISABLE,          /*!< GPIO pull-up                                         */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,      /*!< GPIO pull-down                                       */
        .intr_type = GPIO_INTR_DISABLE              /*!< GPIO interrupt type - previously set                 */
    };

    ESP_ERROR_CHECK(gpio_config(&conf));
}

void backlight_init(void) {
    _backlight_init_pins();
}

void backlight_enable(bool on) {
    gpio_set_level(TFT_BL, on ? 1 : 0);
}
