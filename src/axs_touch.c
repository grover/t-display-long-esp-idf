

#include <stdint.h>

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <hal/i2c_ll.h>

#include <esp_log.h>
#include <esp_check.h>

#include <FreeRTOS.h>
#include <freertos/task.h>

#include "axs_touch.h"

#include "pins_config.h"

static const char *TAG = "AXSTouchDriver";


static uint8_t ALS_ADDRESS = 0x3B;
#define TOUCH_IICSCL GPIO_NUM_10
#define TOUCH_IICSDA GPIO_NUM_15
#define TOUCH_I2C_FREQUENCY 100000UL

#define TOUCH_INT GPIO_NUM_11
#define TOUCH_RES GPIO_NUM_16

#define AXS_TOUCH_ONE_POINT_LEN             6
#define AXS_TOUCH_BUF_HEAD_LEN              2

#define AXS_TOUCH_GESTURE_POS               0
#define AXS_TOUCH_POINT_NUM                 1
#define AXS_TOUCH_EVENT_POS                 2
#define AXS_TOUCH_X_H_POS                   2
#define AXS_TOUCH_X_L_POS                   3
#define AXS_TOUCH_ID_POS                    4
#define AXS_TOUCH_Y_H_POS                   4
#define AXS_TOUCH_Y_L_POS                   5
#define AXS_TOUCH_WEIGHT_POS                6
#define AXS_TOUCH_AREA_POS                  7

#define AXS_GET_POINT_NUM(buf) buf[AXS_TOUCH_POINT_NUM]
#define AXS_GET_GESTURE_TYPE(buf)  buf[AXS_TOUCH_GESTURE_POS]
#define AXS_GET_POINT_X(buf,point_index) (((uint16_t)(buf[AXS_TOUCH_ONE_POINT_LEN*point_index+AXS_TOUCH_X_H_POS] & 0x0F) <<8) + (uint16_t)buf[AXS_TOUCH_ONE_POINT_LEN*point_index+AXS_TOUCH_X_L_POS])
#define AXS_GET_POINT_Y(buf,point_index) (((uint16_t)(buf[AXS_TOUCH_ONE_POINT_LEN*point_index+AXS_TOUCH_Y_H_POS] & 0x0F) <<8) + (uint16_t)buf[AXS_TOUCH_ONE_POINT_LEN*point_index+AXS_TOUCH_Y_L_POS])
#define AXS_GET_POINT_EVENT(buf,point_index) (buf[AXS_TOUCH_ONE_POINT_LEN*point_index+AXS_TOUCH_EVENT_POS] >> 6)

static void _touch_init_gpios(void) {
    const uint64_t outputPins 
        = (1ULL << TOUCH_RES);

    gpio_config_t conf = {
        .pin_bit_mask = outputPins,              /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
        .mode = GPIO_MODE_OUTPUT,                  /*!< GPIO mode: set input/output mode                     */
        .pull_up_en = GPIO_PULLUP_DISABLE,          /*!< GPIO pull-up                                         */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,      /*!< GPIO pull-down                                       */
        .intr_type = GPIO_INTR_DISABLE              /*!< GPIO interrupt type - previously set                 */
    };

    ESP_ERROR_CHECK(gpio_config(&conf));
}

static void _cpu_sleep(uint32_t sleepDurationInMilliseconds) {
    vTaskDelay(sleepDurationInMilliseconds / portTICK_PERIOD_MS);
}

static void _touch_reset(void) {
    gpio_set_level(TOUCH_RES, 1);
    _cpu_sleep(2);
    gpio_set_level(TOUCH_RES, 0);
    _cpu_sleep(10);
    gpio_set_level(TOUCH_RES, 1);
    _cpu_sleep(2);
}

static esp_err_t _touch_init_i2c(void) {
    i2c_config_t conf = { 
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TOUCH_IICSDA,
        .scl_io_num = TOUCH_IICSCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = TOUCH_I2C_FREQUENCY,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL //Any one clock source that is available for the specified frequency may be choosen
    };

    const i2c_port_t port = I2C_NUM_0; // TODO: Init parameter!
    ESP_RETURN_ON_ERROR(i2c_param_config(port, &conf), TAG, "Failed to configure I2C params");
    ESP_RETURN_ON_ERROR(i2c_driver_install(port, conf.mode, 0, 0, 0), TAG, "Failed to install I2C driver");
    ESP_RETURN_ON_ERROR(i2c_set_timeout(port, I2C_LL_MAX_TIMEOUT), TAG, "Failed to set the I2C timeout");

    return ESP_OK;
}

void touch_init(void) {
    _touch_init_gpios();
    _touch_reset();
    _touch_init_i2c();
}

static uint8_t read_touchpad_cmd[] = {0xb5, 0xab, 0xa5, 0x5a, 0x0, 0x0, 0x0, 0x8};

esp_err_t touch_read_input(touch_input_t *input) {
    const i2c_port_t port = I2C_NUM_0; // TODO: Init parameter!
    const uint8_t address = ALS_ADDRESS; // TODO: Move to struct
    const uint32_t timeOutInMillis = 50;

    uint8_t buff[20] = {0};

    ESP_RETURN_ON_ERROR(i2c_master_write_read_device(port, address, read_touchpad_cmd, sizeof(read_touchpad_cmd), buff, 8, timeOutInMillis / portTICK_RATE_MS), TAG, "Failed to transact with touch driver");

    input->gesture = AXS_GET_GESTURE_TYPE(buff);
    input->x = AXS_GET_POINT_X(buff,0);
    input->y = AXS_GET_POINT_Y(buff,0);

    return ESP_OK;
}
