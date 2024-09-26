#include "lvgl.h"      /* https://github.com/lvgl/lvgl.git */
#include "AXS15231B.h"
#include <Arduino.h>
#include <Wire.h>

//================================
//If you turn on software rotation(disp_drv.sw_rotate = 1), Do not update or replace LVGL.
//disp_drv.full_refresh must be 1
//================================

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;
static lv_color_t *buf1;

uint8_t ALS_ADDRESS = 0x3B;
#define TOUCH_IICSCL 10
#define TOUCH_IICSDA 15
#define TOUCH_INT 11
#define TOUCH_RES 16

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


lv_obj_t *ui_cartext = NULL;

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area,
                   lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    char i = 0;
    while (get_lcd_spi_dma_write())
		{
			i = i >> 1;
			lcd_PushColors(0, 0, 0, 0, NULL);
		}
    
    lcd_PushColors(area->x1, area->y1, w, h, (uint16_t *)&color_p->full);
}

uint8_t read_touchpad_cmd[11] = {0xb5, 0xab, 0xa5, 0x5a, 0x0, 0x0, 0x0, 0x8};
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    uint8_t buff[20] = {0};

    Wire.beginTransmission(0x3B);
    Wire.write(read_touchpad_cmd, 8);
    Wire.endTransmission();
    Wire.requestFrom(0x3B, 8);
    while (!Wire.available());
    Wire.readBytes(buff, 8);

    uint16_t pointX;
    uint16_t pointY;
    uint16_t type = 0;

    type = AXS_GET_GESTURE_TYPE(buff);
    pointX = AXS_GET_POINT_X(buff,0);
    pointY = AXS_GET_POINT_Y(buff,0);

    if (!type && (pointX || pointY)) {
        pointX = (640-pointX);
        if(pointX > 640) pointX = 640;
        if(pointY > 180) pointY = 180;
        data->state = LV_INDEV_STATE_PR;
        data->point.x = pointY;
        data->point.y = pointX;

        char buf[20] = {0};
        sprintf(buf, "(%d, %d)", data->point.x, data->point.y);
        if(ui_cartext != NULL)
        lv_label_set_text(ui_cartext, buf);

        Serial.println(buf);
    }
    else {
        data->state = LV_INDEV_STATE_REL;
    }
}

LV_IMG_DECLARE(test_img);
LV_IMG_DECLARE(test1_180640);
LV_IMG_DECLARE(test3_180640);
LV_IMG_DECLARE(test4_180640);
static const lv_img_dsc_t *imgs[4] = {&test1_180640, &test3_180640,
                                      &test4_180640, &test_img};

void demo_init(lv_obj_t *parent, lv_img_dsc_t **dsc, uint8_t num,
               uint32_t duration, lv_coord_t x, lv_coord_t y, lv_coord_t w,
               lv_coord_t h) {
  lv_obj_t *animimg0 = lv_animimg_create(parent);
  lv_obj_center(animimg0);
  lv_animimg_set_src(animimg0, (const void**)dsc, num);

  lv_animimg_set_duration(animimg0, duration);
  lv_animimg_set_repeat_count(animimg0, 0xffff);

  lv_obj_set_style_pad_all(animimg0, 0, 0);

  lv_obj_set_size(animimg0, w, h);
  lv_animimg_start(animimg0);
}

void demo_animation(void) {
  lv_obj_t *obj = lv_btn_create(lv_scr_act());
  lv_obj_set_style_pad_all(obj, 0, 0);
  lv_obj_set_pos(obj, 0, 0);
  lv_obj_set_size(obj, 180, 640);
  lv_obj_set_style_bg_color(obj, lv_color_hex(0xffffff), LV_PART_MAIN);
  demo_init(obj, (lv_img_dsc_t **)imgs, 4, 12000, 0, 0, 180, 640);
}

bool result = false;
void setup() {
    Serial.begin(115200);
    Serial.println("sta\n");

    pinMode(TOUCH_RES, OUTPUT);
    digitalWrite(TOUCH_RES, HIGH);delay(2);
    digitalWrite(TOUCH_RES, LOW);delay(10);
    digitalWrite(TOUCH_RES, HIGH);delay(2);

    Wire.begin(TOUCH_IICSDA, TOUCH_IICSCL);

    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    axs15231_init();

    lv_init();
    size_t buffer_size =
        sizeof(lv_color_t) * EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES;
    buf = (lv_color_t *)ps_malloc(buffer_size);
    if (buf == NULL) {
      while (1) {
        Serial.println("buf NULL");
        delay(500);
      }
    }

    buf1 = (lv_color_t *)ps_malloc(buffer_size);
    if (buf1 == NULL) {
      while (1) {
        Serial.println("buf NULL");
        delay(500);
      }
    }

    lv_disp_draw_buf_init(&draw_buf, buf, buf1, buffer_size);
    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    //disp_drv.sw_rotate = 1;             //If you turn on software rotation, Do not update or replace LVGL
    //disp_drv.rotated = LV_DISP_ROT_90;  
    disp_drv.full_refresh = 1;          //full_refresh must be 1
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    ui_cartext = lv_label_create(lv_layer_top());
    lv_obj_align(ui_cartext, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_color(ui_cartext, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_text(ui_cartext, "(0, 0)");

    demo_animation();

    Serial.println("end\n");
}

extern uint32_t transfer_num;
extern size_t lcd_PushColors_len;
void loop() {
    delay(1);
    if (transfer_num <= 0 && lcd_PushColors_len <= 0)
        lv_timer_handler();

    if (transfer_num <= 1 && lcd_PushColors_len > 0) {
        lcd_PushColors(0, 0, 0, 0, NULL);
    }
}

