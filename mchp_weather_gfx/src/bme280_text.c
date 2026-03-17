
/*
 * BME280 text display module.
 *
 * This module provides a simple LVGL-based UI for displaying real-time
 * BME280 sensor values (Temperature/Pressure/Humidity) as large text labels. It creates a
 * bordered container with three colored labels and separator lines.
 */
#include <stdlib.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>
#include <lvgl.h>
#include <math.h>
#include "bme280_text.h"

#define LABEL_X_OFFSET 10  /* Horizontal offset for labels from left edge */

static lv_obj_t *screen_text;
static lv_obj_t *container;

static lv_obj_t *label_temperature;
static lv_obj_t *label_pressure;
static lv_obj_t *label_humidity;

lv_obj_t* bme280_text_init(lv_obj_t *parent)
{
    /* Create the main container for the text display */
    screen_text = lv_obj_create(parent);
    lv_obj_set_style_bg_color(screen_text, lv_color_black(), 0);
    lv_obj_set_size(screen_text, 180, 180);
    lv_obj_center(screen_text);

    /* Inner container with border and rounded corners */
    container = lv_obj_create(screen_text);
    lv_obj_set_size(container, 180, 180);
    lv_obj_center(container);

    lv_obj_set_style_bg_color(container, lv_color_black(), 0);
    lv_obj_set_style_border_color(container, lv_color_hex(0x444444), 0);
    lv_obj_set_style_border_width(container, 2, 0);
    lv_obj_set_style_radius(container, 10, 0);

    /* Create temperature label (red, top) */
    label_temperature = lv_label_create(container);
    lv_obj_set_style_text_color(label_temperature, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_text_font(label_temperature, &lv_font_montserrat_28, 0);
    lv_obj_align(label_temperature, LV_ALIGN_TOP_LEFT, LABEL_X_OFFSET, 4);

    /* Create pressure label (green, middle) */
    label_pressure = lv_label_create(container);
    lv_obj_set_style_text_color(label_pressure, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(label_pressure, &lv_font_montserrat_28, 0);
    lv_obj_align(label_pressure, LV_ALIGN_LEFT_MID, LABEL_X_OFFSET, 0);

    /* Create humidity label (blue, bottom) */
    label_humidity = lv_label_create(container);
    lv_obj_set_style_text_color(label_humidity, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_set_style_text_font(label_humidity, &lv_font_montserrat_28, 0);
    lv_obj_align(label_humidity, LV_ALIGN_BOTTOM_LEFT, LABEL_X_OFFSET, -4);

    /* Add horizontal separator lines between labels */
    lv_obj_t *line1 = lv_obj_create(container);
    lv_obj_set_size(line1, 160, 1);
    lv_obj_align(line1, LV_ALIGN_CENTER, 0, -30);
    lv_obj_set_style_bg_color(line1, lv_color_hex(0x444444), 0);
    lv_obj_clear_flag(line1, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *line2 = lv_obj_create(container);
    lv_obj_set_size(line2, 160, 1);
    lv_obj_align(line2, LV_ALIGN_CENTER, 0, 30);
    lv_obj_set_style_bg_color(line2, lv_color_hex(0x444444), 0);
    lv_obj_clear_flag(line2, LV_OBJ_FLAG_SCROLLABLE);

    return parent;
}

void update_bme280_text(struct sensor_value *temperature,
                        struct sensor_value *pressure,
                        struct sensor_value *humidity)
{
    /* Convert sensor values to double for display */
    double temp = sensor_value_to_double(temperature);
    double pres = sensor_value_to_double(pressure);
    double hum = sensor_value_to_double(humidity);

    /* Update each label with formatted text */
    lv_label_set_text_fmt(label_temperature, "T: %6.2f", temp);
    lv_label_set_text_fmt(label_pressure, "P: %6.2f", pres);
    lv_label_set_text_fmt(label_humidity, "H: %6.2f", hum);
}

