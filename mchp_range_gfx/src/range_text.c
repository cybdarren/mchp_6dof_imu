
/*
 * Range text display module.
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
#include "range_text.h"

#define LABEL_X_OFFSET 10  /* Horizontal offset for labels from left edge */

static lv_obj_t *screen_text;
static lv_obj_t *label_distance;

lv_obj_t* range_text_init(lv_obj_t *parent)
{
    /* Create the main container for the text display */
    screen_text = lv_obj_create(parent);
    lv_obj_set_style_bg_color(screen_text, lv_color_black(), 0);
    lv_obj_set_size(screen_text, 180, 180);
    lv_obj_center(screen_text);

     /* Create the distance label */
    label_distance = lv_label_create(screen_text);
    lv_obj_set_style_text_color(label_distance, lv_color_white(), 0);
    lv_obj_set_style_text_font(label_distance, &lv_font_montserrat_28, 0);
    lv_label_set_text(label_distance, "-- mm");
    lv_obj_align(label_distance, LV_ALIGN_CENTER, 0, 0);

    return parent;   
}

void update_range_text(struct sensor_value *distance)
{
    char buf[32];
    int distance_mm = distance->val1;
    snprintf(buf, sizeof(buf), "%d mm", distance_mm);
    lv_label_set_text(label_distance, buf);
}

