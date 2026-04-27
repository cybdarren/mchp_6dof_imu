
/*
 * IMU text display module.
 *
 * This module provides a simple LVGL-based UI for displaying real-time
 * accelerometer values (AX/AY/AZ) as large text labels. It creates a
 * bordered container with three colored labels and separator lines.
 */
#include <stdlib.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>
#include <lvgl.h>
#include <math.h>
#include "imu_text.h"

#define LABEL_X_OFFSET 10  /* Horizontal offset for labels from left edge */

static lv_obj_t *screen_text;
static lv_obj_t *container;

static lv_obj_t *label_ax_big;
static lv_obj_t *label_ay_big;
static lv_obj_t *label_az_big;

lv_obj_t* imu_text_init(lv_obj_t *parent)
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

    /* Create AX label (red, top) */
    label_ax_big = lv_label_create(container);
    lv_obj_set_style_text_color(label_ax_big, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_text_font(label_ax_big, &lv_font_montserrat_28, 0);
    lv_obj_align(label_ax_big, LV_ALIGN_TOP_LEFT, LABEL_X_OFFSET, 4);

    /* Create AY label (green, middle) */
    label_ay_big = lv_label_create(container);
    lv_obj_set_style_text_color(label_ay_big, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_text_font(label_ay_big, &lv_font_montserrat_28, 0);
    lv_obj_align(label_ay_big, LV_ALIGN_LEFT_MID, LABEL_X_OFFSET, 0);

    /* Create AZ label (blue, bottom) */
    label_az_big = lv_label_create(container);
    lv_obj_set_style_text_color(label_az_big, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_set_style_text_font(label_az_big, &lv_font_montserrat_28, 0);
    lv_obj_align(label_az_big, LV_ALIGN_BOTTOM_LEFT, LABEL_X_OFFSET, -4);

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

void update_imu_text(struct sensor_value *ax_sv,
                     struct sensor_value *ay_sv,
                     struct sensor_value *az_sv)
{
    /* Convert sensor values to double for display */
    double ax = sensor_value_to_double(ax_sv);
    double ay = sensor_value_to_double(ay_sv);
    double az = sensor_value_to_double(az_sv);

    /* Update each label with formatted text */
    lv_label_set_text_fmt(label_ax_big, "AX: %6.2f", ax);
    lv_label_set_text_fmt(label_ay_big, "AY: %6.2f", ay);
    lv_label_set_text_fmt(label_az_big, "AZ: %6.2f", az);
}

