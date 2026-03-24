/*
 * Range dial display module.
 *
 * Displays the current VL6180X range as a dial/gauge using LVGL.
 */
/*
 * Range dial display module for LVGL 9+.
 *
 * Displays VL6180X distance as a large dial using lv_scale.
 */
#include <stdlib.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <lvgl.h>
#include "range_text.h"

static lv_obj_t *screen_dial;
static lv_obj_t *dial_label;
static lv_obj_t *dial_scale;
static lv_obj_t *dial_indicator;

lv_obj_t* range_dial_init(lv_obj_t *parent)
{
    /* Create main container */
    screen_dial = lv_obj_create(parent);
    lv_obj_set_size(screen_dial, 240, 240);
    lv_obj_center(screen_dial);

    /* Create a scale object (acts as a dial) */
    dial_scale = lv_scale_create(screen_dial);
    lv_obj_set_size(dial_scale, 200, 200);
    lv_obj_center(dial_scale);
    lv_obj_set_style_bg_color(dial_scale, lv_color_white(), 0);
    lv_scale_set_range(dial_scale, 0, 210); /* Max range 210 mm */
    lv_scale_set_total_tick_count(dial_scale, 22);
    lv_scale_set_major_tick_every(dial_scale, 2);
    lv_obj_set_style_line_color(dial_scale, lv_palette_main(LV_PALETTE_BLUE), LV_PART_ITEMS);
    lv_obj_set_style_text_font(dial_scale, &lv_font_montserrat_16, LV_PART_INDICATOR);

    lv_scale_set_angle_range(dial_scale, 270);
    lv_scale_set_rotation(dial_scale, 135);
    lv_obj_set_style_transform_rotation(dial_scale, 
        LV_SCALE_LABEL_ROTATE_MATCH_TICKS | LV_SCALE_LABEL_ROTATE_KEEP_UPRIGHT,
        LV_PART_INDICATOR);

    lv_scale_set_mode(dial_scale, LV_SCALE_MODE_ROUND_INNER);

    /* Create the needle / indicator */
    dial_indicator = lv_line_create(dial_scale);
    lv_obj_set_style_line_width(dial_indicator, 3, LV_PART_MAIN);
    lv_obj_set_style_line_rounded(dial_indicator, true, LV_PART_MAIN);
    lv_obj_set_style_line_color(dial_indicator, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);

    /* Add a central label to show numeric value */
    dial_label = lv_label_create(screen_dial);
    lv_obj_set_style_text_color(dial_label, lv_color_black(), 0);
    lv_obj_set_style_text_font(dial_label, &lv_font_montserrat_16, 0);
    lv_label_set_text(dial_label, "-- mm");
    lv_obj_align(dial_label, LV_ALIGN_CENTER, 0, 50);

    return parent;
}

void update_range_dial(struct sensor_value *distance)
{
    char buf[32];
    int distance_mm = distance->val1;

    /* Update the label */
    snprintf(buf, sizeof(buf), "%d mm", distance_mm);
    lv_label_set_text(dial_label, buf);

    /* Update the scale / needle */
    lv_scale_set_line_needle_value(dial_scale, dial_indicator, 60, distance_mm);
}
