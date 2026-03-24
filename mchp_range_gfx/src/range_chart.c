
/*
 * A simple LVGL chart implementation that plots lv6180x sensor values
 * (range) over time.
 *
 * The chart is configured as a circular rolling buffer, where new values
 * replace the oldest ones. This gives an "oscilloscope" feel.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <stdio.h>
#include <lvgl.h>
#include <math.h>
#include "range_chart.h"

static lv_obj_t *chart;
static lv_chart_series_t *ser_distance;

lv_obj_t* range_chart_init(lv_obj_t *parent)
{
    /* Create a chart object as a child of the given parent */
    chart = lv_chart_create(parent);

    /* Give the chart a fixed size and center it */
    lv_obj_set_size(chart, 200, 200);
    lv_obj_center(chart);

    /*
     * Use a dark background and minimal border to look like an oscilloscope.
     */
    lv_obj_set_style_bg_color(chart, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(chart, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(chart, 1, 0);
    lv_obj_set_style_border_color(chart, lv_color_hex(0x444444), 0);

    /* Grid lines for reference */
    lv_chart_set_div_line_count(chart, 6, 6);

    lv_obj_set_style_line_color(chart, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_line_width(chart, 1, LV_PART_MAIN);

    /* Use circular update mode so the chart scrolls continuously */
    lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_CIRCULAR);
    lv_obj_set_style_size(chart, 0, 0, LV_PART_INDICATOR);

    /* 120 points per series and a Y range that covers distance */
    lv_chart_set_point_count(chart, 120);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 260);

    /* Add series for distance */
    ser_distance = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    return parent;
}

/**
 * update_range_chart - add new sensor values to the chart
 *
 * This is intended to be called whenever a new range sample is available.
 * It pushes the values into the chart series and then clears a few points
 * ahead of the write cursor to avoid visual ghosting artifacts.
 */
void update_range_chart(struct sensor_value *distance_sv)
{
    /* Use only the integer component (val1) from the sensor_value */
    int32_t distance_val = distance_sv->val1;

    /* Append new samples to each series */
    lv_chart_set_next_value(chart, ser_distance, distance_val);

    /*
     * LVGL leaves old points behind when using circular mode. Clearing a few
     * points ahead of the cursor prevents "ghosting" where remnants of old
     * data remain visible after the chart wraps around.
     */
    uint32_t point_count = lv_chart_get_point_count(chart);
    uint32_t start_idx = lv_chart_get_x_start_point(chart, ser_distance);

    int32_t *sd = lv_chart_get_series_y_array(chart, ser_distance);

    for (int i = 1; i <= 3; i++) {
        sd[(start_idx + i) % point_count] = LV_CHART_POINT_NONE;
    }

    /* Force a redraw so the new points appear immediately */
    lv_chart_refresh(chart);
}
