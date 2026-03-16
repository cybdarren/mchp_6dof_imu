#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <stdio.h>
#include <lvgl.h>
#include <math.h>
#include "imu_chart.h"

/*
 * A simple LVGL chart implementation that plots IMU acceleration
 * values (AX/AY/AZ) over time.
 *
 * The chart is configured as a circular rolling buffer, where new values
 * replace the oldest ones.  This gives an "oscilloscope" feel.
 */

static lv_obj_t *chart;
static lv_chart_series_t *ser_ax;
static lv_chart_series_t *ser_ay;
static lv_chart_series_t *ser_az;

static lv_obj_t *legend;

/* Optional labels for live values (currently disabled) */
// static lv_obj_t *label_ax;
// static lv_obj_t *label_ay;
// static lv_obj_t *label_az;

lv_obj_t* imu_chart_init(lv_obj_t *parent)
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

    /* 200 points per series, with a Y range matching normal accel values */
    lv_chart_set_point_count(chart, 120);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -15, 15);

    /* Add three series for X/Y/Z axes, each with a distinct color */
    ser_ax = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    ser_ay = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    ser_az = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);

    /*
     * Legend (disabled): A small overlay showing current values for each axis.
     * Uncomment below to enable.
     */

    // /* Legend container */
    // legend = lv_obj_create(parent);
    // lv_obj_set_style_bg_opa(legend, LV_OPA_TRANSP, 0);
    // lv_obj_set_style_border_width(legend, 0, 0);
    // lv_obj_set_size(legend, 180, 20);
    // lv_obj_align(legend, LV_ALIGN_TOP_MID, 0, 4);

    // /* Labels */
    // label_ax = lv_label_create(legend);
    // lv_label_set_text(label_ax, "AX");
    // lv_obj_set_style_text_color(label_ax, lv_palette_main(LV_PALETTE_RED), 0);
    // lv_obj_align(label_ax, LV_ALIGN_LEFT_MID, 0, 0);

    // label_ay = lv_label_create(legend);
    // lv_label_set_text(label_ay, "AY");
    // lv_obj_set_style_text_color(label_ay, lv_palette_main(LV_PALETTE_GREEN), 0);
    // lv_obj_align(label_ay, LV_ALIGN_CENTER, 0, 0);

    // label_az = lv_label_create(legend);
    // lv_label_set_text(label_az, "AZ");
    // lv_obj_set_style_text_color(label_az, lv_palette_main(LV_PALETTE_BLUE), 0);
    // lv_obj_align(label_az, LV_ALIGN_RIGHT_MID, 0, 0);

    return parent;
}

/**
 * update_imu_chart - add new accelerometer values to the chart
 *
 * This is intended to be called whenever a new IMU sample is available.
 * It pushes the values into the chart series and then clears a few points
 * ahead of the write cursor to avoid visual ghosting artifacts.
 */
void update_imu_chart(struct sensor_value *ax_sv,
                           struct sensor_value *ay_sv,
                           struct sensor_value *az_sv)
{
    /* Use only the integer component (val1) from the sensor_value */
    int32_t ax_val = ax_sv->val1;
    int32_t ay_val = ay_sv->val1;
    int32_t az_val = az_sv->val1;

    /* Append new samples to each series */
    lv_chart_set_next_value(chart, ser_ax, ax_val);
    lv_chart_set_next_value(chart, ser_ay, ay_val);
    lv_chart_set_next_value(chart, ser_az, az_val);

    /*
     * LVGL leaves old points behind when using circular mode.  Clearing a few
     * points ahead of the cursor prevents "ghosting" where remnants of old
     * data remain visible after the chart wraps around.
     */
    uint32_t point_count = lv_chart_get_point_count(chart);
    uint32_t start_idx = lv_chart_get_x_start_point(chart, ser_ax);

    int32_t *sx = lv_chart_get_series_y_array(chart, ser_ax);
    int32_t *sy = lv_chart_get_series_y_array(chart, ser_ay);
    int32_t *sz = lv_chart_get_series_y_array(chart, ser_az);

    for (int i = 1; i <= 3; i++) {
        sx[(start_idx + i) % point_count] = LV_CHART_POINT_NONE;
        sy[(start_idx + i) % point_count] = LV_CHART_POINT_NONE;
        sz[(start_idx + i) % point_count] = LV_CHART_POINT_NONE;
    }

    /* Optional: update the legend with live values (currently commented out) */
    // lv_label_set_text_fmt(label_ax, "AX: %d", ax_val);
    // lv_label_set_text_fmt(label_ay, "AY: %d", ay_val);
    // lv_label_set_text_fmt(label_az, "AZ: %d", az_val);

    /* Force a redraw so the new points appear immediately */
    lv_chart_refresh(chart);
}
