#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <stdio.h>
#include <lvgl.h>
#include <math.h>
#include "imu_chart.h"

static lv_obj_t *chart;
static lv_chart_series_t *ser_ax;
static lv_chart_series_t *ser_ay;
static lv_chart_series_t *ser_az;

static lv_obj_t *legend;

/* Value labels */
static lv_obj_t *label_ax;
static lv_obj_t *label_ay;
static lv_obj_t *label_az;

lv_obj_t* imu_chart_init(lv_obj_t *parent)
{
    chart = lv_chart_create(parent);

    lv_obj_set_size(chart, 200, 200);
    lv_obj_center(chart);

    /* oscilloscope style background */
    lv_obj_set_style_bg_color(chart, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(chart, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(chart, 1, 0);
    lv_obj_set_style_border_color(chart, lv_color_hex(0x444444), 0);

    /* Grid lines */
    lv_chart_set_div_line_count(chart, 6, 6);

    lv_obj_set_style_line_color(chart, lv_color_hex(0x333333), LV_PART_MAIN);
    lv_obj_set_style_line_width(chart, 1, LV_PART_MAIN);

    lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_CIRCULAR);
    lv_obj_set_style_size(chart, 0, 0, LV_PART_INDICATOR);

    lv_chart_set_point_count(chart, 100);
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -15, 15);

    ser_ax = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    ser_ay = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    ser_az = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);

        /* ---------------- Legend ---------------- */

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

/* Call every IMU update */
void update_imu_chart(struct sensor_value *ax_sv,
                           struct sensor_value *ay_sv,
                           struct sensor_value *az_sv)
{
    int32_t ax_val = ax_sv->val1;
    int32_t ay_val = ay_sv->val1;
    int32_t az_val = az_sv->val1;

    lv_chart_set_next_value(chart, ser_ax, ax_val);
    lv_chart_set_next_value(chart, ser_ay, ay_val);
    lv_chart_set_next_value(chart, ser_az, az_val);

    // blank out old values to prevent ghosting effect
    uint32_t p = lv_chart_get_point_count(chart);
    uint32_t s = lv_chart_get_x_start_point(chart, ser_ax);

    int32_t *sx = lv_chart_get_series_y_array(chart, ser_ax);
    int32_t *sy = lv_chart_get_series_y_array(chart, ser_ay);   
    int32_t *sz = lv_chart_get_series_y_array(chart, ser_az);

    for(int i=1;i<=3;i++)
    {
        sx[(s + i) % p] = LV_CHART_POINT_NONE;
        sy[(s + i) % p] = LV_CHART_POINT_NONE;
        sz[(s + i) % p] = LV_CHART_POINT_NONE;
    }

    // /* Update live values in legend */
    // lv_label_set_text_fmt(label_ax, "AX: %d", ax_val);
    // lv_label_set_text_fmt(label_ay, "AY: %d", ay_val);
    // lv_label_set_text_fmt(label_az, "AZ: %d", az_val);

    lv_chart_refresh(chart);
}                 
