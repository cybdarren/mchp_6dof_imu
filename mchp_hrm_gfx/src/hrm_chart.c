/*
 * ECG-style heart rate chart using LVGL.
 *
 * Displays the filtered PPG signal as a classic hospital monitor trace
 * with a sweeping line on a dark background.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <stdio.h>
#include <lvgl.h>
#include <math.h>
#include "hrm_chart.h"

static lv_obj_t *chart;
static lv_chart_series_t *ser_ecg;

/* Auto-scaling state - tracks signal range for tight zoom */
static float sig_min = 0;
static float sig_max = 100;
static bool first_sample = true;
static int32_t last_y_lo = -100;
static int32_t last_y_hi = 300;

lv_obj_t* hrm_chart_init(lv_obj_t *parent)
{
    /* Black background for parent (ECG monitor style) */
    lv_obj_set_style_bg_color(parent, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);

    /* Create chart */
    chart = lv_chart_create(parent);
    lv_obj_set_size(chart, 230, 180);
    lv_obj_align(chart, LV_ALIGN_CENTER, 0, 10);

    /* Dark background with subtle grid (hospital monitor style) */
    lv_obj_set_style_bg_color(chart, lv_color_hex(0x001000), 0);
    lv_obj_set_style_bg_opa(chart, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(chart, 2, 0);
    lv_obj_set_style_border_color(chart, lv_color_hex(0x004400), 0);
    lv_obj_set_style_radius(chart, 4, 0);

    /* Minimal grid for performance */
    lv_chart_set_div_line_count(chart, 0, 0);

    /* Line chart type */
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);

    /* Circular mode for continuous sweep */
    lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_CIRCULAR);

    /* Hide point indicators - show only the line */
    lv_obj_set_style_size(chart, 0, 0, LV_PART_INDICATOR);

    /* Reduced point count for better performance */
    lv_chart_set_point_count(chart, 100);

    /* Y range for filtered AC signal */
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, -100, 300);

    /* Bright green trace */
    ser_ecg = lv_chart_add_series(chart,
        lv_color_hex(0x00FF00),
        LV_CHART_AXIS_PRIMARY_Y);

    /* Thin line for faster rendering */
    lv_obj_set_style_line_width(chart, 1, LV_PART_ITEMS);

    return parent;
}

/**
 * update_hrm_chart - add new filtered AC sample to the ECG trace
 *
 * @param ac_value The filtered AC component from process_ppg (lp_out)
 */
void update_hrm_chart(int32_t ac_value)
{
    float val = (float)ac_value;

    /*
     * Auto-scaling: Track signal min/max with fast attack, slow decay.
     */
    if (first_sample) {
        sig_min = val - 50;
        sig_max = val + 50;
        first_sample = false;
    } else {
        /* Fast attack - immediately expand to fit new extremes */
        if (val < sig_min) sig_min = val;
        if (val > sig_max) sig_max = val;

        /* Slow decay - gradually shrink range toward signal */
        float center = (sig_min + sig_max) / 2.0f;
        sig_min = sig_min + (center - sig_min) * 0.01f;
        sig_max = sig_max - (sig_max - center) * 0.01f;
    }

    /* Ensure minimum range */
    float range = sig_max - sig_min;
    if (range < 50) {
        float center = (sig_min + sig_max) / 2.0f;
        sig_min = center - 25;
        sig_max = center + 25;
        range = 50;
    }

    /* Calculate new range with margin */
    int32_t margin = (int32_t)(range * 0.15f);
    int32_t y_lo = (int32_t)sig_min - margin;
    int32_t y_hi = (int32_t)sig_max + margin;

    /* Only update range if changed significantly (>5%) - avoids costly redraws */
    if (abs(y_lo - last_y_lo) > (last_y_hi - last_y_lo) / 20 ||
        abs(y_hi - last_y_hi) > (last_y_hi - last_y_lo) / 20) {
        lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, y_lo, y_hi);
        last_y_lo = y_lo;
        last_y_hi = y_hi;
    }

    /* Add the sample - LVGL handles invalidation */
    lv_chart_set_next_value(chart, ser_ecg, ac_value);

    /* Clear points ahead of cursor for sweep effect */
    uint32_t point_count = lv_chart_get_point_count(chart);
    uint32_t cursor_idx = lv_chart_get_x_start_point(chart, ser_ecg);
    int32_t *data = lv_chart_get_series_y_array(chart, ser_ecg);

    for (int i = 1; i <= 4; i++) {
        data[(cursor_idx + i) % point_count] = LV_CHART_POINT_NONE;
    }

    /* Let LVGL handle redraw naturally - no explicit refresh */
}
