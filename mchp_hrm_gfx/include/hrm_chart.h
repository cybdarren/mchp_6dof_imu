/**
 * @file hrm_chart.h
 *
 * Helper APIs for rendering a rolling HRM sensor chart using LVGL.
 * This module creates a small oscilloscope-style chart that plots the
 * HRM channel in real time.
 */
#ifndef _HRM_CHART_H
#define _HRM_CHART_H

#include <zephyr/drivers/sensor.h>
#include <lvgl.h>
#include <stdint.h>

/**
 * Initialize the HRM chart.
 *
 * This must be called once before calling update_hrm_chart(). The returned
 * object is the same parent that was passed in, which allows chaining in the
 * caller if desired.
 *
 * @param parent The LVGL object to contain the chart.
 * @return The same parent object passed in.
 */
lv_obj_t* hrm_chart_init(lv_obj_t *parent);

/**
 * Push new filtered AC sample into the ECG chart.
 *
 * This should be called whenever a new sample is available.
 * Pass the filtered AC component (lp_out) for ECG-style display.
 *
 * @param ac_value The filtered AC signal from PPG processing.
 */
void update_hrm_chart(int32_t ac_value);

#endif /* _HRM_CHART_H */
