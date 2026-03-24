/**
 * @file range_dial.h
 *
 * Helper APIs for rendering a vl6180x sensor dial using LVGL.
 * This module creates a small dial that plots the
 * range channel in real time.
 */
#ifndef RANGE_DIAL_H
#define RANGE_DIAL_H

#include <zephyr/drivers/sensor.h>
#include <lvgl.h>
#include <stdint.h>

/**
 * Initialize the range dial.
 *
 * This must be called once before calling update_range_chart(). The returned
 * object is the same parent that was passed in, which allows chaining in the
 * caller if desired.
 *
 * @param parent The LVGL object to contain the chart.
 * @return The same parent object passed in.
 */
lv_obj_t* range_dial_init(lv_obj_t *parent);

/**
 * Push new range samples into the chart.
 *
 * This should be called whenever a new sensor sample is available.
 * Only the integer part (val1) of each sensor_value is used.
 *
 * @param distance Pointer to temperature value.
 */
void update_range_dial(struct sensor_value *distance);

#endif /* RANGE_DIAL_H */


