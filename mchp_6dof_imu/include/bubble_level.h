/**
 * @file bubble_level.h
 *
 * Simple bubble-level visualization using LVGL.
 *
 * This module provides a small UI widget that simulates a physical bubble
 * level using accelerometer data. It can be updated continuously from IMU
 * readings to reflect device orientation.
 */
#ifndef BUBBLE_LEVEL_H
#define BUBBLE_LEVEL_H

#include <zephyr/drivers/sensor.h>
#include <lvgl.h>
#include <stdint.h>

/**
 * Initialize the bubble level UI.
 *
 * This must be called once before calling update_bubble_physics(). The
 * returned object is the same parent passed in and allows chaining configuration
 * in the caller.
 *
 * @param parent The LVGL parent object to contain the bubble level widget.
 * @return The same parent object passed in.
 */
lv_obj_t* bubble_level_init(lv_obj_t *parent);

/**
 * Update the bubble level simulation using new accelerometer samples.
 *
 * Only the integer component (val1) of each sensor_value is used.
 *
 * @param ax_sv X-axis accelerometer sample.
 * @param ay_sv Y-axis accelerometer sample.
 * @param az_sv Z-axis accelerometer sample.
 */
void update_bubble_physics(struct sensor_value *ax_sv,
                           struct sensor_value *ay_sv,
                           struct sensor_value *az_sv);

#endif /* BUBBLE_LEVEL_H */
