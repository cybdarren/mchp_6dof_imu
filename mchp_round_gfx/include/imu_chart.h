/**
 * @file imu_chart.h
 *
 * Helper APIs for rendering a rolling IMU acceleration chart using LVGL.
 * This module creates a small oscilloscope-style chart that plots the
 * X/Y/Z accelerometer channels in real time.
 */
#ifndef IMU_CHART_H
#define IMU_CHART_H

#include <zephyr/drivers/sensor.h>
#include <lvgl.h>
#include <stdint.h>

/**
 * Initialize the IMU chart.
 *
 * This must be called once before calling update_imu_chart(). The returned
 * object is the same parent that was passed in, which allows chaining in the
 * caller if desired.
 *
 * @param parent The LVGL object to contain the chart.
 * @return The same parent object passed in.
 */
lv_obj_t* imu_chart_init(lv_obj_t *parent);

/**
 * Push new IMU samples into the chart.
 *
 * This should be called whenever a new accelerometer sample is available.
 * Only the integer part (val1) of each sensor_value is used.
 *
 * @param ax_sv Pointer to X-axis accelerometer value.
 * @param ay_sv Pointer to Y-axis accelerometer value.
 * @param az_sv Pointer to Z-axis accelerometer value.
 */
void update_imu_chart(struct sensor_value *ax_sv,
                      struct sensor_value *ay_sv,
                      struct sensor_value *az_sv);

#endif /* IMU_CHART_H */
