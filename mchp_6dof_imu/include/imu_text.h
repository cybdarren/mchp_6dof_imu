/**
 * @file imu_text.h
 *
 * Helper APIs for IMU acceleration text display using LVGL.
 * This module creates a simple text-based display for showing real-time
 * accelerometer values (AX/AY/AZ).
 */
#ifndef IMU_TEXT_H
#define IMU_TEXT_H

#include <zephyr/drivers/sensor.h>
#include <lvgl.h>
#include <stdint.h>

/**
 * Initialize the IMU Text.
 *
 * This must be called once before calling update_imu_text(). The returned
 * object is the same parent that was passed in, which allows chaining in the
 * caller if desired.
 *
 * @param parent The LVGL object to contain the text.
 * @return The same parent object passed in.
 */
lv_obj_t* imu_text_init(lv_obj_t *parent);

/**
 * Push new IMU samples into the text.
 *
 * This should be called whenever a new accelerometer sample is available.
 *
 * @param ax_sv Pointer to X-axis accelerometer value.
 * @param ay_sv Pointer to Y-axis accelerometer value.
 * @param az_sv Pointer to Z-axis accelerometer value.
 */
void update_imu_text(struct sensor_value *ax_sv,
                      struct sensor_value *ay_sv,
                      struct sensor_value *az_sv);

#endif /* IMU_TEXT_H */
