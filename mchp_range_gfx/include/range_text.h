/**
 * @file bme280_text.h
 *
 * Helper APIs for BME280 sensor text display using LVGL.
 * This module creates a simple text-based display for showing real-time
 * sensor values (Temperature/Pressure/Humidity).
 */
#ifndef RANGE_TEXT_H
#define RANGE_TEXT_H

#include <zephyr/drivers/sensor.h>
#include <lvgl.h>
#include <stdint.h>

/**
 * Initialize the Range Text.
 *
 * This must be called once before calling update_range_text(). The returned
 * object is the same parent that was passed in, which allows chaining in the
 * caller if desired.
 *
 * @param parent The LVGL object to contain the text.
 * @return The same parent object passed in.
 */
lv_obj_t* range_text_init(lv_obj_t *parent);

/**
 * Push new range samples into the text.
 *
 * This should be called whenever a new range sample is available.
 *
 * @param distance Pointer to distance value.
 */
void update_range_text(struct sensor_value *distance);

#endif /* RANGE_TEXT_H */
