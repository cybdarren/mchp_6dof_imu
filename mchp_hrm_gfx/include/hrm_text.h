/**
 * @file bme280_text.h
 *
 * Helper APIs for BME280 sensor text display using LVGL.
 * This module creates a simple text-based display for showing real-time
 * sensor values (Temperature/Pressure/Humidity).
 */
#ifndef _HRM_TEXT_H
#define _HRM_TEXT_H

#include <zephyr/drivers/sensor.h>
#include <lvgl.h>
#include <stdint.h>

/**
 * Initialize the HRM Text.
 *
 * This must be called once before calling update_hrm_text(). The returned
 * object is the same parent that was passed in, which allows chaining in the
 * caller if desired.
 *
 * @param parent The LVGL object to contain the text.
 * @return The same parent object passed in.
 */
lv_obj_t* hrm_text_init(lv_obj_t *parent);

/**
 * Push new heart rate samples into the text.
 *
 * This should be called whenever a new heart rate sample is available.
 *
 * @param bpm The current beats per minute value.
 */
void update_hrm_text(int bpm);

#endif /* _HRM_TEXT_H */
