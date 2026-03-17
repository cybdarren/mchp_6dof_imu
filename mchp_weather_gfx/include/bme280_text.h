/**
 * @file bme280_text.h
 *
 * Helper APIs for BME280 sensor text display using LVGL.
 * This module creates a simple text-based display for showing real-time
 * sensor values (Temperature/Pressure/Humidity).
 */
#ifndef BME280_TEXT_H
#define BME280_TEXT_H

#include <zephyr/drivers/sensor.h>
#include <lvgl.h>
#include <stdint.h>

/**
 * Initialize the BME280 Text.
 *
 * This must be called once before calling update_bme280_text(). The returned
 * object is the same parent that was passed in, which allows chaining in the
 * caller if desired.
 *
 * @param parent The LVGL object to contain the text.
 * @return The same parent object passed in.
 */
lv_obj_t* bme280_text_init(lv_obj_t *parent);

/**
 * Push new BME280 samples into the text.
 *
 * This should be called whenever a new BME280 sample is available.
 *
 * @param temperature Pointer to temperature value.
 * @param pressure Pointer to pressure value.
 * @param humidity Pointer to humidity value.
 */
void update_bme280_text(struct sensor_value *temperature,
                        struct sensor_value *pressure,
                        struct sensor_value *humidity);

#endif /* BME280_TEXT_H */
