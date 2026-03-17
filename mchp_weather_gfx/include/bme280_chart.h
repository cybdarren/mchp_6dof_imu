/**
 * @file bme280_chart.h
 *
 * Helper APIs for rendering a rolling BME280 sensor chart using LVGL.
 * This module creates a small oscilloscope-style chart that plots the
 * temperature, humidity, and pressure channels in real time.
 */
#ifndef BME280_CHART_H
#define BME280_CHART_H

#include <zephyr/drivers/sensor.h>
#include <lvgl.h>
#include <stdint.h>

/**
 * Initialize the BME280 chart.
 *
 * This must be called once before calling update_bme280_chart(). The returned
 * object is the same parent that was passed in, which allows chaining in the
 * caller if desired.
 *
 * @param parent The LVGL object to contain the chart.
 * @return The same parent object passed in.
 */
lv_obj_t* bme280_chart_init(lv_obj_t *parent);

/**
 * Push new BME280 samples into the chart.
 *
 * This should be called whenever a new sensor sample is available.
 * Only the integer part (val1) of each sensor_value is used.
 *
 * @param temp_sv Pointer to temperature value.
 * @param hum_sv Pointer to humidity value.
 * @param press_sv Pointer to pressure value.
 */
void update_bme280_chart(struct sensor_value *temp_sv,
                         struct sensor_value *hum_sv,
                         struct sensor_value *press_sv);

#endif /* BME280_CHART_H */
