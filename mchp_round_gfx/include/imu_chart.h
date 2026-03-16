#ifndef IMU_CHART_H
#define IMU_CHART_H

#include <zephyr/drivers/sensor.h>
#include <lvgl.h>
#include <stdint.h>

/* Call once to initialize bubble variables */
lv_obj_t* imu_chart_init(lv_obj_t *parent);

/* Call every IMU update */
void update_imu_chart(struct sensor_value *ax_sv,
                           struct sensor_value *ay_sv,
                           struct sensor_value *az_sv);                    

#endif /* IMU_CHART_H */
