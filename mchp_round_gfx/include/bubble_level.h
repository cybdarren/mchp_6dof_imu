#ifndef BUBBLE_LEVEL_H
#define BUBBLE_LEVEL_H

#include <zephyr/drivers/sensor.h>
#include <lvgl.h>
#include <stdint.h>

/* Call once to initialize bubble variables */
lv_obj_t* bubble_level_init(lv_obj_t *parent);

/* Call every IMU update */
void update_bubble_physics(struct sensor_value *ax_sv,
                           struct sensor_value *ay_sv,
                           struct sensor_value *az_sv);                    

#endif /* BUBBLE_LEVEL_H */
