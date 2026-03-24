#ifndef ZEPHYR_DRIVERS_SENSOR_VL6180X_H_
#define ZEPHYR_DRIVERS_SENSOR_VL6180X_H_

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

/* Device configuration */
struct vl6180x_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
    struct gpio_dt_spec en_gpio;
    bool continuous;
};

/* Runtime data */
struct vl6180x_data {
    uint8_t distance;

    const struct device *dev;

    /* Interrupt handling */
    struct gpio_callback gpio_cb;
    struct k_sem data_ready;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_VL6180X_H_ */
