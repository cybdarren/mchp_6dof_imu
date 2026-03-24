#define DT_DRV_COMPAT st_vl6180x

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#include "vl6180x.h"

/* Registers */
#define SYSRANGE_START            0x018
#define SYSTEM_INTERRUPT_CONFIG   0x014
#define SYSTEM_INTERRUPT_CLEAR    0x015
#define SYSTEM_FRESH_OUT_OF_RESET 0x016
#define SYSTEM_MODE_GPIO1         0x011
#define RESULT_RANGE_STATUS       0x04D
#define RESULT_INTERRUPT_STATUS   0x04F
#define RESULT_RANGE_VALUE        0x062

/* --- Low-level I2C --- */

static int write_reg16(const struct i2c_dt_spec *i2c, uint16_t reg, uint8_t val)
{
    uint8_t buf[3] = { reg >> 8, reg & 0xFF, val };
    return i2c_write_dt(i2c, buf, 3);
}

static int read_reg16(const struct i2c_dt_spec *i2c, uint16_t reg, uint8_t *val)
{
    uint8_t addr[2] = { reg >> 8, reg & 0xFF };
    return i2c_write_read_dt(i2c, addr, 2, val, 1);
}

/* --- GPIO interrupt callback --- */

static void vl6180x_gpio_callback(const struct device *port,
                                  struct gpio_callback *cb,
                                  uint32_t pins)
{
    struct vl6180x_data *data =
        CONTAINER_OF(cb, struct vl6180x_data, gpio_cb);

    k_sem_give(&data->data_ready);
}

/* --- Sensor init --- */
static int vl6180x_init(const struct device *dev)
{
    const struct vl6180x_config *cfg = dev->config;
    struct vl6180x_data *data = dev->data;

    uint8_t reset;

    /* configure the enable pin and reset the device to a known state*/
    if (cfg->en_gpio.port && device_is_ready(cfg->en_gpio.port)) {
        const struct gpio_dt_spec en_gpio = cfg->en_gpio;
        const struct device* port = en_gpio.port;

        gpio_pin_configure(port, en_gpio.pin, GPIO_OUTPUT);
        gpio_pin_set(port, en_gpio.pin, 1);
        k_msleep(2);
        gpio_pin_set(port, en_gpio.pin, 0);     // Pull low to reset (pin is active low)
        k_msleep(10);                           // Hold 10 ms
        gpio_pin_set(port, en_gpio.pin, 1);     // Release
        k_msleep(2);                            // Wait for stabilization
    }

    if (!device_is_ready(cfg->i2c.bus)) {
        printk("VL6180X: I2C bus not ready\n");
        return -ENODEV;
    }

    data->dev = dev;
    k_sem_init(&data->data_ready, 0, 1);

    /* Check reset state */
    read_reg16(&cfg->i2c, SYSTEM_FRESH_OUT_OF_RESET, &reset);

    if (reset) {
        /* Minimal ST init sequence */
        write_reg16(&cfg->i2c, 0x0207, 0x01);
        write_reg16(&cfg->i2c, 0x0208, 0x01);
        write_reg16(&cfg->i2c, 0x0096, 0x00);
        write_reg16(&cfg->i2c, 0x0097, 0xfd);
        write_reg16(&cfg->i2c, 0x00e3, 0x00);
        write_reg16(&cfg->i2c, 0x00e4, 0x04);
        write_reg16(&cfg->i2c, 0x00e5, 0x02);
        write_reg16(&cfg->i2c, 0x00e6, 0x01);
        write_reg16(&cfg->i2c, 0x00e7, 0x03);
        write_reg16(&cfg->i2c, 0x00f5, 0x02);
        write_reg16(&cfg->i2c, 0x00d9, 0x05);
        write_reg16(&cfg->i2c, 0x00db, 0xce);
        write_reg16(&cfg->i2c, 0x00dc, 0x03);
        write_reg16(&cfg->i2c, 0x00dd, 0xf8);
        write_reg16(&cfg->i2c, 0x009f, 0x00);
        write_reg16(&cfg->i2c, 0x00a3, 0x3c);
        write_reg16(&cfg->i2c, 0x00b7, 0x00);
        write_reg16(&cfg->i2c, 0x00bb, 0x3c);
        write_reg16(&cfg->i2c, 0x00b2, 0x09);
        write_reg16(&cfg->i2c, 0x00ca, 0x09);
        write_reg16(&cfg->i2c, 0x0198, 0x01);
        write_reg16(&cfg->i2c, 0x01b0, 0x17);
        write_reg16(&cfg->i2c, 0x01ad, 0x00);
        write_reg16(&cfg->i2c, 0x00ff, 0x05);
        write_reg16(&cfg->i2c, 0x0100, 0x05);
        write_reg16(&cfg->i2c, 0x0199, 0x05);
        write_reg16(&cfg->i2c, 0x01a6, 0x1b);
        write_reg16(&cfg->i2c, 0x01ac, 0x3e);
        write_reg16(&cfg->i2c, 0x01a7, 0x1f);
        write_reg16(&cfg->i2c, 0x0030, 0x00);

        /* additional public registers */
        //write_reg16(&cfg->i2c, 0x0011, 0x10); 
        write_reg16(&cfg->i2c, 0x010a, 0x30);
        write_reg16(&cfg->i2c, 0x003f, 0x46);
        write_reg16(&cfg->i2c, 0x0031, 0xFF);
        write_reg16(&cfg->i2c, 0x0040, 0x63);
        write_reg16(&cfg->i2c, 0x002e, 0x01);
        
        /* optional public registers */
        write_reg16(&cfg->i2c, 0x001b, 0x09);
        write_reg16(&cfg->i2c, 0x003e, 0x31);
        write_reg16(&cfg->i2c, 0x0014, 0x24);

        write_reg16(&cfg->i2c, SYSTEM_FRESH_OUT_OF_RESET, 0x00);
    }

    /* Configure interrupt GPIO setup */
    if (cfg->int_gpio.port && device_is_ready(cfg->int_gpio.port)) {
        gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);

        gpio_init_callback(&data->gpio_cb,
                           vl6180x_gpio_callback,
                           BIT(cfg->int_gpio.pin));

        gpio_add_callback(cfg->int_gpio.port, &data->gpio_cb);        
        
        write_reg16(&cfg->i2c, SYSTEM_INTERRUPT_CLEAR, 0x07);
        
        gpio_pin_interrupt_configure_dt(&cfg->int_gpio,
                                        GPIO_INT_EDGE_TO_ACTIVE);
    }
    
    /* Configure continuous mode if enabled */
    if (cfg->continuous) {
        write_reg16(&cfg->i2c, SYSTEM_MODE_GPIO1, 0x10);       /* GPIO1 interrupt, active low */
        write_reg16(&cfg->i2c, SYSTEM_INTERRUPT_CONFIG, 0x04); /* new sample ready interrupt */
        write_reg16(&cfg->i2c, SYSTEM_INTERRUPT_CLEAR, 0x07);
        write_reg16(&cfg->i2c, SYSRANGE_START, 0x03);          /* continuous */
    }

    printk("VL6180X init OK\n");
    return 0;
}

/* --- Sample fetch --- */
static int vl6180x_sample_fetch(const struct device *dev,
                                enum sensor_channel chan)
{
    struct vl6180x_data *data = dev->data;
    const struct vl6180x_config *cfg = dev->config;

    /* Continuous mode with interrupt */
    if (cfg->continuous && cfg->int_gpio.port) {
        if (k_sem_take(&data->data_ready, K_MSEC(10)) != 0) {
            return -ETIMEDOUT;
        }

        read_reg16(&cfg->i2c, RESULT_RANGE_VALUE, &data->distance);
        write_reg16(&cfg->i2c, SYSTEM_INTERRUPT_CLEAR, 0x07);

        return 0;
    }

    /* Fallback: single-shot polling */
    uint8_t status;

    write_reg16(&cfg->i2c, SYSRANGE_START, 0x01);

    do {
        k_msleep(10);
        read_reg16(&cfg->i2c, RESULT_RANGE_STATUS, &status);
    } while ((status & 0x01) == 0);

    read_reg16(&cfg->i2c, RESULT_RANGE_VALUE, &data->distance);
    write_reg16(&cfg->i2c, SYSTEM_INTERRUPT_CLEAR, 0x07);

    return 0;
}

/* --- Channel get --- */
static int vl6180x_channel_get(const struct device *dev,
                               enum sensor_channel chan,
                               struct sensor_value *val)
{
    struct vl6180x_data *data = dev->data;

    if (chan != SENSOR_CHAN_DISTANCE) {
        return -ENOTSUP;
    }

    val->val1 = data->distance;
    val->val2 = 0;

    return 0;
}

/* --- API --- */
static const struct sensor_driver_api vl6180x_api = {
    .sample_fetch = vl6180x_sample_fetch,
    .channel_get = vl6180x_channel_get,
};

/* --- Instance macro --- */
#define VL6180X_DEFINE(inst)                                      \
    static struct vl6180x_data vl6180x_data_##inst;               \
    static const struct vl6180x_config vl6180x_config_##inst = {  \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                        \
        .int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}), \
        .en_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, {0}), \
        .continuous = DT_INST_NODE_HAS_PROP(inst, continuous_mode), \
    };                                                           \
    DEVICE_DT_INST_DEFINE(inst,                                  \
                          vl6180x_init,                          \
                          NULL,                                  \
                          &vl6180x_data_##inst,                  \
                          &vl6180x_config_##inst,                \
                          POST_KERNEL,                           \
                          CONFIG_SENSOR_INIT_PRIORITY,           \
                          &vl6180x_api);

DT_INST_FOREACH_STATUS_OKAY(VL6180X_DEFINE)
