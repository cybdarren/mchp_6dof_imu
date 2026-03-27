#define DT_DRV_COMPAT ti_afe4404

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#include "afe4404.h"

/* --- Low-level I2C --- */
static int write_reg24(const struct i2c_dt_spec *i2c, uint8_t reg, uint32_t val)
{
    /* AFE4404 registers are 24-bit: [addr][MSB][MID][LSB]. */
    uint8_t buf[4];
    buf[0] = reg;
    buf[1] = val >> 16;
    buf[2] = val >> 8;
    buf[3] = val;
    return i2c_write_dt(i2c, buf, 4);
}

static int read_reg24(const struct i2c_dt_spec *i2c, uint8_t reg, uint32_t *val)
{
    uint8_t hr_storage[5];
    uint8_t buf[2];
    uint32_t return_value = 0;
    int rval;

    buf[0] = reg;
    rval = i2c_write_read_dt(i2c, buf, 1, hr_storage, 3);
    
    /* Rebuild host-endian 24-bit value from the device byte stream. */
    return_value = hr_storage[0];
    return_value = return_value << 16;
    return_value |= hr_storage[1] << 8;
    return_value |= hr_storage[2];
    *val = return_value;

    return rval;
}

/* --- GPIO interrupt callback --- */
static void afe4404_gpio_callback(const struct device *port,
                                  struct gpio_callback *cb,
                                  uint32_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(pins);

    struct afe4404_data *data =
        CONTAINER_OF(cb, struct afe4404_data, gpio_cb);

    /* Wake the sampling thread waiting for DRDY. */
    k_sem_give(&data->data_ready);
}

/* --- Sensor init --- */
static int afe4404_init(const struct device *dev)
{
    const struct afe4404_config *cfg = dev->config;
    struct afe4404_data *data = dev->data;

    /* configure the enable pin and reset the device to a known state*/
    if (cfg->rst_gpio.port && device_is_ready(cfg->rst_gpio.port)) {
        const struct gpio_dt_spec rst_gpio = cfg->rst_gpio;
        const struct device* port = rst_gpio.port;

        gpio_pin_configure(port, rst_gpio.pin, GPIO_OUTPUT);
        gpio_pin_set(port, rst_gpio.pin, 1);
        k_msleep(2);
        gpio_pin_set(port, rst_gpio.pin, 0);     // Pull low to reset (pin is active low)
        k_msleep(10);                           // Hold 10 ms
        gpio_pin_set(port, rst_gpio.pin, 1);     // Release
        k_msleep(2);                            // Wait for stabilization
    }

    if (!device_is_ready(cfg->i2c.bus)) {
        printk("AFE4404: I2C bus not ready\n");
        return -ENODEV;
    }

    data->dev = dev;
    k_sem_init(&data->data_ready, 0, 1);

    /*
     * Program timing/control registers for the HRM use case.
     * These values define LED/ADC phases, conversion windows, and clocking.
     */
    write_reg24( &cfg->i2c, AFE4404_REG0H, 0x000000 );
    write_reg24( &cfg->i2c, AFE4404_REG1H, 0x000050 );
    write_reg24( &cfg->i2c, AFE4404_REG2H, 0x00018F );
    write_reg24( &cfg->i2c, AFE4404_REG3H, 0x000320 );
    write_reg24( &cfg->i2c, AFE4404_REG4H, 0x0004AF );
    write_reg24( &cfg->i2c, AFE4404_REG5H, 0x0001E0 );
    write_reg24( &cfg->i2c, AFE4404_REG6H, 0x00031F );
    write_reg24( &cfg->i2c, AFE4404_REG7H, 0x000370 );
    write_reg24( &cfg->i2c, AFE4404_REG8H, 0x0004AF );
    write_reg24( &cfg->i2c, AFE4404_REG9H, 0x000000 );
    write_reg24( &cfg->i2c, AFE4404_REGAH, 0x00018F );
    write_reg24( &cfg->i2c, AFE4404_REGBH, 0x0004FF );
    write_reg24( &cfg->i2c, AFE4404_REGCH, 0x00063E );
    write_reg24( &cfg->i2c, AFE4404_REGDH, 0x000198 );
    write_reg24( &cfg->i2c, AFE4404_REGEH, 0x0005BB );
    write_reg24( &cfg->i2c, AFE4404_REGFH, 0x0005C4 );
    write_reg24( &cfg->i2c, AFE4404_REG10H, 0x0009E7 );
    write_reg24( &cfg->i2c, AFE4404_REG11H, 0x0009F0 );
    write_reg24( &cfg->i2c, AFE4404_REG12H, 0x000E13 );
    write_reg24( &cfg->i2c, AFE4404_REG13H, 0x000E1C );
    write_reg24( &cfg->i2c, AFE4404_REG14H, 0x00123F );
    write_reg24( &cfg->i2c, AFE4404_REG15H, 0x000191 );
    write_reg24( &cfg->i2c, AFE4404_REG16H, 0x000197 );
    write_reg24( &cfg->i2c, AFE4404_REG17H, 0x0005BD );
    write_reg24( &cfg->i2c, AFE4404_REG18H, 0x0005C3 );
    write_reg24( &cfg->i2c, AFE4404_REG19H, 0x0009E9 );
    write_reg24( &cfg->i2c, AFE4404_REG1AH, 0x0009EF );
    write_reg24( &cfg->i2c, AFE4404_REG1BH, 0x000E15 );
    write_reg24( &cfg->i2c, AFE4404_REG1CH, 0x000E1B );
    write_reg24( &cfg->i2c, AFE4404_REG1DH, 0x009C3E );
    write_reg24( &cfg->i2c, AFE4404_REG1EH, 0x000103 ); 
    write_reg24( &cfg->i2c, AFE4404_REG20H, 0x008003 ); 
    write_reg24( &cfg->i2c, AFE4404_REG21H, 0x000013 ); 
    write_reg24( &cfg->i2c, AFE4404_REG22H, 0x01B6D9 ); 
    write_reg24( &cfg->i2c, AFE4404_REG23H, 0x104218 ); 
    write_reg24( &cfg->i2c, AFE4404_REG29H, 0x000000 ); 
    write_reg24( &cfg->i2c, AFE4404_REG31H, 0x000000 ); 
    write_reg24( &cfg->i2c, AFE4404_REG32H, 0x00155F ); 
    write_reg24( &cfg->i2c, AFE4404_REG33H, 0x00991E ); 
    write_reg24( &cfg->i2c, AFE4404_REG34H, 0x000000 );
    write_reg24( &cfg->i2c, AFE4404_REG35H, 0x00000f ); 
    write_reg24( &cfg->i2c, AFE4404_REG36H, 0x000190 ); 
    write_reg24( &cfg->i2c, AFE4404_REG37H, 0x00031F ); 
    write_reg24( &cfg->i2c, AFE4404_REG39H, 0x000000 ); 
    write_reg24( &cfg->i2c, AFE4404_REG3AH, 0x000000 );

    /* Configure interrupt GPIO setup */
    if (cfg->int_gpio.port && device_is_ready(cfg->int_gpio.port)) {
        gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);

        gpio_init_callback(&data->gpio_cb,
                           afe4404_gpio_callback,
                           BIT(cfg->int_gpio.pin));

        gpio_add_callback(cfg->int_gpio.port, &data->gpio_cb);        
        
        gpio_pin_interrupt_configure_dt(&cfg->int_gpio,
                                        GPIO_INT_EDGE_TO_ACTIVE);
    }

    printk("AFE4404 init OK\n");
    return 0;
}

/* --- Sample fetch --- */
static int afe4404_sample_fetch(const struct device *dev,
                                enum sensor_channel chan)
{
    struct afe4404_data *data = dev->data;
    const struct afe4404_config *cfg = dev->config;

    ARG_UNUSED(chan);

    /* Wait for DRDY from GPIO interrupt before reading a fresh conversion. */
    if (k_sem_take(&data->data_ready, K_MSEC(10)) != 0) {
        return -ETIMEDOUT;
    }

    /* Read only the registers needed for PPG processing */
    read_reg24(&cfg->i2c, AFE4404_LED1VAL,  &data->led1_val);   /* Saturation detection */
    read_reg24(&cfg->i2c, AFE4404_ALED1VAL, &data->aled1_val);  /* PPG signal */

    return 0;
}

/* --- Channel get --- */
static int afe4404_channel_get(const struct device *dev,
                               enum sensor_channel chan,
                               struct sensor_value *val)
{
    struct afe4404_data *data = dev->data;

    val->val2 = 0;

    switch (chan) {
    /* Standard Zephyr sensor channels */
    case SENSOR_CHAN_GREEN:
    case SENSOR_CHAN_AFE4404_LED2:
        val->val1 = data->led2_val;
        break;
    case SENSOR_CHAN_RED:
    case SENSOR_CHAN_AFE4404_LED1:
        val->val1 = data->led1_val;
        break;
    case SENSOR_CHAN_LIGHT:
    case SENSOR_CHAN_AFE4404_LED1_ALED1:
        val->val1 = data->led1_diff;
        break;
    case SENSOR_CHAN_IR:
    case SENSOR_CHAN_AFE4404_LED2_ALED2:
        val->val1 = data->led2_diff;
        break;

    /* AFE4404 ambient channels */
    case SENSOR_CHAN_AFE4404_ALED1:
        val->val1 = data->aled1_val;
        break;
    case SENSOR_CHAN_AFE4404_ALED2:
        val->val1 = data->aled2_val;
        break;

    default:
        return -ENOTSUP;
    }

    return 0;
}

/* --- API --- */
static const struct sensor_driver_api afe4404_api = {
    .sample_fetch = afe4404_sample_fetch,
    .channel_get = afe4404_channel_get,
};

/* --- Instance macro --- */
#define AFE4404_DEFINE(inst)                                      \
    static struct afe4404_data afe4404_data_##inst;               \
    static const struct afe4404_config afe4404_config_##inst = {  \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                        \
        .int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}), \
        .rst_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, rst_gpios, {0}), \
    };                                                           \
    DEVICE_DT_INST_DEFINE(inst,                                  \
                          afe4404_init,                          \
                          NULL,                                  \
                          &afe4404_data_##inst,                  \
                          &afe4404_config_##inst,                \
                          POST_KERNEL,                           \
                          CONFIG_SENSOR_INIT_PRIORITY,           \
                          &afe4404_api);

DT_INST_FOREACH_STATUS_OKAY(AFE4404_DEFINE)
