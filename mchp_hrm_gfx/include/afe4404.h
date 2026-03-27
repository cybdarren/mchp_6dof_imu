#ifndef ZEPHYR_DRIVERS_SENSOR_AFE4404_H_
#define ZEPHYR_DRIVERS_SENSOR_AFE4404_H_

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

/**
 * AFE4404 custom sensor channels (starting from SENSOR_CHAN_PRIV_START)
 */
enum afe4404_channel {
    SENSOR_CHAN_AFE4404_LED1 = SENSOR_CHAN_PRIV_START,  /* LED1 (red/IR) raw */
    SENSOR_CHAN_AFE4404_LED2,                           /* LED2 (green) raw */
    SENSOR_CHAN_AFE4404_ALED1,                          /* Ambient during LED1 phase */
    SENSOR_CHAN_AFE4404_ALED2,                          /* Ambient during LED2 phase */
    SENSOR_CHAN_AFE4404_LED1_ALED1,                     /* LED1 - ambient (corrected) */
    SENSOR_CHAN_AFE4404_LED2_ALED2,                     /* LED2 - ambient (corrected) */
};


/**
 * AFE4404 Timing control registers.
 */
#define AFE4404_REG0H                       0x00
#define AFE4404_REG1H                       0x01
#define AFE4404_REG2H                       0x02
#define AFE4404_REG3H                       0x03
#define AFE4404_REG4H                       0x04
#define AFE4404_REG5H                       0x05
#define AFE4404_REG6H                       0x06
#define AFE4404_REG7H                       0x07
#define AFE4404_REG8H                       0x08
#define AFE4404_REG9H                       0x09
#define AFE4404_REGAH                       0x0A
#define AFE4404_REGBH                       0x0B
#define AFE4404_REGCH                       0x0C
#define AFE4404_REGDH                       0x0D
#define AFE4404_REGEH                       0x0E
#define AFE4404_REGFH                       0x0F
#define AFE4404_REG10H                      0x10
#define AFE4404_REG11H                      0x11
#define AFE4404_REG12H                      0x12
#define AFE4404_REG13H                      0x13
#define AFE4404_REG14H                      0x14
#define AFE4404_REG15H                      0x15
#define AFE4404_REG16H                      0x16
#define AFE4404_REG17H                      0x17
#define AFE4404_REG18H                      0x18
#define AFE4404_REG19H                      0x19
#define AFE4404_REG1AH                      0x1A
#define AFE4404_REG1BH                      0x1B
#define AFE4404_REG1CH                      0x1C
#define AFE4404_REG1DH                      0x1D
#define AFE4404_REG1EH                      0x1E
#define AFE4404_REG20H                      0x20
#define AFE4404_REG21H                      0x21
#define AFE4404_REG22H                      0x22
#define AFE4404_REG23H                      0x23
#define AFE4404_REG29H                      0x29
#define AFE4404_REG2AH                      0x2A
#define AFE4404_REG2BH                      0x2B
#define AFE4404_REG2CH                      0x2C
#define AFE4404_REG2DH                      0x2D
#define AFE4404_REG2EH                      0x2E
#define AFE4404_REG2FH                      0x2F

/* ADC result registers with descriptive names */
#define AFE4404_LED2VAL                     0x2A  /* Green LED */
#define AFE4404_ALED2VAL                    0x2B  /* Ambient during LED2 phase */
#define AFE4404_LED1VAL                     0x2C  /* Red/IR LED */
#define AFE4404_ALED1VAL                    0x2D  /* Ambient during LED1 phase */
#define AFE4404_LED2_ALED2                  0x2E  /* LED2 minus ambient */
#define AFE4404_LED1_ALED1                  0x2F  /* LED1 minus ambient */
#define AFE4404_REG31H                      0x31

/**
 * AFE4404 timing controls for dynamic power-down registers.
 */
#define AFE4404_REG32H                      0x32
#define AFE4404_REG33H                      0x33
#define AFE4404_REG34H                      0x34
#define AFE4404_REG35H                      0x35

/**
 * AFE4404 Timing controls for driving the third LED registers.
 */
#define AFE4404_REG36H                      0x36
#define AFE4404_REG37H                      0x37
#define AFE4404_REG39H                      0x39
#define AFE4404_REG3AH                      0x3A
#define AFE4404_REG3DH                      0x3D
#define AFE4404_REG3FH                      0x3F
#define AFE4404_REG40H                      0x40

/* Device configuration */
struct afe4404_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
    struct gpio_dt_spec rst_gpio;
    bool continuous;
};

/* Runtime data */
struct afe4404_data {
    uint32_t led2_val;      /* Green LED raw */
    uint32_t aled2_val;     /* Ambient during LED2 phase */
    uint32_t led1_val;      /* Red/IR LED raw */
    uint32_t aled1_val;     /* Ambient during LED1 phase */
    uint32_t led2_diff;     /* LED2 - ambient */
    uint32_t led1_diff;     /* LED1 - ambient */

    const struct device *dev;

    /* Interrupt handling */
    struct gpio_callback gpio_cb;
    struct k_sem data_ready;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_AFE4404_H_ */
