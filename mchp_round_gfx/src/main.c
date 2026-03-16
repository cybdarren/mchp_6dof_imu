#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/spi.h>
#include <lvgl.h>
#include <soc.h>
#include <stdio.h>
#include <stdlib.h>
#include "bubble_level.h"
#include "imu_chart.h"

#define SV_FMT "%d.%06d"
#define SV_ARG(x) (x).val1, abs((x).val2)

#define SLEEP_TIME_MS 10

static struct sensor_trigger imu_trigger;
static volatile int irq_from_sensor = 0;

static volatile int irq_from_button = 0;
static bool button_pressed = false;

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static const struct gpio_dt_spec switch0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec switch1 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw1), gpios, {0});
static const struct device *imu = DEVICE_DT_GET(DT_ALIAS(imusensor));

/* Static sotrage for the two screens */
static lv_obj_t *bubble_screen = NULL;
static lv_obj_t *chart_screen = NULL;
static bool showing_chart = false;

// interrupt from imu
static void handle_imu_trigger(const struct device *dev, const struct sensor_trigger *trig)
{
    if (trig->type == SENSOR_TRIG_DATA_READY) {
        int rc = sensor_sample_fetch_chan(dev, trig->chan);

        if (rc < 0) {
            printk("Failed to fetch sensor data: %d\n", rc);
            return;     
        } else if (rc == 0) {
            irq_from_sensor = 1;
        }
    }   
}

// LVGL input read callback for button input device
static void lv_button_read(lv_indev_t * indev_drv, lv_indev_data_t * data)
{
    // Map Zephyr button state to LVGL touch state
    button_pressed = gpio_pin_get_dt(&switch0) == 1;
    data->state = button_pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    data->point.x = 120;
    data->point.y = 160;
}

static void button_input_cb(struct input_event *evt, void *user_data)
{
    if (evt->sync == 0) {
        return;
    }

    irq_from_button = 1;
    printk("Button %d %s\n", evt->code, evt->value ? "pressed" : "released");
}

INPUT_CALLBACK_DEFINE(NULL, button_input_cb, NULL);

/* SPI device from device tree */
static const struct spi_dt_spec spi_dev =
    SPI_DT_SPEC_GET(DT_NODELABEL(icm42688),
                    SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
                    0);

uint8_t icm_read_reg(uint8_t reg)
{
    uint8_t tx[3] = {reg | 0x80, 0x00, 0x00};
    uint8_t rx[3];

    const struct spi_buf tx_buf = {
        .buf = tx,
        .len = sizeof(tx),
    };

    const struct spi_buf rx_buf = {
        .buf = rx,
        .len = sizeof(rx),
    };

    const struct spi_buf_set tx_set = {
        .buffers = &tx_buf,
        .count = 1,
    };

    const struct spi_buf_set rx_set = {
        .buffers = &rx_buf,
        .count = 1,
    };

    spi_transceive_dt(&spi_dev, &tx_set, &rx_set);

    printk("RX: %02X %02X %02X\n", rx[0], rx[1], rx[2]);

    return rx[1];
}

int main(void)
{
    struct sensor_value accel[3];
    double ax, ay, az;
    struct sensor_value gyro[3];
    struct sensor_value temperature;
    int ret;

    printk("Application started\n");

    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

    // enable button interrupts
    if (device_is_ready(switch0.port)) {
        gpio_pin_interrupt_configure_dt(&switch0, GPIO_INPUT);  
    }
 
    if (device_is_ready(switch1.port)) {
        gpio_pin_interrupt_configure_dt(&switch1, GPIO_INPUT);  
    }   

    // configure the imu
    if (imu == NULL) {
        printk("IMU device not found\n");
        return 0;       
    }
	printf("Device %p name is %s\n", imu, imu->name);

    if (device_is_ready(imu)) {
        printk("IMU device is ready\n");
    } else {
        printk("IMU device is not ready\n");

        printk("ICM42688 SPI test\n");
        while (1) {
            uint8_t who = icm_read_reg(0x75);
            printk("WHO_AM_I = 0x%02X\n", who);
            k_sleep(K_SECONDS(1));
        }
    }

    // configure interrupts from the IMU
    imu_trigger = (struct sensor_trigger){
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ALL,
    };
    ret = sensor_trigger_set(imu, &imu_trigger, handle_imu_trigger);
    if (ret == 0) {
        printk("IMU trigger set successfully\n");
    } else {
        printk("Failed to set IMU trigger (%d)\n", ret);
    }

    // Start the display driver and LVGL
	const struct device *display_dev;

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		printk("Device not ready, aborting test\n");
        return 0;
	}

    // enforce RGB565 if not set in DTS
    display_set_pixel_format(display_dev, PIXEL_FORMAT_RGB_565);
    display_blanking_off(display_dev);

    // create and register the LVGL input device
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lv_button_read);

    struct display_capabilities caps;
    display_get_capabilities(display_dev, &caps);
    uint16_t width = caps.x_resolution;
    uint16_t height = caps.y_resolution;
    printk("Display size: %dx%d\n", width, height);

    /* ======== LVGL initialization ======== */
    // create the bubble screen
    bubble_screen = lv_obj_create(NULL);
    lv_scr_load(bubble_screen);
    bubble_level_init(bubble_screen);

    /* create label */
    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello Zephyr + LVGL!");
    lv_obj_center(label);

    /* Create button */
    lv_obj_t *btn = lv_button_create(lv_screen_active());
    lv_obj_set_size(btn, 80, 40);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 60);

    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Next");
    lv_obj_center(btn_label);

    /* create the chart screen */
    chart_screen = lv_obj_create(NULL);
    imu_chart_init(chart_screen);

    while (1) {
        if (irq_from_sensor) {
            sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, accel);
            sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyro);
            sensor_channel_get(imu, SENSOR_CHAN_DIE_TEMP, &temperature);
            ax = -sensor_value_to_double(&accel[0]);
            ay = sensor_value_to_double(&accel[1]);
            az = sensor_value_to_double(&accel[2]);

            if (showing_chart) {
                update_imu_chart(&accel[0], &accel[1], &accel[2]);
            } else {
                update_bubble_physics(&accel[0], &accel[1], &accel[2]);  
            }
            irq_from_sensor = 0;
        }

        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            return 0;   
        }

        if (irq_from_button) {
            irq_from_button = 0;
            button_pressed = gpio_pin_get_dt(&switch0) == 1;
            // Remove the label
            if (label != NULL) {
                lv_obj_del(label);
                label = NULL;
            }

            if (button_pressed) {
                if (showing_chart) {
                    lv_scr_load(bubble_screen);
                    showing_chart = false;
                } else {
                    lv_scr_load(chart_screen);
                    showing_chart = true;
                }
            }
        }   

        // Let LVGL process tasks and trigger flush
        lv_timer_handler(); 

        k_msleep(SLEEP_TIME_MS);
    }

    return 0;
}
