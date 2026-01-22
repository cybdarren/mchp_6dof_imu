#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <stdlib.h>

#define SV_FMT "%d.%06d"
#define SV_ARG(x) (x).val1, abs((x).val2)

#define SLEEP_TIME_MS 1000

#define LED0_NODE DT_ALIAS(led0)

static struct sensor_trigger imu_trigger;
static volatile int irq_from_sensor = 0;

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct device *imu = DEVICE_DT_GET(DT_ALIAS(imusensor));

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

int main(void)
{
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    struct sensor_value temperature;
    int ret;
    bool led_state = true;

    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }
    led_state = false;

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
    }

    // configure interrupts from the IMU
    imu_trigger = (struct sensor_trigger){
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ALL,
    };
    if (sensor_trigger_set(imu, &imu_trigger, handle_imu_trigger)) {
        printk("Failed to set IMU trigger\n");
    } else {
        printk("IMU trigger set successfully\n");
    }

    while (1) {
        if (irq_from_sensor) {
            sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, accel);
            sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyro);
            sensor_channel_get(imu, SENSOR_CHAN_DIE_TEMP, &temperature);

            printf("temp " SV_FMT " C  "
                "accel " SV_FMT " " SV_FMT " " SV_FMT " m/s^2  "
                "gyro  " SV_FMT " " SV_FMT " " SV_FMT " rad/s\n",
                SV_ARG(temperature),
                SV_ARG(accel[0]), SV_ARG(accel[1]), SV_ARG(accel[2]),
                SV_ARG(gyro[0]),  SV_ARG(gyro[1]),  SV_ARG(gyro[2]));          
            irq_from_sensor = 0;
        }

        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            return 0;   
        }

        led_state = !led_state;
        // printk("LED state: %s\n", led_state ? "ON" : "OFF");
        // k_msleep(SLEEP_TIME_MS);
    }

    return 0;
}
