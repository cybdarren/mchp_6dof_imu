/*
 * Main application for testing hw features of Zephyr handheld platform
 *
 */
 #include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/spi.h>
#include <lvgl.h>
#include <soc.h>
#include <stdio.h>
#include <stdlib.h>
#include <ws2812_common.h>

#define SLEEP_TIME_MS 200  /* Main loop sleep time */

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec switch0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec switch1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);

static void button_input_cb(struct input_event *evt, void *user_data)
{
    if (evt->sync == 0) {
        return;
    }

    printk("Button %d %s\n", evt->code, evt->value ? "pressed" : "released");
}

INPUT_CALLBACK_DEFINE(NULL, button_input_cb, NULL);

int main(void)
{
    int ret;
    bool strip_toggle = false;

    printk("Application started\n");

    /* Initialize LED GPIO */
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

    ws2812_init();
    ws2812_set_pixel(1, 0, 0, 64, 0);  /* Set first LED of strip 1 to green */
    ws2812_set_pixel(1, 1, 0, 0, 64);  /* Set second LED of strip 1 to blue */
    ws2812_update(1);

    ws2812_set_pixel(0, 0, 64, 0, 0);  /* Set first LED of strip 0 to red */
    ws2812_update(0);
    // const struct device *porta = DEVICE_DT_GET(DT_NODELABEL(porta));
    // if (!device_is_ready(porta)) {
    //     printk("PORTA device not ready\n");
    //     return 0;
    // }
    // ret = gpio_pin_configure(porta, 14, GPIO_OUTPUT);   

    /* Initialize display and LVGL */
    const struct device *display_dev;

    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        printk("Device not ready, aborting test\n");
        return 0;
    }

    display_set_pixel_format(display_dev, PIXEL_FORMAT_RGB_565);
    display_blanking_off(display_dev);

    /* ======== LVGL Screen Initialization ======== */

    /* Temporary welcome label */
    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Zephyr on SAM D51!");
    lv_obj_center(label);

    /* Main application loop */
    while (1) {
        ret = gpio_pin_toggle_dt(&led);
        if (strip_toggle) {
            ws2812_set_pixel(1, 0, 0, 64, 0);  /* Set first LED of strip 1 to green */
            ws2812_update(1);
        } else {
            ws2812_set_pixel(1, 0, 64, 0, 0);  /* Set first LED of strip 1 to red */
            ws2812_update(1);
        }   
        strip_toggle = !strip_toggle;
        // gpio_pin_toggle(porta, 14);

        /* Run LVGL timer handler for UI updates */
        lv_timer_handler();

        k_msleep(SLEEP_TIME_MS);
    }

    return 0;
}
