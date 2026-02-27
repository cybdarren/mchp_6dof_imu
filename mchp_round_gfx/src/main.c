#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/sys/byteorder.h>
#include <lvgl.h>
#include <soc.h>
#include <stdio.h>
#include <stdlib.h>

#define SLEEP_TIME_MS 200

static volatile int irq_from_button = 0;

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static const struct gpio_dt_spec switch0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec switch1 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw1), gpios, {0});

static void button_input_cb(struct input_event *evt, void *user_data)
{
    if (evt->sync == 0) {
        return;
    }

    irq_from_button = 1;
    printk("Button %d %s\n", evt->code, evt->value ? "pressed" : "released");
}

INPUT_CALLBACK_DEFINE(NULL, button_input_cb, NULL);

int main(void)
{
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

	const struct device *display_dev;

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		printk("Device not ready, aborting test\n");
        return 0;
	}

    // enforce RGB565 if not set in DTS
    display_set_pixel_format(display_dev, PIXEL_FORMAT_RGB_565);
    display_blanking_off(display_dev);

    struct display_capabilities caps;
    display_get_capabilities(display_dev, &caps);
    uint16_t width = caps.x_resolution;
    uint16_t height = caps.y_resolution;
    printk("Display size: %dx%d\n", width, height);

    /* ======== LVGL initialization ======== */
    /* create label */
    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello Zephyr + LVGL!");
    lv_obj_center(label);

    /* Create button */
    lv_obj_t *btn = lv_button_create(lv_screen_active());
    lv_obj_set_size(btn, 100, 50);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 40);

    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Press");
    lv_obj_center(btn_label);

    while (1) {
        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            return 0;   
        }

        if (irq_from_button) {
            irq_from_button = 0;
            lv_label_set_text(label, "Button pressed!");
        }   

        // Let LVGL process tasks and trigger flush
        lv_timer_handler(); 

        k_msleep(SLEEP_TIME_MS);
    }

    return 0;
}
