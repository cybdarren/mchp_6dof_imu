
/*
 * Heart-rate monitor (HRM) text display module.
 *
 * Provides a minimal LVGL-based UI that shows the current heart rate
 * as a large centred label (e.g. "72 bpm") inside a black container.
 * Call hrm_text_init() once to build the widget tree, then call
 * update_hrm_text() each time a new BPM value is available.
 */
#include <stdlib.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>
#include <lvgl.h>
#include <math.h>
#include "hrm_text.h"

#define LABEL_X_OFFSET 10  /* Horizontal offset for labels from left edge */

static lv_obj_t *screen_text;  /* Outer container widget */
static lv_obj_t *label_bpm;    /* BPM value label       */

/**
 * hrm_text_init() - Build the HRM text widget tree.
 * @parent: LVGL parent object to attach the container to.
 *
 * Creates a 180x100 black container centred on @parent and places a
 * white 28-pt label inside it, initially showing "-- bpm".
 *
 * Return: @parent (pass-through for chaining).
 */
lv_obj_t* hrm_text_init(lv_obj_t *parent)
{
    /* Create and style the outer container (black background, centred) */
    screen_text = lv_obj_create(parent);
    lv_obj_set_style_bg_color(screen_text, lv_color_black(), 0);
    lv_obj_set_size(screen_text, 180, 100);
    lv_obj_center(screen_text);

    /* Create the BPM label (large white text, centred in container) */
    label_bpm = lv_label_create(screen_text);
    lv_obj_set_style_text_color(label_bpm, lv_color_white(), 0);
    lv_obj_set_style_text_font(label_bpm, &lv_font_montserrat_28, 0);
    lv_label_set_text(label_bpm, "-- bpm");
    lv_obj_align(label_bpm, LV_ALIGN_CENTER, 0, 0);

    return parent;   
}

/**
 * update_hrm_text() - Refresh the displayed heart-rate value.
 * @bpm: Beats-per-minute to display.  Values <= 0 show "-- bpm"
 *       to indicate that no valid reading is available.
 */
void update_hrm_text(int bpm)
{
    char buf[32];

    if (bpm > 0) {
        snprintf(buf, sizeof(buf), "%d bpm", bpm);
    } else {
        snprintf(buf, sizeof(buf), "-- bpm");
    }
    lv_label_set_text(label_bpm, buf);
}

