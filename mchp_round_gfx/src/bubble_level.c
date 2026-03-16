#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <stdio.h>
#include <lvgl.h>
#include <math.h>
#include "bubble_level.h"

#define SCALE           32768
#define RADIUS          94
#define RADIUS2        (RADIUS * RADIUS)
#define BUBBLE_SIZE     28
#define LEVEL_SIZE      220

static lv_obj_t *level_bg;
static lv_obj_t *center_dot;
static lv_obj_t *bubble;

/* Simple integer sqrt using binary search */
static int32_t isqrt(int64_t x)
{
    int64_t res = 0;
    int64_t bit = (int64_t)1 << 30; /* largest power of four <= 2^63 */

    while (bit > x) bit >>= 2;

    while (bit != 0) {
        if (x >= res + bit) {
            x -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return (int32_t)res;
}

void level_create(lv_obj_t *parent)
{
    level_bg = lv_obj_create(parent);
    lv_obj_set_size(level_bg, 220, 220);
    lv_obj_center(level_bg);

    lv_obj_set_style_radius(level_bg, LV_RADIUS_CIRCLE, 0);

    lv_obj_set_style_bg_color(level_bg, lv_color_hex(0x0a3d62), 0);
    lv_obj_set_style_bg_grad_color(level_bg, lv_color_hex(0x1e90ff), 0);
    lv_obj_set_style_bg_grad_dir(level_bg, LV_GRAD_DIR_VER, 0);

    lv_obj_set_style_border_width(level_bg, 4, 0);
    lv_obj_set_style_border_color(level_bg, lv_color_hex(0x444444), 0);
}

void create_ring(int size)
{
    lv_obj_t *ring = lv_obj_create(level_bg);

    lv_obj_set_size(ring, size, size);
    lv_obj_center(ring);

    lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(ring, LV_OPA_TRANSP, 0);

    lv_obj_set_style_border_color(ring, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_border_width(ring, 2, 0);
    lv_obj_set_style_border_opa(ring, LV_OPA_40, 0);
}

void create_center()
{
    center_dot = lv_obj_create(level_bg);

    lv_obj_set_size(center_dot, 8, 8);
    lv_obj_center(center_dot);

    lv_obj_set_style_radius(center_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(center_dot, lv_color_hex(0xffffff), 0);
}

void create_bubble()
{
    bubble = lv_obj_create(level_bg);

    lv_obj_set_size(bubble, 28, 28);

    lv_obj_set_style_radius(bubble, LV_RADIUS_CIRCLE, 0);

    lv_obj_set_style_bg_color(bubble, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_bg_grad_color(bubble, lv_color_hex(0xdddddd), 0);
    lv_obj_set_style_bg_grad_dir(bubble, LV_GRAD_DIR_VER, 0);

    lv_obj_set_style_border_color(bubble, lv_color_hex(0x888888), 0);
    lv_obj_set_style_border_width(bubble, 2, 0);

    lv_obj_set_style_shadow_width(bubble, 0, 0);
//    lv_obj_set_style_shadow_opa(bubble, LV_OPA_40, 0);

    lv_obj_set_pos(bubble, LEVEL_SIZE/2 - BUBBLE_SIZE/2, LEVEL_SIZE/2 - BUBBLE_SIZE/2);
}

lv_obj_t* bubble_level_init(lv_obj_t *parent)
{
    if (parent == NULL)
        return NULL;
        
    level_create(parent);
    create_ring(160);
    create_ring(110);
    create_ring(60);

    create_center();
    create_bubble();  

    return parent;
}

void update_bubble_physics(struct sensor_value *ax_sv,
                          struct sensor_value *ay_sv,
                          struct sensor_value *az_sv)
{
    /* convert sensor values (µg style scaling) */
    int64_t ax = -((int64_t)ax_sv->val1 * 1000000 + ax_sv->val2);
    int64_t ay = (int64_t)ay_sv->val1 * 1000000 + ay_sv->val2;
    int64_t az = (int64_t)az_sv->val1 * 1000000 + az_sv->val2;

    /* magnitude approximation (fast) */
    int64_t mag = llabs(ax) + llabs(ay) + llabs(az);
    if (mag == 0)
        return;

    /* normalized tilt in Q15 */
    int32_t nx = (-ax * SCALE) / mag;
    int32_t ny = (-ay * SCALE) / mag;

    // nx = ((nx * nx) / SCALE) * (nx >= 0 ? 1 : -1);
    // ny = ((ny * ny) / SCALE) * (ny >= 0 ? 1 : -1);

    /* convert tilt to pixel displacement */
    int target_x = (nx * RADIUS) / SCALE;
    int target_y = (ny * RADIUS) / SCALE;

//    printf("nx: %8d,\tny: %8d,\tnx_px: %8d,\tny_px: %8d,\ttarget_x: %8d,\ttarget_y: %8d\n", nx, ny, nx_px, ny_px, target_x, target_y);

    /* clamp bubble inside circle */
    int32_t dist2 = target_x * target_x + target_y * target_y;
    if (dist2 > RADIUS2) {
        int32_t dist = isqrt(dist2);
        target_x = (target_x * RADIUS) / dist;
        target_y = (target_y * RADIUS) / dist;
    }

    /* smoothing filter (liquid behaviour) */
    static int bubble_x = 0;
    static int bubble_y = 0;
    bubble_x = (bubble_x * 5 + target_x * 3) / 8;
    bubble_y = (bubble_y * 5 + target_y * 3) / 8;

    /* convert to LVGL coordinates */
    int parent_center_x = lv_obj_get_content_width(level_bg) / 2;
    int parent_center_y = lv_obj_get_content_height(level_bg) / 2;
    int x = parent_center_x + bubble_x - lv_obj_get_width(bubble) / 2;
    int y = parent_center_y + bubble_y - lv_obj_get_height(bubble) / 2;

    lv_obj_set_pos(bubble, x, y);

    /* level detection (~2 degrees) */
    if (llabs(nx) < SCALE/40 && llabs(ny) < SCALE/40)
        lv_obj_set_style_bg_color(bubble, lv_color_hex(0x00ff88), 0);
    else
        lv_obj_set_style_bg_color(bubble, lv_color_white(), 0);
}
