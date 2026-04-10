/*
 * Image utilities header
 */
#ifndef IMG_UTIL_H
#define IMG_UTIL_H

#include <zephyr/device.h>

/**
 * @brief Display a binary image file on the screen
 *
 * @param display Display device
 * @param path Path to the .bin image file (4-byte header: width, height as uint16_t)
 * @param x0 X position on display
 * @param y0 Y position on display
 * @return 0 on success, negative on error
 */
int display_bin_image(const struct device *display,
                      const char *path,
                      int x0, int y0);

#endif /* IMG_UTIL_H */
