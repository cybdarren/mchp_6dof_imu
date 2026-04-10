#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <ff.h>
#include <string.h>

#define LINE_BUF_MAX 240  // adjust to your max width

static __aligned(4) uint16_t line_buf[LINE_BUF_MAX];

int display_bin_image(const struct device *display,
                      const char *path,
                      int x0, int y0)
{
    FIL file;
    UINT br;

    uint16_t width, height;
    uint16_t v;

    /* Open file */
    if (f_open(&file, path, FA_READ) != FR_OK) {
        printk("Failed to open %s\n", path);
        return -1;
    }

    /* Read header */
    f_read(&file, &width, 2, &br);
    f_read(&file, &height, 2, &br);

    if (width > LINE_BUF_MAX) {
        printk("Width too large\n");
        f_close(&file);
        return -1;
    }

    struct display_buffer_descriptor desc = {
        .width = width,
        .height = 1,
        .pitch = width,
        .buf_size = width * 2,
    };

    /* Stream each row */
    for (int y = 0; y < height; y++) {

        f_read(&file, line_buf, width * 2, &br);
        if (br != width * 2) {
            printk("Read error at line %d\n", y);
            break;
        }

        for(int i = 0; i < width; i++) {
            v = line_buf[i];
            line_buf[i] = (v >> 8) | (v << 8);  // Swap bytes for little-endian display
        }

        /* Write one line to display */
        display_write(display,
                      x0,
                      y0 + y,
                      &desc,
                      line_buf);
    }

    f_close(&file);

    return 0;
}
