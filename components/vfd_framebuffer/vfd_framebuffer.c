#include "vfd_framebuffer.h"

#include <string.h>

static uint8_t framebuffer[VFD_BUFFER_SIZE];

void vfd_framebuffer_init(void)
{
    // LVGL I1 uses cleared bits for illuminated pixels; 0xFF is all off.
    memset(framebuffer, 0xFF, sizeof(framebuffer));
}

void vfd_framebuffer_copy(const uint8_t *pixels)
{
    if (pixels == NULL) {
        return;
    }

    memcpy(framebuffer, pixels, sizeof(framebuffer));
}

const uint8_t *vfd_framebuffer_get(void)
{
    return framebuffer;
}
