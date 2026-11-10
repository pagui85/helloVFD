#include "vfd_framebuffer.h"

#include <string.h>

static uint8_t framebuffer[VFD_BUFFER_SIZE];

void vfd_framebuffer_init(void)
{
    memset(framebuffer, 0, sizeof(framebuffer));
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