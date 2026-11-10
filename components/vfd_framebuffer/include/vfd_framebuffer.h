#pragma once

#include <stdint.h>
#include <stddef.h>

#define VFD_WIDTH       128
#define VFD_HEIGHT      32
#define VFD_BUFFER_SIZE (VFD_WIDTH * VFD_HEIGHT / 8)

void vfd_framebuffer_init(void);

void vfd_framebuffer_copy(const uint8_t *pixels);

const uint8_t *vfd_framebuffer_get(void);