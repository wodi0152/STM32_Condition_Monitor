#ifndef PINTEREST2_OLED_ANIMATION_H
#define PINTEREST2_OLED_ANIMATION_H

#include <stdint.h>

#define PINTEREST2_FRAME_WIDTH 128u
#define PINTEREST2_FRAME_HEIGHT 64u
#define PINTEREST2_FRAME_BYTES 1024u
#define PINTEREST2_FRAME_COUNT 95u
#define PINTEREST2_FRAME_INTERVAL_MS 167u

/* SSD1306-native page-major bit-packed pixels. Stored in Flash. */
extern const uint8_t pinterest2_frames[PINTEREST2_FRAME_COUNT][PINTEREST2_FRAME_BYTES];

#endif
