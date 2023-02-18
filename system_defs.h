#pragma once





#include <stdint.h>

typedef uint8_t pixel_t;

static const uint32_t VGA_PIN_BASE = 10;
static const uint32_t SAMPLEIN_PIN_BASE =  2;
static const uint32_t USER_PUSHBUTTON_PIN =  1;
static const bool INCREASED_SYS_CLOCK =  true;

// Overrides for specially positioned duplicated sync outputs for the mini-pico module
#define VIDOUT_VSYNC_BITS  0x10 // 0x90  // 17 and 14
#define VIDOUT_HSYNC_BITS  0x04 // 0x44  // 16 and 12
#define VIDOUT_SYNC_BITS  (VIDOUT_VSYNC_BITS|VIDOUT_HSYNC_BITS)


#ifdef __cplusplus
extern "C" {
#endif
//extern void apploop(pixel_t *framebuffer);

extern void diag_printf(const char* fmt, ...);

extern size_t fw_check_diag_printf_(char* readbuf);


#ifdef __cplusplus
}
#endif

