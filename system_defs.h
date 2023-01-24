#pragma once





#include <stdint.h>

typedef uint8_t pixel_t;

static const uint32_t VGA_PIN_BASE = 10;
static const uint32_t SAMPLEIN_PIN_BASE =  2;
static const uint32_t USER_PUSHBUTTON_PIN =  1;
static const bool INCREASED_SYS_CLOCK =  true;

enum VidType{
    QL_NATIVE,
    ZX_SP_PLUS23,
    INVALID
};

#ifdef __cplusplus
extern "C" {
#endif
//extern void apploop(pixel_t *framebuffer);

extern void diag_printf(const char* fmt, ...);

extern size_t fw_check_diag_printf_(char* readbuf);


#ifdef __cplusplus
}
#endif

