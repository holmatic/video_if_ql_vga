#ifndef VGA_6BIT_H_FILE
#define VGA_6BIT_H_FILE

#include <stdbool.h>

#define VGA_ERROR_ALLOC     (-1)
#define VGA_ERROR_MULTICORE (-2)

#ifdef __cplusplus
extern "C" {
#endif

struct VGA_MODE {
  unsigned int   pixel_clock_mhz;

  unsigned short h_front_porch;
  unsigned short h_sync_pulse;
  unsigned short h_back_porch;
  unsigned short h_pixels;

  unsigned short v_front_porch;
  unsigned short v_sync_pulse;
  unsigned short v_back_porch;
  unsigned short v_pixels;
  unsigned char  v_div;

  unsigned char h_polarity;
  unsigned char v_polarity;
};

struct VGA_SCREEN {
  int width;
  int height;
  unsigned char sync_bits;
  unsigned int **framebuffer;
};
  
#if VGA_ENABLE_MULTICORE
extern void (*volatile vga_core1_func)(void);
#endif

int vga_init(const struct VGA_MODE *mode, unsigned int pin_out_base, void** framebuffer);
void vga_clear_screen(unsigned char color);
void vga_swap_buffers(bool wait_sync);

extern struct VGA_SCREEN vga_screen;

extern const struct VGA_MODE vga_mode_320x240;
extern const struct VGA_MODE vga_mode_512x256;


extern unsigned int *framebuffers[];

#ifdef __cplusplus
}
#endif

#endif /* VGA_6BIT_H_FILE */
