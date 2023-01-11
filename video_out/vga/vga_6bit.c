/**
 * vga_6bit.c
 *
 * Copyright (C) 2021 MoeFH
 * Released under the MIT License
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* 
 * Note: Modified / reduced in functionality for the retro-video-interface 2023.
 * 
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/structs/bus_ctrl.h"

#include "vga_6bit.h"
#include "vga_6bit.pio.h"

//                                         clock     hf  hp  hb  hpix    vf  vb  vb  vpix  vdiv  (h,v)pol
const struct VGA_MODE vga_mode_320x240 = { 12587500,  8, 48, 24, 320,    10,  2, 33, 480,     2,  1,1  };
const struct VGA_MODE vga_mode_320x200 = { 12587500,  8, 48, 24, 320,    50,  2, 73, 400,     2,  1,1  };
const struct VGA_MODE vga_mode_512x256 = { 32500000,  12, 68, 80, 512,    3,  6, 29, 768,     3,  1,1  };

const static struct VGA_MODE *vga_mode = NULL;

#define PIX_CLOCK_MHZ (vga_mode->pixel_clock_mhz)
#define H_FRONT_PORCH (vga_mode->h_front_porch)
#define H_SYNC_PULSE  (vga_mode->h_sync_pulse)
#define H_BACK_PORCH  (vga_mode->h_back_porch)
#define H_PIXELS      (vga_mode->h_pixels)
#define V_FRONT_PORCH (vga_mode->v_front_porch)
#define V_SYNC_PULSE  (vga_mode->v_sync_pulse)
#define V_BACK_PORCH  (vga_mode->v_back_porch)
#define V_PIXELS      (vga_mode->v_pixels)
#define V_DIV         (vga_mode->v_div)
#define H_POLARITY    (vga_mode->h_polarity)
#define V_POLARITY    (vga_mode->v_polarity)

#define H_FULL_LINE   (H_FRONT_PORCH+H_SYNC_PULSE+H_BACK_PORCH+H_PIXELS)
#define V_FULL_FRAME  (V_FRONT_PORCH+V_SYNC_PULSE+V_BACK_PORCH+V_PIXELS)

#define HSYNC_ON           (!H_POLARITY)
#define HSYNC_OFF          ( H_POLARITY)
#define VSYNC_ON           (!V_POLARITY)
#define VSYNC_OFF          ( V_POLARITY)
#define HBLANK_BUFFER_LEN  ((H_FRONT_PORCH+H_SYNC_PULSE+H_BACK_PORCH)/4)
#define HPIXELS_BUFFER_LEN (H_PIXELS/4)

#define SYNC_BITS     ((VSYNC_OFF<<7) | (HSYNC_OFF<<6))
#define SCREEN_WIDTH  H_PIXELS
#define SCREEN_HEIGHT (V_PIXELS/V_DIV)

static unsigned int *hblank_buffer_vsync_on;
static unsigned int *hblank_buffer_vsync_off;
static unsigned int *hpixels_buffer_vsync_on;
static unsigned int *hpixels_buffer_vsync_off;
unsigned int *framebuffers[2];
static unsigned int **cur_framebuffer_lines;

struct DMA_BUFFER_INFO {
  uintptr_t read_addr;
  uintptr_t write_addr;
  uint32_t  transfer_count;
  uint32_t  ctrl_trig;
};
static struct DMA_BUFFER_INFO *dma_chain;
static void *dma_restart_buffer[1];
static uint dma_control_chan;
static uint dma_data_chan;

static volatile uint frame_count;
static uint cur_framebuffer;
struct VGA_SCREEN vga_screen;

static void __isr __time_critical_func(dma_handler)(void)
{
  dma_hw->ints1 = 1u << dma_data_chan;
  frame_count++;
}

static void set_dma_buffer_src(struct DMA_BUFFER_INFO *buf, volatile void *src, uint32_t count)
{
  buf->read_addr = (uintptr_t) src;
  buf->transfer_count = count;
}

static void set_dma_buffer_dst(struct DMA_BUFFER_INFO *buf, volatile void *dest, uint32_t ctrl)
{
  buf->write_addr = (uintptr_t) dest;
  buf->ctrl_trig = ctrl;
}

static int init_pio(unsigned int pin_out_base)
{
  PIO pio = pio0;
  uint sm = pio_claim_unused_sm(pio, true);
  uint pio_dreq = pio_get_dreq(pio, sm, true);

  // choose PIO clock divider based on the CPU clock and VGA pixel clock
  uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
  float clock_div = ((float)f_clk_sys * 1000.f) / (float)PIX_CLOCK_MHZ;

  uint offset = pio_add_program(pio, &vga_program);
  vga_program_init(pio, sm, offset, pin_out_base, clock_div);

  dma_control_chan = dma_claim_unused_channel(true);
  dma_data_chan    = dma_claim_unused_channel(true);

  // DMA control channel config
  dma_channel_config cfg = dma_channel_get_default_config(dma_control_chan);
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
  channel_config_set_read_increment(&cfg, true);
  channel_config_set_write_increment(&cfg, true);
  channel_config_set_ring(&cfg, true, 4);    // loop write address every 1<<4 = 16 bytes
  
  dma_channel_configure(dma_control_chan,
                        &cfg,
                        &dma_hw->ch[dma_data_chan].read_addr,     // dest (update data channel and trigger it)
                        &dma_chain[0],                            // source
                        4,                                        // num words for each transfer
                        false                                     // don't start now
                        );

  // all blocks of dma_chain except last are set to trigger dma_data_chan to copy data to PIO
  for (int i = 0; i < 2*V_FULL_FRAME; i++) {
    // src will be set by init_buffers() and vga_swap_buffers()
    set_dma_buffer_dst(&dma_chain[i],
                       &pio->txf[sm],                                              // write to PIO
                       DMA_CH0_CTRL_TRIG_INCR_READ_BITS                         |  // increment read ptr
                       (pio_dreq            << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB)  |  // as fast as PIO requires
                       (dma_control_chan    << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB)  |  // chain to dma_control_chan
                       (((uint)DMA_SIZE_32) << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) |  // copy 32 bits per count
                       DMA_CH0_CTRL_TRIG_IRQ_QUIET_BITS                         |  // suppress IRQ
                       DMA_CH0_CTRL_TRIG_EN_BITS);
  }

  // last block of dma_chain is set to trigger dma_data_chan to copy the dma_chain start address to the control chain (restarting it)
  set_dma_buffer_src(&dma_chain[2*V_FULL_FRAME], dma_restart_buffer, 1);
  set_dma_buffer_dst(&dma_chain[2*V_FULL_FRAME],
                     &dma_hw->ch[dma_control_chan].al3_read_addr_trig,           // write to dma_control_chan read address trigger
                     (DREQ_FORCE          << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB)  |  // as fast as possible
                     (dma_data_chan       << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB)  |  // chain to itself (don't chain)
                     (((uint)DMA_SIZE_32) << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) |  // copy 32 bits per count
                     0                                                        |  // trigger IRQ
                     DMA_CH0_CTRL_TRIG_EN_BITS);

  if(1){  // TODO - conflict with audio DMA..
    dma_channel_set_irq1_enabled(dma_data_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_1, dma_handler);
    irq_set_priority(DMA_IRQ_1, 0xff);
    irq_set_enabled(DMA_IRQ_1, true);
  }
  return 0;
}

static void clear_framebuffer(uint fb_num, uint8_t color)
{
  uint8_t val = SYNC_BITS | (color & 0x3f);
  memset(framebuffers[fb_num], val, SCREEN_WIDTH*SCREEN_HEIGHT);
}

static int alloc_buffers(int num_framebuffers)
{
  for (int i = 0; i < num_framebuffers; i++) {
    framebuffers[i] = NULL;
  }
  cur_framebuffer_lines    = NULL;
  hblank_buffer_vsync_on   = NULL;
  hblank_buffer_vsync_off  = NULL;
  hpixels_buffer_vsync_on  = NULL;
  hpixels_buffer_vsync_off = NULL;
  dma_chain                = NULL;

#define ALLOC(p, size)  p = malloc(size); if (! p) goto error
  for (int i = 0; i < num_framebuffers; i++) {
    ALLOC(framebuffers[i], SCREEN_WIDTH * SCREEN_HEIGHT);
  }
  ALLOC(cur_framebuffer_lines,    SCREEN_HEIGHT      * sizeof(unsigned int *));
  ALLOC(hblank_buffer_vsync_on,   HBLANK_BUFFER_LEN  * sizeof(unsigned int));
  ALLOC(hblank_buffer_vsync_off,  HBLANK_BUFFER_LEN  * sizeof(unsigned int));
  ALLOC(hpixels_buffer_vsync_on,  HPIXELS_BUFFER_LEN * sizeof(unsigned int));
  ALLOC(hpixels_buffer_vsync_off, HPIXELS_BUFFER_LEN * sizeof(unsigned int));
  ALLOC(dma_chain,                (2*V_FULL_FRAME+1) * sizeof(struct DMA_BUFFER_INFO));
#undef ALLOC

  return 0;

 error:
  for (int i = 0; i < num_framebuffers; i++) {
    free(framebuffers[i]);
  }
  free(cur_framebuffer_lines);
  free(hblank_buffer_vsync_on);
  free(hblank_buffer_vsync_off);
  free(hpixels_buffer_vsync_on);
  free(hpixels_buffer_vsync_off);
  free(dma_chain);
  return -1;
}

static int init_buffers(int num_framebuffers)
{
  if (alloc_buffers(num_framebuffers) < 0) {
    return VGA_ERROR_ALLOC;
  }

  uint8_t sync_h0v0 = (VSYNC_OFF<<7) | (HSYNC_OFF<<6);
  uint8_t sync_h1v0 = (VSYNC_OFF<<7) | (HSYNC_ON <<6);
  uint8_t sync_h0v1 = (VSYNC_ON <<7) | (HSYNC_OFF<<6);
  uint8_t sync_h1v1 = (VSYNC_ON <<7) | (HSYNC_ON <<6);
  
  // hblank lines
  unsigned char *hblank_vsync_on   = (unsigned char *) hblank_buffer_vsync_on;
  unsigned char *hblank_vsync_off  = (unsigned char *) hblank_buffer_vsync_off;
  for (int i = 0; i < H_FRONT_PORCH+H_SYNC_PULSE+H_BACK_PORCH; i++) {
    if (i >= H_FRONT_PORCH && i < H_FRONT_PORCH+H_SYNC_PULSE) {
      hblank_vsync_on[i]  = sync_h1v1;
      hblank_vsync_off[i] = sync_h1v0;
    } else {
      hblank_vsync_on[i]  = sync_h0v1;
      hblank_vsync_off[i] = sync_h0v0;
    }
  }

  // vblank pixel lines
  memset(hpixels_buffer_vsync_on,  sync_h0v1, H_PIXELS);
  memset(hpixels_buffer_vsync_off, sync_h0v0, H_PIXELS);

  // framebuffers
  for (int i = 0; i < num_framebuffers; i++) {
    clear_framebuffer(i, 0);
  }
  
  // setup DMA chain buffers
  struct DMA_BUFFER_INFO *buf = &dma_chain[0];
  for (int i = 0; i < V_FULL_FRAME; i++) {
    if (i < V_SYNC_PULSE) {
      // vblank with vsync active
      set_dma_buffer_src(buf++, hblank_buffer_vsync_on,  HBLANK_BUFFER_LEN);
      set_dma_buffer_src(buf++, hpixels_buffer_vsync_on, HPIXELS_BUFFER_LEN);
    } else if (i < V_SYNC_PULSE+V_BACK_PORCH || i >= V_SYNC_PULSE+V_BACK_PORCH+V_PIXELS) {
      // vblank with vsync inactive
      set_dma_buffer_src(buf++, hblank_buffer_vsync_off,  HBLANK_BUFFER_LEN);
      set_dma_buffer_src(buf++, hpixels_buffer_vsync_off, HPIXELS_BUFFER_LEN);
    } else {
      // pixel data
      set_dma_buffer_src(buf++, hblank_buffer_vsync_off, HBLANK_BUFFER_LEN);
      set_dma_buffer_src(buf++, NULL, HPIXELS_BUFFER_LEN);  // set by vga_swap_buffers()
    }
  }

  // setup DMA restart buffer
  dma_restart_buffer[0] = &dma_chain[0];

  return 0;
}

// === INTERFACE ====================================================

void vga_swap_buffers(bool wait_sync)
{
  if (wait_sync) {
    uint start_frame_count = frame_count;
    while (frame_count == start_frame_count) {
      //sleep_ms(1);  // should we remove this?
    }
  }
  
  // inject new framebuffer in DMA chain
  for (int i = 0; i < V_PIXELS; i++) {
    struct DMA_BUFFER_INFO *buf = &dma_chain[2*(V_SYNC_PULSE+V_BACK_PORCH+i) + 1];
    set_dma_buffer_src(buf, &framebuffers[cur_framebuffer][i/V_DIV*HPIXELS_BUFFER_LEN], HPIXELS_BUFFER_LEN);
  }

  // setup old framebuffer for drawing
  cur_framebuffer = 0; // ONLY 1 BUF !cur_framebuffer;
  for (int i = 0; i < SCREEN_HEIGHT; i++) {
    cur_framebuffer_lines[i] = &framebuffers[cur_framebuffer][i*HPIXELS_BUFFER_LEN];
  }
}

void vga_clear_screen(unsigned char color)
{
  clear_framebuffer(cur_framebuffer, color);
}

int vga_init(const struct VGA_MODE *mode, unsigned int pin_out_base, void** framebuffer)
{
  vga_mode = mode;

  int err = init_buffers(1);
  if (err < 0) return err;

  if(framebuffer) *framebuffer = framebuffers[0];

  err = init_pio(pin_out_base);
  if (err < 0) return err;

  vga_screen.width       = SCREEN_WIDTH;
  vga_screen.height      = SCREEN_HEIGHT;
  vga_screen.sync_bits   = SYNC_BITS;
  vga_screen.framebuffer = cur_framebuffer_lines;

  // setup first framebuffer
  cur_framebuffer = 0;
  vga_swap_buffers(false);

  // start video output
  dma_channel_start(dma_control_chan);
  return 0;
}
