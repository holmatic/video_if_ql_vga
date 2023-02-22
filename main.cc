

/*
* QLpicoVGA - Video Interface QL-to-VGA with a Pi Pico
* Copyright (C) 2023 holmatic
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/






//#include <vector>
#include <cmath>
#include <cstring>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#include "pico.h"
#include "pico/stdlib.h"   // sleep_us etc
#include "pico/util/queue.h"
#include "pico/binary_info.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/gpio.h"


#include "system_defs.h"
#include "video_out/vga/vga_6bit.h"
#include "video_out/vid_render.h"
#include "sampling/sample_input.h"
#include "signalproc/video_inproc.h"




CU_REGISTER_DEBUG_PINS(generation)



// For diagnostic and error output
static queue_t dprint_queue;


void diag_printf(const char* format, ...){
    char prtbuf[32];  // on stack for reentrance
    va_list args;
    va_start (args, format);
    int num=vsnprintf(prtbuf, sizeof(prtbuf), format, args);
    va_end (args);
    for(int i=0; i<num; ++i){
        queue_try_add(&dprint_queue, &prtbuf[i]);
    }
}

size_t fw_check_diag_printf_(char* readbuf){
    size_t num = queue_get_level(&dprint_queue);
    if(num && readbuf) queue_remove_blocking(&dprint_queue, readbuf);
    return num;
}




namespace{

    void show_upper_diagnostics(VideoWindow& w1, InputSampler& ismp, bool nosignal){
        const uint32_t SAMPLESIZE=8192;
        // read some sampled data, first dummy read to recover from overrrun condition
        ismp.GetNextSampleBuffer();
        ismp.GetNextSampleBuffer();
        // collect enough buffers
        std::vector<uint8_t> smpl;
        smpl.resize(SAMPLESIZE);
        size_t wr_ix=0;
        while( wr_ix < SAMPLESIZE){
            size_t chunksize=InputSampler::BUF_SIZE;
            uint8_t* buf = ismp.GetNextSampleBuffer(true);      // blocking call
            std::memcpy(&smpl[wr_ix], buf, chunksize);
            wr_ix+=chunksize;
        }
        // plot result
        pixel_t c_hi = PixelFromRGB(0, 255, 000);
        pixel_t c_lo = PixelFromRGB(255, 255, 255);
        pixel_t c_bg = PixelFromRGB(64, 64, 128);

        w1.SetXyPos(5,5,2);
        w1.PrintF("Sinclair NU1500 Pico VGA Adapter");
        w1.SetXyPos(w1.getScrWidthPixel()+325-512,33);
        w1.PrintF("by ZX TEAM 2023");

        float xscale = (w1.getScrWidthPixel()-52.0) / SAMPLESIZE;  // 460

        w1.SetXyPos(3, 66);
        w1.PrintF("CSync");
        w1.SetXyPos(3, 78);
        w1.PrintF(" Blue");
        w1.SetXyPos(3, 90);
        w1.PrintF("Green");
        w1.SetXyPos(3, 102);
        w1.PrintF("  Red");

        uint16_t oldxpos = 0;
        for(size_t t=0; t<SAMPLESIZE; t++){
            uint16_t xpos = 38+std::round(t*xscale);
            if(xpos!=oldxpos){
                for(uint16_t bit=0; bit<8; bit+=2){
                    // clean any previous traces
                    w1.DrawPixelRaw(xpos, 73 + bit*6, c_bg);
                    w1.DrawPixelRaw(xpos, 71 + bit*6, c_bg);
                }
                oldxpos=xpos;
            }
            for(uint16_t bit=0; bit<8; bit+=2){
                if(smpl[t] & (0x80>>bit) ){
                    // high
                    w1.DrawPixelRaw(xpos, 71 + bit*6, c_hi);
                } else {
                    // low
                    w1.DrawPixelRaw(xpos, 73 + bit*6, c_lo);
                }
            }
        }
        w1.SetXyPos(w1.getScrWidthPixel()/2 + 185-256, 145 ,2);
        w1.PrintF(nosignal ? " Signal ? ": "           " );
        w1.SetXyPos(w1.getScrWidthPixel()+465-512,205);
        w1.PrintF("v0.nu1");
    }



    void run_main_loop_forever(pixel_t* video_buffer, VidType vt)
    {
        const uint32_t diag_lines=20;

        uint16_t scwidth = vt==QL_NATIVE ? 512: 320;
        uint16_t scheight = vt==QL_NATIVE ? 256: 240;
        VideoRenderer v(video_buffer, scwidth, scheight);
        VideoWindow w1(&v,0, scheight-diag_lines, PixelFromRGB(0, 255, 255), PixelFromRGB(64, 64, 128));
        VideoWindow w2(&v, scheight-diag_lines,scheight, PixelFromRGB(128, 128, 128), PixelFromRGB(64, 64, 128));

        InputSampler ismp(SAMPLEIN_PIN_BASE, 32000000);
        VideoInProc vid_in(&ismp, video_buffer, vt);

        uint32_t ok_frames=1;
        uint32_t failed_frames=0;
        uint32_t upper_diag_active=0;
        uint32_t lower_diag_active=0;

        // Draw some fancy initial pattern for the initial screen.
        auto c=PixelFromRGB(255, 0, 0);
        for(uint32_t y=0; y<scheight; y+=4){
            for(uint32_t x=0; x<scwidth; x+=4){
                if(y%32==16 || x%44==36) video_buffer[y*scwidth+x]=c;
            }
        }

        // Enter infinite loop.
        for(uint32_t cyclecount=0;;cyclecount++){

            if(gpio_get(USER_PUSHBUTTON_PIN)==0){
                ok_frames=0;
                lower_diag_active=50;
                upper_diag_active=50;
                sleep_ms(20);
            } else {
                if (vid_in.ProcessFrame(lower_diag_active ? diag_lines:0)){
                    ok_frames++;
                    failed_frames=0;
                    upper_diag_active=0;
                    w1.MarkClearPending();
                    if(!lower_diag_active) w2.MarkClearPending();
                } else {
                    failed_frames++;
                    upper_diag_active++;
                    if(failed_frames>20) ok_frames=0;
                }
            }
            if(lower_diag_active && (cyclecount%10==0) ){
                if(upper_diag_active || failed_frames)
                    w2.Clear();  // do not show the lowe part when anything is disturbing
                else{
                    w2.MarkClearPending();
                    vid_in.PrintDiagInfo();
                }
                lower_diag_active--;
            }

            gpio_put(PICO_DEFAULT_LED_PIN, (cyclecount&0x1F)<2 ? 1: 0  );

            if(upper_diag_active>=50 && (cyclecount%20==5)){
                show_upper_diagnostics(w1, ismp, failed_frames>20);
            }
            
            // collect any diagnostics
            while(fw_check_diag_printf_(NULL)){
                char c;
                fw_check_diag_printf_(&c);
                w2.PutChar(c);
            }
        }
    }


    void init_low_level_parts(pixel_t** framebuffer_pt, VidType vt){

        stdio_init_all();

        if(INCREASED_SYS_CLOCK){
            vreg_set_voltage(VREG_VOLTAGE_1_30);
            sleep_ms(10);
            set_sys_clock_khz(48000*5, false);  // up to 6 is used in demos, stick with 5 for now
        } else {
            set_sys_clock_khz(48000*3, false);
        }

        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);

        // Declare the video output pins.
        bi_decl_if_func_used(bi_pin_mask_with_name(3 << (VGA_PIN_BASE + 4), "Blue 0-1"));
        bi_decl_if_func_used(bi_pin_mask_with_name(3 << (VGA_PIN_BASE + 2), "Green 0-1"));
        bi_decl_if_func_used(bi_pin_mask_with_name(3 << (VGA_PIN_BASE + 0), "Red 0-1"));
        bi_decl_if_func_used(bi_1pin_with_name(VGA_PIN_BASE + 6, "H-Sync"));   // Goes to 16
        bi_decl_if_func_used(bi_1pin_with_name(VGA_PIN_BASE + 7, "V-Sync"));      // Goes to 17

        // Declare the video input pins used for sampling
        bi_decl_if_func_used(bi_1pin_with_name( SAMPLEIN_PIN_BASE + 7, "SAMPLE C-Sync"));   // 9
        bi_decl_if_func_used(bi_1pin_with_name( SAMPLEIN_PIN_BASE + 5, "SAMPLE Blue"));     // 7
        bi_decl_if_func_used(bi_1pin_with_name( SAMPLEIN_PIN_BASE + 3, "SAMPLE Green"));    // 5  
        bi_decl_if_func_used(bi_1pin_with_name( SAMPLEIN_PIN_BASE + 1, "SAMPLE Red"));      // 3

        // Pushbutton input with pullup for diagnostcs
        gpio_init(USER_PUSHBUTTON_PIN);
        gpio_set_dir(USER_PUSHBUTTON_PIN, GPIO_IN);
        gpio_pull_up(USER_PUSHBUTTON_PIN);
        bi_decl_if_func_used(bi_1pin_with_name( USER_PUSHBUTTON_PIN, "User Pushbutton (to GND)"));      // 3

        // Some blinking.
        for(int i=0;i<3;i++){
            sleep_ms(30);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(30);
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
        }

        // Init the VGA library. If for some reason this does not work (out of mem), show a blinking pattern on the LED.
        VGA_MODE vgamode = (vt==QL_NATIVE) ? vga_mode_512x256 : vga_mode_320x240;
        
        if (vga_init(&vgamode , VGA_PIN_BASE, reinterpret_cast<void**>(framebuffer_pt)) < 0) {
            printf("ERROR initializing VGA\n");
            fflush(stdout);
            while(1){
                gpio_put(PICO_DEFAULT_LED_PIN, 1);
                sleep_ms(1000);
                gpio_put(PICO_DEFAULT_LED_PIN, 0);
                sleep_ms(1000);
            }
        }
        vga_clear_screen(0x3F);
        queue_init(&dprint_queue, sizeof(uint8_t), 150);
        setup_default_uart();
    }
}


int main(void){
    VidType vt = ZX_NU_1500;    
    pixel_t* framebuffer;
    init_low_level_parts(&framebuffer, vt);
    run_main_loop_forever(framebuffer, vt);
    return 1;
} 