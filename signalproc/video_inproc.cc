
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




#include "pico/time.h"
#include "hardware/clocks.h"
#include "../system_defs.h"

#include "video_inproc.h"




VideoInProc::VideoInProc(InputSampler* sampler, pixel_t* screenbuf)
 : sampler_(sampler), screenbuf_start_(screenbuf), rd_buf_(nullptr),
    vsync_cycle_max_(0), vsync_cycle_min_(0), diag_duration_us_(0)

{ }

void VideoInProc::PrintDiagInfo(){
    double syncline_us = vsync_cycle_ * 1e6 / SMPLFREQ_HZ;
    diag_printf("  | HSync %.3fus |",syncline_us);
    //diag_printf("vs min %d, max %d. ", vsync_cycle_min_, vsync_cycle_max_);
    double samples_per_pixel = hsync_hrperiod_.float_index()/SCR.num_hsync_line_pixel;
    if(samples_per_pixel){
        double porchdelay = hs_to_pixel_hrduration_.float_index()/samples_per_pixel;
        diag_printf(" BPorch %.2f pix |",porchdelay);
    }
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    diag_printf(" Sysclk %d MHz |",f_clk_sys/1000);

    diag_printf(" CpuLd %.1f%% | t=%dus |\n", sampler_->GetCpuLoadPercent(), diag_duration_us_);
   
}

// Check for loading next buffer or going for timeout
bool VideoInProc::CheckBuf(size_t* rd_ix){
    while(*rd_ix >= InputSampler::BUF_SIZE){
        if(rd_cnt_timeout_ <= InputSampler::BUF_SIZE){
            rd_cnt_timeout_ = 0;
            return false;
        }
        rd_cnt_timeout_ -= InputSampler::BUF_SIZE;
        rd_counter_ += InputSampler::BUF_SIZE;
        *rd_ix -= InputSampler::BUF_SIZE;
        rd_buf_ = sampler_->GetNextSampleBuffer(true);
    }
    return true;
}



bool VideoInProc::WaitForDuration(uint32_t min_time_usec, uint8_t level){
    uint32_t num_steps = 0;
    uint32_t required_steps = min_time_usec+1;
    const size_t SCREENING_STEP = SMPLFREQ_HZ/1000000;    // 1 usec step for screening

    for(;;){
        if (!((rd_buf_[rd_ix_] && SMPL_VSYNC) ^ level)){
            ++num_steps;
            if (num_steps >= required_steps) return true;
        } else {
            num_steps=0;
        }
        rd_ix_+=SCREENING_STEP;
        if(!CheckBuf(&rd_ix_)) return false;
    }

}


bool VideoInProc::ProcessFrame(uint32_t diag_display_lines){
    uint32_t last_vsync=0;
    uint32_t vsync_count=0;
    uint32_t vsync_sum=0;
    uint32_t vsync_max=0;
    uint32_t vsync_min=0;
    uint32_t duration=0;
    const size_t STEP = 32;  // 1 usec step for screening

    rd_counter_=0;

    // flush
    sampler_->GetNextSampleBuffer();
    sampler_->GetNextSampleBuffer();

    rd_buf_ = sampler_->GetNextSampleBuffer();
    rd_ix_ = 0;


    rd_cnt_timeout_ = SMPLFREQ_HZ/20; // frame start plus full frame == two frames
    
    // wait for VSync = CSYNC low > 40us
    if (!WaitForDuration(40, 0) ) return false;
    // wait for first line after VSYNC = CSYNC high > 45us
    if (!WaitForDuration(45, 1) ) return false;

    // start syncing, go to next step block end
    rd_ix_ = rd_ix_ | (STEP-1);
    uint32_t syncstart_ix = rd_ix_ & ~(STEP-1);

    // measure vsync
    while(vsync_count < SCR.num_upper_lines-2){
        // Wait for sync end
        while(!(rd_buf_[rd_ix_] && SMPL_VSYNC )){
            rd_ix_+=STEP;
            if(!CheckBuf(&rd_ix_)) return false;
        }
        // Look for rough sync start
        while((rd_buf_[rd_ix_] && SMPL_VSYNC )){
            rd_ix_+=STEP;
            if(!CheckBuf(&rd_ix_)) return false;
        }
        // Look some samples back for fine sync start
        syncstart_ix = rd_ix_ & ~(STEP-1);
        while((rd_buf_[syncstart_ix] && SMPL_VSYNC )) syncstart_ix++;


        duration = rd_counter_ + syncstart_ix - last_vsync;
        rd_counter_ = 0;
        last_vsync = rd_counter_ + syncstart_ix;
        if(vsync_count>2){
            vsync_max = std::max<uint32_t>(vsync_max, duration);
            vsync_min = std::min<uint32_t>(vsync_min, duration);
            vsync_sum += duration;
        }
        else if(vsync_count==2) {
            vsync_max = duration;
            vsync_min = duration;
            vsync_sum = 0;
        }
        ++vsync_count;

    }


    vsync_cycle_ = static_cast<decltype(vsync_cycle_)>(vsync_sum)/(vsync_count-3);
    if(vsync_cycle_max_==0) vsync_cycle_max_ = vsync_max;
    vsync_cycle_max_ = std::max<uint32_t>(vsync_max, vsync_cycle_max_);
    if(vsync_cycle_min_==0) vsync_cycle_min_ = vsync_min;
    vsync_cycle_min_ = std::min<uint32_t>(vsync_min, vsync_cycle_min_);

    // check if the vsync is in the expected range
    double syncline_us = vsync_cycle_ * 1e6 / SMPLFREQ_HZ;
    if(syncline_us < SCR.hsync_line_microsec*0.8)  return false;
    if(syncline_us > SCR.hsync_line_microsec*1.25)  return false;

    // calculate the timings for scanning the sceen
    const HiresBufIx HR_SAMPLE_DUR(1);
    hsync_hrperiod_ = static_cast<double>(vsync_sum)/(vsync_count-3.0);
    HiresBufIx pixel_hrdur{hsync_hrperiod_/SCR.num_hsync_line_pixel};
    if(hs_to_pixel_hrduration_==0) hs_to_pixel_hrduration_ = pixel_hrdur * SCR.num_hsync_to_disp_pixel;
    HiresBufIx syncstart_hrix{syncstart_ix};
    syncstart_hrix += hsync_hrperiod_ - HR_SAMPLE_DUR/2;  // half pixel back as this was the first active pixel 
    const HiresBufIx BUF_SIZE_HR{InputSampler::BUF_SIZE};


    pixel_t* scr_wr = screenbuf_start_;
    //for(uint32_t pix=0; pix < 512; pix++){
    //    auto col=PixelFromRGB(255, 0,0);
    //    *scr_wr++=col;
    //}

    absolute_time_t blocktime_start = get_absolute_time();
    diag_duration_us_=0;

    int16_t scanline;
    for(scanline=-2;  scanline < static_cast<decltype(scanline)>(SCR.num_vert_displ_pixel-diag_display_lines); scanline++){
        // blindly move to next sync, we assume the period fits
        while(syncstart_hrix >= BUF_SIZE_HR) {
            diag_duration_us_ += absolute_time_diff_us(blocktime_start , get_absolute_time()); 
            rd_buf_ = sampler_->GetNextSampleBuffer();
            blocktime_start = get_absolute_time();
            syncstart_hrix-=BUF_SIZE_HR;
        }

        // incrementally re-adjust exact phase for next line
        if(rd_buf_[syncstart_hrix.index()] && SMPL_VSYNC) {
            // pulse starts later, slightly stretch the line so we move closer to the edge
            syncstart_hrix += HR_SAMPLE_DUR/8;
        } else {
            // pulse already started, slightly shorten the next line so we move closer to the edge
            syncstart_hrix -= HR_SAMPLE_DUR/8;
        }

        // move to pixel read position
        auto read_hrix = syncstart_hrix + hs_to_pixel_hrduration_;
        syncstart_hrix += hsync_hrperiod_; // proceed to next line sync already
        
        if(scanline<0) continue;  // first lines only for sync adjust

        while(read_hrix >= BUF_SIZE_HR) {
            diag_duration_us_ += absolute_time_diff_us(blocktime_start , get_absolute_time()); 
            rd_buf_ = sampler_->GetNextSampleBuffer();
            blocktime_start = get_absolute_time();
            read_hrix-=BUF_SIZE_HR;
            syncstart_hrix-=BUF_SIZE_HR;
        }

        // check if we should now do some adjustments
        if( (scanline&0x1F) == 0){
            // do this only every now and then, to not put too much burden on a single buffer processing..
            if( read_hrix > pixel_hrdur && read_hrix + pixel_hrdur*SCR.num_hori_displ_pixel < BUF_SIZE_HR){
                auto new_hrdur=hs_to_pixel_hrduration_;
                // check for pixel oputside screen
                if(rd_buf_[ (read_hrix-pixel_hrdur).index() ] & SMPL_ANYCOLOR ) new_hrdur+=pixel_hrdur;
                if(rd_buf_[ (read_hrix+pixel_hrdur*SCR.num_hori_displ_pixel).index() ] & SMPL_ANYCOLOR ) new_hrdur-=pixel_hrdur;
                // adjust on pixel transitions
                auto adjust_hrix = read_hrix;
                uint8_t numchanges=0;
                pixel_t lastval=rd_buf_[adjust_hrix.index()];
                for(uint32_t probe = 0; probe < 10; probe++){
                    size_t ix = adjust_hrix.index();
                    if(rd_buf_[ix]!=lastval) numchanges++;
                    if(rd_buf_[ix-1] != rd_buf_[ix] ) new_hrdur += HR_SAMPLE_DUR/32; // need to sample later
                    if(rd_buf_[ix+1] != rd_buf_[ix] ) new_hrdur -= HR_SAMPLE_DUR/32; // need to sample earlier
                    adjust_hrix += pixel_hrdur;
                }
                // We need at least two edges to not slip in one both direction.
                // Also, make sure we do not move outside the rough 40..160 window, as default was 109 an no bigger deviation expected.
                if(numchanges>=2 && new_hrdur > hsync_hrperiod_/16 && new_hrdur < hsync_hrperiod_/4  ){
                    hs_to_pixel_hrduration_=new_hrdur; 
                }
            }
        }

        // check if we have the full line in buffer, or need to reload in between
        uint32_t pix=0;
        while(pix < SCR.num_hori_displ_pixel){
            if(read_hrix + pixel_hrdur*(SCR.num_hori_displ_pixel-pix) >= BUF_SIZE_HR){
                // running into buffer end limit
                while(read_hrix < BUF_SIZE_HR){
                    pixel_t col = rd_buf_[read_hrix.index()];
                    col|=VIDOUT_SYNC_BITS;
                    *scr_wr++=col;
                    read_hrix += pixel_hrdur;
                    pix++;
                }
                // reload
                while(read_hrix >= BUF_SIZE_HR) {
                    diag_duration_us_ += absolute_time_diff_us(blocktime_start , get_absolute_time()); 
                    rd_buf_ = sampler_->GetNextSampleBuffer();
                    blocktime_start = get_absolute_time();
                    read_hrix-=BUF_SIZE_HR;
                    syncstart_hrix-=BUF_SIZE_HR;
                }
            } else {
                // just free running till end of line
                while(pix < SCR.num_hori_displ_pixel){
                    pixel_t col = rd_buf_[read_hrix.index()];
                    col|=VIDOUT_SYNC_BITS;
                    *scr_wr++=col;
                    read_hrix += pixel_hrdur;
                    pix++;
                }
            }
        }
    }
    diag_duration_us_ += absolute_time_diff_us(blocktime_start , get_absolute_time()); 
    diag_duration_us_ /= scanline;



    //for(uint32_t pix=0; pix < 512*1; pix++){
    //    auto col=PixelFromRGB(0, 255,255);
    //    *scr_wr++=col;
    //}

    return true;

}

