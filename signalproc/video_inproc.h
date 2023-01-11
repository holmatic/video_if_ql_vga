
#pragma once


#include <stdint.h>
#include <vector>

#include "../sampling/sample_input.h"
#include "../video_out/vid_render.h"
#include "hires_buffer_index.h"





struct ScreenParameters{
    uint32_t num_hori_displ_pixel;
    uint32_t num_vert_displ_pixel;
    uint32_t num_hsync_line_pixel;
    uint32_t num_upper_lines;
    uint32_t num_hsync_to_disp_pixel;
    uint32_t hsync_line_microsec;
};

// Specific parameters for the video input signal 
namespace scrpara{
    constexpr ScreenParameters QL_SCR_PARA = ScreenParameters{512, 256, 640, 27, 109, 64};
};
class VideoInProc{

public:
    explicit VideoInProc(InputSampler* sampler, pixel_t* screenbuf);

    bool ProcessFrame(uint32_t diag_display_lines=0);

    void PrintDiagInfo();

private:

    static const sample_t SMPL_VSYNC = 0x80;
    static const sample_t SMPL_ANYCOLOR = 0x15;
    static const uint32_t SMPLFREQ_HZ = 32000000;
    static const size_t ADJUST_BUF_SAMPLES = 32;

    static constexpr ScreenParameters SCR = scrpara::QL_SCR_PARA;


    sample_t GetSample();
    bool CheckBuf(size_t* index);

    bool WaitForDuration(uint32_t min_time_usec, uint8_t level);

    InputSampler* sampler_;
    pixel_t* screenbuf_start_;
    sample_t* rd_buf_;
    size_t rd_ix_ = 0;

    HiresBufIx hs_to_pixel_hrduration_ = 0;
    HiresBufIx hsync_hrperiod_ = 0;

    uint32_t rd_counter_ = 0;
    uint32_t rd_cnt_timeout_ = 0xffffffff;

    double vsync_cycle_ = 0.0;
    uint32_t vsync_cycle_max_;
    uint32_t vsync_cycle_min_;

    uint32_t diag_duration_us_;

};