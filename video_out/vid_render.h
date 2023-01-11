
#pragma once


#include <stdint.h>

typedef uint8_t pixel_t;


extern pixel_t PixelFromRGB(uint8_t r, uint8_t g, uint8_t b);

class VideoRenderer
{
public:

    VideoRenderer(pixel_t* video_buffer);
    void ClearScreen(pixel_t color);

    pixel_t* GetAddrFromPos(int16_t xpos, int16_t ypos);
    void DrawPixelRaw(int16_t x, int16_t y,  pixel_t p) {if(y<kScreenHeight) video_buf_[kScreenWidth*y + x] = p; }

    static const int16_t kScreenWidth=512;
    static const int16_t kScreenHeight=256;

private:
    pixel_t* video_buf_; // Screen buffer
};

class VideoWindow
{
public:
    VideoWindow(VideoRenderer* rnd, int16_t y_start, int16_t y_end, pixel_t fg, pixel_t bg);

    void Clear();
    void PutChar(uint16_t character_ix);
    void PrintF(const char* fmt, ...);
    void DrawPixelRaw(int16_t x, int16_t y,  pixel_t p) {video_->DrawPixelRaw(x+xstart_,y+ystart_,p); }
    void MarkClearPending() {clear_pending_ = true;}
    void SetXyPos(int16_t x_pos, int16_t y_pos, int16_t charscale = 1);

private:
    
    pixel_t* GetAddrFromPos(int16_t xpos, int16_t ypos);
    void Scroll(uint16_t num_pix);
    void NewLine();

    VideoRenderer* video_; // Screen buffer
    int16_t xstart_, xend_, ystart_, yend_;
    pixel_t* pos_addr_;
    int16_t xpos_, ypos_;
    pixel_t fg_color_, bg_color_;
    int16_t charscale_ = 1;
    bool scroll_pending_ = false;
    bool clear_pending_ = true;
};




