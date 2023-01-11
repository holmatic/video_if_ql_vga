#include <cstring>
#include <cstdarg>
#include <cstdio>
#include "pico.h"
#include "pico/stdlib.h"   // sleep_us etc
#include "pico/stdio.h"  
#include "vid_render.h"

#include "charset_code.inc"  // const uint8_t charset16x6[]




pixel_t PixelFromRGB(uint8_t r, uint8_t g, uint8_t b){
    return (0xc0  | (b&0xc0) >> 2) | ((g&0xc0) >> 4)| ((r&0xc0) >> 6);
}


VideoRenderer::VideoRenderer(pixel_t* video_buffer)
    : video_buf_(video_buffer)
{
    ClearScreen(PixelFromRGB(255, 255, 255));
}


void VideoRenderer::ClearScreen(pixel_t color)
{
    pixel_t* pt = video_buf_;
    for(int16_t y=0; y<kScreenHeight; y++){
        for(int16_t x=0; x<kScreenWidth; x++){
            *pt++ = color;
        }
    }
}


pixel_t* VideoRenderer::GetAddrFromPos(int16_t xpos, int16_t ypos)
{
    pixel_t* addr = &video_buf_[xpos+VideoRenderer::kScreenWidth*ypos];
    return addr;
}

pixel_t* VideoWindow::GetAddrFromPos(int16_t xpos, int16_t ypos){
    return video_->GetAddrFromPos(xstart_+xpos, ystart_+ypos);
}




VideoWindow::VideoWindow(VideoRenderer* video, int16_t y_start, int16_t y_end, pixel_t fg, pixel_t bg)
 : video_(video), xstart_(0), xend_(512), ystart_(y_start), yend_(y_end), 
   xpos_(0), ypos_(0), fg_color_(fg), bg_color_(bg)
{
}

static char PrintBuf[150];

const int kCharWidth = 6;
const int kCharHight = 12;


void VideoWindow::PrintF(const char* format, ...){
    va_list args;
    va_start (args, format);
    int num=vsnprintf(PrintBuf, sizeof(PrintBuf), format, args);
    va_end (args);
    
    for(int i=0; i<num; ++i){
        char c=PrintBuf[i];
        PutChar(c);
    }
}

void VideoWindow::NewLine()
{
    xpos_ = 0;
    if (ypos_ + ystart_ + 2*kCharHight*charscale_ < yend_)
        ypos_ += kCharHight*charscale_;
    else
        scroll_pending_= true;
    pos_addr_= GetAddrFromPos(xpos_, ypos_);
}

void VideoWindow::PutChar(uint16_t character)
{
    if(clear_pending_) Clear();
    if(scroll_pending_){
        Scroll(kCharHight*charscale_);
        scroll_pending_=false;
    }
    if(character>=32 && character<128){
        const uint8_t* src = &charset16x6[16*(character&0x007f)];
        // Assume pos_addr_ allows us to draw directly there
        if(charscale_==1){
            for(int y=0; y<kCharHight; y++){
                uint8_t pattern = *src++;
                for(int x=0; x<kCharWidth; x++){
                    *pos_addr_++ = (pattern&0x20) ? fg_color_ : bg_color_;
                    pattern <<= 1;
                }
                pos_addr_ += VideoRenderer::kScreenWidth-kCharWidth;
            }
            xpos_ += kCharWidth;
        } else  {
            for(int y=0; y<kCharHight; y++){
                uint8_t pattern = *src++;
                for(int x=0; x<kCharWidth; x++){
                    auto p = (pattern&0x20) ? fg_color_ : bg_color_;
                    for(int ysc=0; ysc < charscale_; ysc++){
                        for(int xsc=0; xsc < charscale_; xsc++){
                            pos_addr_[ysc*VideoRenderer::kScreenWidth + xsc] = p;
                        }
                    }
                    pos_addr_+=charscale_;
                    pattern <<= 1;
                }
                pos_addr_ += charscale_ * (VideoRenderer::kScreenWidth-kCharWidth);
            }
            xpos_ += kCharWidth*charscale_;
        }
        // Check here where to put next char
        if (xpos_ + xstart_ + kCharWidth*charscale_ > xend_) NewLine();

    }else{
        switch(character){
            case '\012':
                NewLine();
                break;
        }
    }
    pos_addr_= GetAddrFromPos(xpos_, ypos_);
}

void VideoWindow::SetXyPos(int16_t x_pos, int16_t y_pos, int16_t charscale){
    if(clear_pending_) Clear();
    xpos_ = x_pos;
    ypos_ = y_pos;
    charscale_ = charscale;
    pos_addr_ = GetAddrFromPos(xpos_, ypos_);
}


void VideoWindow::Scroll(uint16_t num_pix)
{
    pixel_t* dest = GetAddrFromPos(0, 0);
    pixel_t* src = GetAddrFromPos(0, num_pix);
    uint16_t skippix = VideoRenderer::kScreenWidth-xend_+xstart_;
    int16_t y;
    for(y=ystart_; y+num_pix<yend_; y++){
        memcpy(dest, src, (xend_*xstart_)*sizeof(pixel_t));
        src += VideoRenderer::kScreenWidth;
        dest+= VideoRenderer::kScreenWidth;
    }
    for(; y<yend_; y++){
        for(int16_t x=xstart_; x<xend_; x++){
            *dest++ = bg_color_;
        }
        src += skippix;
        dest += skippix;
    }
}

void VideoWindow::Clear()
{
    xpos_=0;
    ypos_=0;
    pixel_t* pt = pos_addr_ = GetAddrFromPos(xpos_, ypos_);
    clear_pending_ = false;
    scroll_pending_ = false;
    for(int16_t y=ystart_; y<yend_; y++){
        for(int16_t x=xstart_; x<xend_; x++){
            *pt++ = bg_color_;
        }
        pt += VideoRenderer::kScreenWidth-xend_+xstart_;
    }
}

