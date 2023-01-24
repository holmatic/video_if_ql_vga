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


#include <stdio.h>
#include <stdlib.h>
#include <array>

#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/clocks.h"
#include "../system_defs.h"
#include "sample_input.h"



static inline uint bits_packed_per_word(uint pin_count) {
    // If the number of pins to be sampled divides the shift register size, we
    // can use the full SR and FIFO width, and push when the input shift count
    // exactly reaches 32. If not, we have to push earlier, so we use the FIFO
    // a little less efficiently.
    const uint SHIFT_REG_WIDTH = 32;
    return SHIFT_REG_WIDTH - (SHIFT_REG_WIDTH % pin_count);
}

namespace SampleBuf{
    // Interrupt counter, lowest bit determines the buffer that was filled.
    uint32_t overrun_count=0;

    std::array<uint,2> dma_chan;

    // Each completion interrupt will write the number matching the filled buffer.
    // Application program can block on this waiting for next data. According to doc,
    // certain operations are interrupt save
    queue_t buf_done_queue;

    const uint32_t BUF_SZ_BITS = InputSampler::BUF_SZ_BITS;
    const uint32_t BUF_SIZE = (1<<BUF_SZ_BITS);
    const uint32_t BUF_ADDR_MASK = (1<<BUF_SZ_BITS)-1;

    // Aligned for the DMA ring address warp.
    sample_t buf[BUF_SIZE*2] __attribute__((aligned(BUF_SIZE)));


    void __isr __time_critical_func(dma_handler)() {
        static uint8_t buf_ix=0;  // last buffer index
        if(dma_channel_get_irq0_status(dma_chan[buf_ix^1])){
            buf_ix=buf_ix^1;
        } else if(dma_channel_get_irq0_status(dma_chan[buf_ix])) {
            overrun_count++; // two times the same channel, should not happen during normal operation ...
        }
        else return; // no request pending

        if(!queue_is_full(&buf_done_queue)){
            queue_try_add(&buf_done_queue,&buf_ix);
        }
        else overrun_count++;
        // Clear the interrupt request.
        dma_channel_acknowledge_irq0(dma_chan[buf_ix]);
        //dma_channel_acknowledge_irq0(dma_chan[(int_count&1)^1]);

        // TEST ONLY: buf[BUF_SIZE*((buf_ix&1)^1)+BUF_SIZE/2]=0x0f; // should be okay to override the other now
    }


};




InputSampler::InputSampler(uint32_t sample_pin_base, uint32_t clk_hz)
 : sampleclock_hz_(clk_hz), consec_reads_(0), consec_rd_overall_time_us_(0), consec_rd_wait_time_us_(0)
{
    // A queue notifies about filled buffers 
    queue_init (&SampleBuf::buf_done_queue, sizeof(uint32_t), 2); // as we only have two buffers, it does not make sense to have more than two elements in the queue
    
    // choose PIO clock divider based on the CPU clock and VGA pixel clock
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    float clock_div = ((float)f_clk_sys * 1000.f) / (float)clk_hz;
    //diag_printf("Sysclk %d kHz, factor %.2f ",f_clk_sys,(double)clock_div);

    // Grant high bus priority to the DMA, so it can shove the processors out
    // of the way. This improves sampling consistency for high data rates and smaller buffers
    // (32MHz/4KB).
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    for(auto i=sample_pin_base; i<sample_pin_base+PIN_COUNT; i++){
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_pull_down(i);
    }

    // Load a program to capture n pins. This is just a single `in pins, n`
    // instruction with a wrap.
    uint16_t capture_prog_instr = pio_encode_in(pio_pins, PIN_COUNT);
    struct pio_program capture_prog = {
            .instructions = &capture_prog_instr,
            .length = 1,
            .origin = -1
    };
    PIO pio = pio1;  // pio0 for VGA
    uint sm = pio_claim_unused_sm(pio, true);  
    uint offset = pio_add_program(pio, &capture_prog);

    // Configure state machine to loop over this `in` instruction forever,
    // with autopush enabled.
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, sample_pin_base);
    sm_config_set_wrap(&c, offset, offset);
    sm_config_set_clkdiv(&c, clock_div);
    // Note that we may push at a < 32 bit threshold if pin_count does not
    // divide 32. We are using shift-to-right, so the sample data ends up
    // left-justified in the FIFO in this case, with some zeroes at the LSBs.
    sm_config_set_in_shift(&c, true, true, bits_packed_per_word(PIN_COUNT));
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, sm, offset, &c);

    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    // todo set up DMA ring buffer, interrupts, and worker callbacks
    SetupDma(pio,sm);


    //pio_sm_exec(pio, sm, pio_encode_wait_gpio(trigger_level, trigger_pin));
    pio_sm_set_enabled(pio, sm, true);

}


void InputSampler::SetupDma(PIO pio, uint sm){
    //uint32_t wordcount = SampleBuf::BUF_SIZE/sizeof(uint32_t);

    for (auto& ch : SampleBuf::dma_chan){
        ch = dma_claim_unused_channel(true);
        //diag_win_->PrintF(" DMA chan %d ", ch);
    }
    for (uint i=0; i<SampleBuf::dma_chan.size(); ++i){
        auto ch = SampleBuf::dma_chan[i];
        auto other_ch = SampleBuf::dma_chan[SampleBuf::dma_chan.size()-1-i];
        bool last_chan = (i+1==SampleBuf::dma_chan.size());
        dma_channel_config cfg = dma_channel_get_default_config(ch);
        // Reading from constant address, writing to incrementing byte addresses
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
        channel_config_set_read_increment(&cfg, false);
        channel_config_set_write_increment(&cfg, true);
        channel_config_set_ring(&cfg, true/*write*/, SampleBuf::BUF_SZ_BITS);
        channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm, false));
        channel_config_set_chain_to(&cfg, other_ch);

        // Tell the DMA to raise IRQ line 0 when the channel finishes a block
        dma_channel_set_irq0_enabled(ch, true);
        // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
        if(last_chan){
            // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
            irq_set_exclusive_handler(DMA_IRQ_0, SampleBuf::dma_handler);
            irq_set_priority(DMA_IRQ_0, 0xff);
            irq_set_enabled(DMA_IRQ_0, true);
        }        
        dma_channel_configure(ch, &cfg,
            // // Aligned for the DMA ring address warp. uint16_t capture_buf1[CAPTURE_DEPTH] __attribute__((aligned(2048)));
            &SampleBuf::buf[SampleBuf::BUF_SIZE*i],    // dst
            &pio->rxf[sm],  // src
            SampleBuf::BUF_SIZE/4,   // transfer count
            last_chan        // start immediately
        );
    }
    //diag_win_->PrintF("\n Sample DMA cfg done ");
    sleep_ms(1500);  // 
}


sample_t* InputSampler::GetNextSampleBuffer(bool blocking){
    uint32_t count = 0;
    if(blocking){
        absolute_time_t blocktime_start = get_absolute_time();
        queue_remove_blocking(&SampleBuf::buf_done_queue, &count);
        consec_rd_wait_time_us_ += absolute_time_diff_us(blocktime_start , get_absolute_time()); 
    }
    else if (!queue_try_remove(&SampleBuf::buf_done_queue, &count)){
        return nullptr;
    }
    if(SampleBuf::overrun_count){
        // Had an overrun, restart
        consec_reads_=0;
        consec_rd_overall_time_us_=0;
        consec_rd_wait_time_us_=0;
        SampleBuf::overrun_count = 0;
    } else {
        consec_reads_+=1;
        consec_rd_overall_time_us_ += absolute_time_diff_us(last_read_timestamp_ , get_absolute_time()); 
    }
    last_read_timestamp_ = get_absolute_time();

    return &SampleBuf::buf[SampleBuf::BUF_SIZE*(count&1)];
}


double InputSampler::GetCpuLoadPercent(){
    if(consec_reads_ < 10){
        return 200.0;
    } else {
        return 100.0*(consec_rd_overall_time_us_-consec_rd_wait_time_us_) / consec_rd_overall_time_us_;
    }
}


