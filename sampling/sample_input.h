#pragma once


#include <stdint.h>
#include <vector>
#include "pico/time.h"
#include "hardware/pio.h"


typedef uint8_t sample_t;


class InputSampler {

public:
    explicit InputSampler(uint32_t sample_pin_base, uint32_t clk_Hz);

    // Read from a queue of sample buffers. Buffer is fixed and of limited size, so 
    // reading the buffers should be done at highest speed.
    sample_t* GetNextSampleBuffer(bool blocking=true);

    uint32_t GetSampleClockHz(){return sampleclock_hz_;}

    double GetCpuLoadPercent();

    static const uint32_t BUF_SZ_BITS = 13;
    static const uint32_t BUF_SIZE = (1<<BUF_SZ_BITS);

private:
    static const uint PIN_COUNT = 8;

    uint32_t sampleclock_hz_;

    // Statistics for calculating CpuLoad etc.
    uint32_t consec_reads_; 
    uint32_t consec_rd_overall_time_us_;
    uint32_t consec_rd_wait_time_us_;
    absolute_time_t last_read_timestamp_;

    void SetupDma(PIO pio, uint sm);


    //absolute_time_t get_absolute_time(void) 

};