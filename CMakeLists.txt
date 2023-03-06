add_executable(video_if_ql_vga)

target_sources(video_if_ql_vga PRIVATE main.cc)
target_sources(video_if_ql_vga PRIVATE video_out/vid_render.cc)
target_sources(video_if_ql_vga PRIVATE video_out/vga/vga_6bit.c)
target_sources(video_if_ql_vga PRIVATE sampling/sample_input.cc)
target_sources(video_if_ql_vga PRIVATE signalproc/video_inproc.cc)

pico_generate_pio_header(video_if_ql_vga ${CMAKE_CURRENT_LIST_DIR}/video_out/vga/vga_6bit.pio)

target_compile_definitions(video_if_ql_vga PRIVATE
        )

#  change the start-up delay for PR2040PICO board that may need more time (thanks Stephan for the hint!)
add_compile_definitions( PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64 )

target_link_libraries(video_if_ql_vga PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_dma
        )

pico_add_extra_outputs(video_if_ql_vga)


