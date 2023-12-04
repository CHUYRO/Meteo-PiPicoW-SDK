add_library(dht22 INTERFACE)

pico_generate_pio_header(dht22 ${CMAKE_CURRENT_LIST_DIR}/dht22.pio)

target_include_directories(dht22
    INTERFACE
    ./include)

target_sources(dht22
    INTERFACE
    dht22.c
)

target_link_libraries(dht22
    INTERFACE
    hardware_clocks
    hardware_dma
    hardware_pio
)
