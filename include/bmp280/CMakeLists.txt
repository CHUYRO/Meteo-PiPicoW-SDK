add_library(bmp280 INTERFACE)

target_include_directories(bmp280
    INTERFACE
    ./include)

target_sources(bmp280
    INTERFACE
    bmp280.cpp
)

target_link_libraries(bmp280
    INTERFACE
    pico_stdlib 
    hardware_spi
)