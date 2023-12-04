add_library(ina219 INTERFACE)

target_include_directories(ina219
    INTERFACE
    ./include)

target_sources(ina219
    INTERFACE
    ina219.cc
)

target_link_libraries(ina219
    INTERFACE
    pico_stdlib 
    hardware_i2c
)