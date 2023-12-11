add_library(wifiLib INTERFACE)

target_include_directories(wifiLib
    INTERFACE
    ./include)

target_sources(wifiLib
    INTERFACE
    wifiLib.cpp
)

target_link_libraries(wifiLib
    INTERFACE
    pico_stdlib        
    pico_cyw43_arch 
)