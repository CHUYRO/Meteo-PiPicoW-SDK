cmake_minimum_required(VERSION 3.20)

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/cmake/pico_sdk_import.cmake)
        include(${CMAKE_CURRENT_LIST_DIR}/cmake/pico_sdk_import.cmake)
else()
        message(WARNING "pico_sdk_import.cmake not found!")
endif()

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/cmake/pico_extras_import.cmake)
        include(${CMAKE_CURRENT_LIST_DIR}/cmake/pico_extras_import.cmake)
else()
        message(WARNING "pico_extras_import.cmake not found!")
endif()

project(pico_w_Meteo C CXX)
set(PICO_BOARD pico_w)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_CXX_ENABLE_RTTI 0)
set(PICO_CXX_ENABLE_EXCEPTIONS 0)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.5.1")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.5.1 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

pico_sdk_init()

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/cmake/credentials.cmake)
        include(${CMAKE_CURRENT_LIST_DIR}/cmake/credentials.cmake)
else()
        message(WARNING "Credentials file not found")
endif()

include("ina219/ina219.cmake")
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/dht)

add_executable(main 
        main.cpp 
        bmp/BMP280.hpp
        bmp/BMP280.cpp   
        )

target_compile_definitions(main PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"  
        MQTT_CLIENT_ID=\"${MQTT_CLIENT_ID}\"
        MQTT_CLIENT_PSW=\"${MQTT_CLIENT_PSW}\"   
        )

# enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)   

# create map/bin/hex/uf2 files.         
pico_add_extra_outputs(main)

target_compile_options(main PRIVATE -Wall -Wno-unused-function -Wno-format -Wno-error=unused-variable -Werror=unused-but-set-variable -Wno-maybe-uninitialized -Wno-error=unused-value)

target_include_directories(main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )

target_link_libraries(main        
        pico_stdlib        
        pico_cyw43_arch_lwip_poll 
        pico_lwip_mqtt        
        dht
        ina219
        hardware_rtc
        hardware_adc 
        hardware_sleep  
        hardware_spi     
        )

#Print memory usage
add_custom_command(TARGET main POST_BUILD
COMMAND arm-none-eabi-size ${CMAKE_CURRENT_LIST_DIR}/build/main.elf
VERBATIM)        
string(APPEND CMAKE_EXE_LINKER_FLAGS "-Wl,--print-memory-usage")   