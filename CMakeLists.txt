cmake_minimum_required(VERSION 3.18)

set(PICO_BOARD pico_w)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/utils.cmake)

get_pico_sdk_import_cmake()
# initialize pico-sdk from GIT
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_FETCH_FROM_GIT on)
# pico_sdk_import.cmake is a single file copied from this SDK
# note: this must happen before project()
include(${CMAKE_CURRENT_LIST_DIR}/cmake/pico_sdk_import.cmake)

# We also need PICO EXTRAS for sleep
include(${CMAKE_CURRENT_LIST_DIR}/cmake/pico_extras_import.cmake)

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/cmake/credentials.cmake)
    # copy it over from cmake/credentials.cmake.example
    include(${CMAKE_CURRENT_LIST_DIR}/cmake/credentials.cmake)
else()
    message(WARNING "Credentials file not found, using default values!")
endif()

project(pico_w_Meteo C CXX ASM)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_CXX_ENABLE_EXCEPTIONS 0) 
set(PICO_CXX_ENABLE_RTTI 0)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/pico-arduino-compat/libs/adafruit-ina219)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/dht)

add_executable(${PROJECT_NAME} 
        main.cpp    
        )
target_compile_definitions(${PROJECT_NAME} PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"  
        MQTT_CLIENT_ID=\"${MQTT_CLIENT_ID}\"
        MQTT_CLIENT_PSW=\"${MQTT_CLIENT_PSW}\"   
        )
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wno-error=unused-variable -Werror=unused-but-set-variable -Wno-error=unused-value) # -Werror

# create map/bin/hex/uf2 files.         
pico_add_extra_outputs(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}        
        pico_stdlib        
        pico_cyw43_arch_lwip_threadsafe_background 
        hardware_adc 
        hardware_i2c
        pac-adafruit-ina219
        pico_lwip_mqtt
        dht
        hardware_rtc
        pico_runtime 
        hardware_sleep       
        )

#Print memory usage
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
COMMAND arm-none-eabi-size ${CMAKE_CURRENT_LIST_DIR}/build/${PROJECT_NAME}.elf
VERBATIM
)        
string(APPEND CMAKE_EXE_LINKER_FLAGS "-Wl,--print-memory-usage")        

#pico_set_binary_type(${PROJECT_NAME} no_flash)         