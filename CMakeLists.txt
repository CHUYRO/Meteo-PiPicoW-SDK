cmake_minimum_required(VERSION 3.20)

set(NAME main) #Name your project/executable here!

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

project(pico_w_Meteo VERSION 1.0 DESCRIPTION "PicoW_Meteo" LANGUAGES C CXX)
set(PICO_BOARD pico_w)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_CXX_ENABLE_RTTI 0)
set(PICO_CXX_ENABLE_EXCEPTIONS 0)
set(CMAKE_CXX_EXTENSIONS 0)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.5.1")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.5.1 (or later) required!. 
    Your version is ${PICO_SDK_VERSION_STRING}")
endif()

pico_sdk_init()

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/cmake/credentials.cmake)
        include(${CMAKE_CURRENT_LIST_DIR}/cmake/credentials.cmake)
else()
        message(WARNING "Credentials file not found!")
endif()

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/include/ina219)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/include/dht22)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/include/bmp280)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/include/wifiLib)

add_executable(${NAME} 
        src/${NAME}.cpp  
        )

target_compile_definitions(${NAME} PRIVATE
        WIFI_SSID1=\"${WIFI_SSID1}\"
        WIFI_PASSWORD1=\"${WIFI_PASSWORD1}\"
        WIFI_SSID2=\"${WIFI_SSID2}\"
        WIFI_PASSWORD2=\"${WIFI_PASSWORD2}\"  
        MQTT_CLIENT_ID=\"${MQTT_CLIENT_ID}\"
        MQTT_CLIENT_PSW=\"${MQTT_CLIENT_PSW}\"  
        PICOW_HOSTNAME=\"${PICOW_HOSTNAME}\" 
        #NDEBUG=0  # LWIP debug logic disabled
        )

target_compile_options(${NAME} PRIVATE -Wall -Wno-unused-function -Wno-format -Wno-error=unused-variable  -Wno-maybe-uninitialized -Werror=unused-but-set-variable -Wno-error=unused-value -Wno-unknown-pragmas) 

target_include_directories(${NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include/lwip
        )

target_link_libraries(${NAME}        
        pico_stdlib        
        pico_cyw43_arch_lwip_threadsafe_background
        #pico_cyw43_arch_lwip_poll
        pico_lwip_mqtt
        hardware_adc 
        hardware_sleep 
        ina219      
        dht22
        bmp280  
        wifiLib
        )

# Print memory usage
add_custom_command(TARGET ${NAME} POST_BUILD
COMMAND arm-none-eabi-size ${CMAKE_CURRENT_LIST_DIR}/build/${NAME}.elf
VERBATIM)        
string(APPEND CMAKE_EXE_LINKER_FLAGS "-Wl,--print-memory-usage")   

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)   

# create map/bin/hex/uf2 files.         
pico_add_extra_outputs(${NAME})