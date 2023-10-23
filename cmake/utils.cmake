function(get_pico_sdk_import_cmake)
        set(PICO_SDK_IMPORT_CMAKE ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/pico_sdk_import.cmake)

        if (NOT EXISTS ${PICO_SDK_IMPORT_CMAKE})
                file(DOWNLOAD
                        https://raw.githubusercontent.com/raspberrypi/pico-sdk/2e6142b15b8a75c1227dd3edbe839193b2bf9041/external/pico_sdk_import.cmake
                        ${PICO_SDK_IMPORT_CMAKE}
                        )
        endif()
endfunction()
