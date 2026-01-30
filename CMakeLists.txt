# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Dies erzwingt die korrekte Architektur für den RP2350
set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350 CACHE STRING "Platform type")

cmake_minimum_required(VERSION 3.13...3.27)

include(pico_sdk_import.cmake)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_TINYUSB_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/tinyusb)
project(pico-usb-audio-loopback-reverb C CXX ASM)
pico_sdk_init()

if (NOT TARGET pico_audio_32b)
    add_library(pico_audio_32b INTERFACE)
    target_include_directories(pico_audio_32b INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/pico_audio_i2s_32b/src/pico_audio_32b/include
    )
endif()

# == DUMMY TARGET FÜR LINKER FEHLER ==
add_library(pico_util_buffer INTERFACE)

# == SUBDIRECTORIES ==
add_subdirectory(lib/pico_audio_i2s_32b/src/pico_audio_32b pico_audio_32b)
add_subdirectory(lib/pico_audio_i2s_32b pico_audio_i2s_32)

add_executable(${CMAKE_PROJECT_NAME}
  src/main.c
  src/usb_descriptors.c
  src/led.c
  src/ringbuffer.c
  src/fx_reverb_rp2040.c
  lib/pico_audio_i2s_32b/src/pico_audio_32b/audio.cpp
#  lib/pico_audio_i2s_32b/src/audio_i2s.c
)

target_include_directories(${PROJECT_NAME} PRIVATE 
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/pico_audio_i2s_32b/src/include      # Ermöglicht #include "pico/audio_i2s.h"
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/pico_audio_i2s_32b/src/pico_audio_32b/include
)

target_link_libraries(${CMAKE_PROJECT_NAME}
  pico_stdlib
  tinyusb_device
  tinyusb_board
  pico_multicore
  hardware_dma
  hardware_pio
  hardware_irq
  pico_audio_32b
  pico_audio_i2s_32b
)

# == PIO GENERIERUNG ==
pico_generate_pio_header(${PROJECT_NAME} 
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/pico_audio_i2s_32b/src/audio_i2s.pio 
)


# Verhindert, dass der Pico versucht, "Double-Tap-Reset" in den Bootloader zu machen,
# was oft USB-Audio-Interrupts stört:
pico_enable_stdio_usb(${PROJECT_NAME} 0) # Deaktiviere USB-Serial, falls du nur Audio willst
pico_enable_stdio_uart(${PROJECT_NAME} 1) # Nutze UART für Debugging

pico_add_extra_outputs(${CMAKE_PROJECT_NAME})
