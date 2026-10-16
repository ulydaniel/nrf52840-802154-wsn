cmake_minimum_required(VERSION 3.20.0)

set(BOARD OzturkLab/nrf52840)

# Added explicitly rather than as an auto-detected app.overlay:
# auto-detection is skipped once a build folder has cached an
# empty DTC_OVERLAY_FILE, so existing build folders would miss it.
list(APPEND EXTRA_DTC_OVERLAY_FILE ${CMAKE_CURRENT_LIST_DIR}/battery_adc.overlay)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nrf52840_blinky)

target_sources(app PRIVATE src/main.c)