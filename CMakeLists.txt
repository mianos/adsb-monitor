# adsb-monitor — local flight radar for the Freenove FNK0104S (ESP32-S3, 4.0" ST7796 320x480).
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(adsb-monitor)

# LVGL's pool holds widgets, text and draw tasks, none of it for DMA; in PSRAM it
# leaves 64 KB of internal RAM to Wi-Fi.
idf_component_get_property(lvgl lvgl__lvgl COMPONENT_LIB)
# A compile option, as target_compile_definitions drops function-like macros.
target_compile_options(${lvgl} PRIVATE "-DLV_MEM_POOL_INCLUDE=<esp_heap_caps.h>"
                       "-DLV_MEM_POOL_ALLOC(size)=heap_caps_malloc(size, MALLOC_CAP_SPIRAM)")
