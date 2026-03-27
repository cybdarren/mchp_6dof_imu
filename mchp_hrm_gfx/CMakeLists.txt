cmake_minimum_required(VERSION 3.20.0)

# ------------------------------------------------------------
# Board-specific DTS overlays (MUST be set before Zephyr)
# ------------------------------------------------------------
if(DEFINED BOARD)
  if(BOARD STREQUAL "same54_xpro")
    list(APPEND DTC_OVERLAY_FILE
      ${CMAKE_CURRENT_SOURCE_DIR}/boards/overlays/same54_xpro.overlay
    )
  elseif(BOARD STREQUAL "samd21_iot_wg")
    list(APPEND DTC_OVERLAY_FILE
      ${CMAKE_CURRENT_SOURCE_DIR}/boards/overlays/samd21_iot_wg.overlay
    )
  elseif(BOARD STREQUAL "samd21_iot_wg2")
    list(APPEND DTC_OVERLAY_FILE
      ${CMAKE_CURRENT_SOURCE_DIR}/boards/overlays/samd21_iot_wg2.overlay
    )
    elseif(BOARD STREQUAL "samd21_xpro_ot")
    list(APPEND DTC_OVERLAY_FILE
      ${CMAKE_CURRENT_SOURCE_DIR}/boards/overlays/samd21_xpro_ot.overlay
    )
  endif()

endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(mchp_hrm_gfx)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_sources(app PRIVATE
    src/main.c
    src/hrm_text.c
    src/hrm_chart.c
    src/afe4404.c
)
