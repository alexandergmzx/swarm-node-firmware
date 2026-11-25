cmake_minimum_required(VERSION 3.16)

# M1 bring-up diagnostic (ADR 0027/0028) — esp32 and esp32s3, one-off tool, NOT
# part of the shipped satellite/receiver pair. Confirms per-board radio
# identity, PMU identity, I2C wiring, and NMEA/PPS presence before the real
# satellite firmware trusts board_pins.h's candidate values.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components"
    "${CMAKE_CURRENT_LIST_DIR}/../../third_party/RadioLib"
    "${CMAKE_CURRENT_LIST_DIR}/../../third_party/XPowersLib"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(swarm_board_probe)
