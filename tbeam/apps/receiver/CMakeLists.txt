cmake_minimum_required(VERSION 3.16)

# The USB-tethered ground-station receiver, node_id 0 (ADR 0027/0028). Primary
# target: esp32s3 (T-Beam S3 Supreme); also builds for esp32 so a classic can be
# a spare receiver (tether over its UART bridge) — the 2026-09-03 swap test. Continuous-RX radio-to-USB bridge —
# no TDMA, no sensor pipeline this phase; the GNSS UART is opened for position
# only (the tether's FIX line, components/gps_time); see
# tbeam/README.md for why this is a separate app from the satellite
# rather than a role flag in a shared main.c.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components"
    "${CMAKE_CURRENT_LIST_DIR}/../../third_party/RadioLib"
    "${CMAKE_CURRENT_LIST_DIR}/../../third_party/XPowersLib"
)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/band_guard.cmake)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(swarm_receiver)
