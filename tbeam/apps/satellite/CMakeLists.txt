cmake_minimum_required(VERSION 3.16)

# A TDMA satellite, node_id 1-9 (ADR 0027/0028). Primary target: esp32
# (classic T-Beam). Also builds for esp32s3 so the deferred all-Supreme
# fleet (backlog 0030) stays buildable without a second app.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components"
    "${CMAKE_CURRENT_LIST_DIR}/../../third_party/RadioLib"
    "${CMAKE_CURRENT_LIST_DIR}/../../third_party/XPowersLib"
)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/band_guard.cmake)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(swarm_satellite)
