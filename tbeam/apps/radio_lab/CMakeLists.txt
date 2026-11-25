cmake_minimum_required(VERSION 3.16)

# Field radio test bench for the T-Beam S3 Supreme (esp32s3 only): ping-pong
# range tests between two boards with GPS distance, a listen mode for other
# LoRa radios, a noise sweep across the band and tagged bursts — every
# transmission held under the build profile's airtime ceiling. Not a node app.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components"
    "${CMAKE_CURRENT_LIST_DIR}/../../third_party/RadioLib"
    "${CMAKE_CURRENT_LIST_DIR}/../../third_party/XPowersLib"
)

include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/band_guard.cmake)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(swarm_radio_lab)
