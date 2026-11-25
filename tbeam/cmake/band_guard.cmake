# Band guard, included by every app before ESP-IDF's project.cmake.
#
# The radio band is a Kconfig choice set by an overlay in SDKCONFIG_DEFAULTS
# (../../profiles/band-433.defaults or band-915.defaults). ESP-IDF applies
# defaults only to values an existing sdkconfig does not already hold, so a
# reused build directory would silently keep the band it was first configured
# with. This guard compares the overlay passed now with the band recorded in
# the sdkconfig and stops before anything is built when they differ.

set(_swarm_defaults "${SDKCONFIG_DEFAULTS}")
if(NOT _swarm_defaults AND DEFINED ENV{SDKCONFIG_DEFAULTS})
    set(_swarm_defaults "$ENV{SDKCONFIG_DEFAULTS}")
endif()
set(_swarm_sdkconfig "${SDKCONFIG}")
if(NOT _swarm_sdkconfig)
    set(_swarm_sdkconfig "${CMAKE_SOURCE_DIR}/sdkconfig")
endif()

set(_swarm_wanted "")
if(_swarm_defaults MATCHES "band-915\\.defaults")
    set(_swarm_wanted "915")
elseif(_swarm_defaults MATCHES "band-433\\.defaults")
    set(_swarm_wanted "433")
endif()

if(_swarm_wanted AND EXISTS "${_swarm_sdkconfig}")
    file(STRINGS "${_swarm_sdkconfig}" _swarm_lines REGEX "^CONFIG_SWARM_RADIO_PROFILE_(433|915|UNSET)=y$")
    if(_swarm_lines)
        string(REGEX REPLACE "^CONFIG_SWARM_RADIO_PROFILE_([A-Z0-9]+)=y$" "\\1" _swarm_have "${_swarm_lines}")
        if(NOT _swarm_have STREQUAL _swarm_wanted)
            message(FATAL_ERROR
                "Band mismatch: SDKCONFIG_DEFAULTS names band-${_swarm_wanted}.defaults, but\n"
                "  ${_swarm_sdkconfig}\n"
                "already records radio profile ${_swarm_have}, and an existing sdkconfig wins over defaults.\n"
                "Delete that sdkconfig (or use a build directory named for the band, e.g. build-node4-915)\n"
                "and run the same command again.")
        endif()
    endif()
endif()
