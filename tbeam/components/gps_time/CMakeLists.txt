# NMEA $--RMC time/position estimate with an optional PPS tightening path
# (gps_time.h; ADR 0016 / ADR 0027). Shared by both apps: the satellite
# disciplines its TDMA slot clock from it, the receiver only reads the
# position it prints on the tether as a FIX line (protocols/specs/messages.md).
# Public dependency: board (SWARM_PIN_GPS_* and swarm_board_has_pps()); the
# UART/GPIO drivers and esp_timer are implementation details.
idf_component_register(
    SRCS "gps_time.c"
    INCLUDE_DIRS "include"
    REQUIRES board
    PRIV_REQUIRES driver esp_timer
)
