# Onboard OLED status screen (backlog ADR 0032). Header-only font, one C file;
# CONFIG_SWARM_DISPLAY=n compiles it to no-ops.
idf_component_register(
    SRCS "display.c"
    INCLUDE_DIRS "include"
    REQUIRES driver board
)
