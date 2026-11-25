# The shared wire-format headers are included, never copied (CLAUDE.md). This
# component exists only to put firmware/shared/ on the include path, so code
# says #include "protocol/sensor_bundle.h" and firmware/shared/ keeps its rule
# of carrying no build system of its own.
idf_component_register(INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../shared")
