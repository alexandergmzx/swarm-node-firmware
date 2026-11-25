# Wrapper around Bosch's vendored BME68x_SensorAPI (../../third_party/,
# pinned v4.4.8) — it ships as plain C source with no build system attached,
# so this component just registers it as an ESP-IDF component. Shared under
# components/ (not folded into apps/satellite/main) because any app could
# carry a BME688 — the receiver's kit is fitted but idle this phase (ADR
# 0027), not absent.
idf_component_register(
    SRCS "${CMAKE_CURRENT_LIST_DIR}/../../third_party/BME68x_SensorAPI/bme68x.c"
    INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../third_party/BME68x_SensorAPI"
)
