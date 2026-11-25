idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES radio board pmu gps_time display
    PRIV_REQUIRES esp_timer
)
