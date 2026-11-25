idf_component_register(
    SRCS "app_main.c"
         "tdma.c"
         "frame.c"
         "shared_state.c"
         "i2c_bus.c"
         "sensors/pms7003.c"
         "sensors/scd30.c"
         "sensors/bme688.c"
    INCLUDE_DIRS "." "sensors"
    REQUIRES radio board pmu gps_time swarm_proto bme68x
    PRIV_REQUIRES driver esp_timer
)
