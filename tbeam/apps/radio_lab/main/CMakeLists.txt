idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES radio board pmu gps_time display
    PRIV_REQUIRES driver esp_timer esp_driver_usb_serial_jtag esp_driver_gpio
)
