idf_component_register(
    SRCS
        "isr_spi.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_driver_spi   # spi_host_device_t, bus types
        esp_hal_gpspi    # spi_hal.h / spi_ll.h, IRAM transaction functions
        hal
)
