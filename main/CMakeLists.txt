idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver_dac8871 driver_dacx0004 esp_driver_spi log
)
