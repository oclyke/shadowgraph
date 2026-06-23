idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver_dac8871 driver_dacx0004 laser_engine point_stream wifi_ap esp_driver_spi esp_driver_gptimer log
)
