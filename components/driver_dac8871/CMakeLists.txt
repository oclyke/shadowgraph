set(DRIVER_DIR "${CMAKE_SOURCE_DIR}/third-party/github.com/oclyke/driver-DAC8871")

idf_component_register(
    SRCS
        "${DRIVER_DIR}/dac8871.c"
        "dac8871_idf6.c"
    INCLUDE_DIRS
        "${DRIVER_DIR}"
        "include"
    REQUIRES
        esp_driver_spi
        esp_driver_gpio
)
