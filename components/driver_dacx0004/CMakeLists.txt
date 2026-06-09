set(DRIVER_DIR "${CMAKE_SOURCE_DIR}/third-party/github.com/oclyke/driver-DACx0004")

idf_component_register(
    SRCS
        "${DRIVER_DIR}/dacx0004.c"
        "dacx0004_idf6.c"
    INCLUDE_DIRS
        "${DRIVER_DIR}"
        "include"
    REQUIRES
        esp_driver_spi
        esp_driver_gpio
)

# dacx0004.c uses intentional fall-through in a switch; suppress for third-party source
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-implicit-fallthrough")
