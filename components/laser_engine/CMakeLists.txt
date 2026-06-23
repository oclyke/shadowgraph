idf_component_register(
    SRCS
        "laser_engine.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        point_ring          # laser_point_t + SPSC ring (public: in laser_engine.h)
        isr_spi             # ISR-safe SPI writer (public: in laser_engine.h)
        driver_dacx0004     # DACX0004 command/channel enums used by the encoder
        esp_driver_gptimer
)
