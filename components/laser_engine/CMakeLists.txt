idf_component_register(
    SRCS
        "laser_engine.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        byte_queue
        laser_command
        curve_interp        # fixed-point Bézier interpolator (CURVE command)
        isr_spi             # ISR-safe SPI writer (public: in laser_engine.h)
        driver_dacx0004     # DACX0004 command/channel enums used by the encoder
        esp_driver_gptimer
)
