idf_component_register(
    SRCS "point_stream.c"
    INCLUDE_DIRS "include"
    REQUIRES
        point_ring          # laser_point_t (public: in point_stream.h)
    PRIV_REQUIRES
        lwip                # BSD sockets for the TCP receiver task
)
