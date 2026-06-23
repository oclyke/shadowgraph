idf_component_register(
    SRCS "artnet_control.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES
        lwip                # BSD sockets for the Art-Net UDP receiver task
        esp_netif           # self IP lookup for ArtPollReply (discovery)
)
