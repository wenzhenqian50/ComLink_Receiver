idf_component_register(SRCS "comlink_now_receiver.c"
                       INCLUDE_DIRS "include"
                       REQUIRES comlink_protocol esp_wifi esp_event esp_netif)
