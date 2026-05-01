idf_component_register(SRCS "comlink_crsf.c"
                       INCLUDE_DIRS "include"
                       REQUIRES comlink_protocol esp_driver_uart esp_driver_gpio)
