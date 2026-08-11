idf_component_register(SRCS "RS485_CAN.c"
                    INCLUDE_DIRS "."
                    REQUIRES mqtt esp_wifi nvs_flash esp_netif esp_driver_uart esp_driver_gpio esp_driver_spi) 
