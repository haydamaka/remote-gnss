idf_component_register(SRCS "main.c" "wifi.c" "webserver.c" "mdns_service.c" "gnss.c" "magnetometer.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_uart esp_event esp_http_server esp_netif
                                     esp_timer esp_wifi minmea nvs_flash
                       EMBED_FILES "../html/index.html")
