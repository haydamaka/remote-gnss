set(srcs "main.c" "wifi.c" "webserver.c" "mdns_service.c" "gnss.c" "magnetometer.c" "mag_cal.c" "mag_fit.c" "ota.c"
         "nmea_hub.c" "nmea_tcp.c" "nmea_ble.c" "bearing.c" "bearing_cal.c" "wmm.c" "led.c" "waypoint.c" "radio_power.c")

# One receiver profile, per menuconfig "Remote GNSS -> GNSS -> Receiver". Both
# define gnss_receiver, so only the chosen one may be compiled in.
if(CONFIG_RGNSS_GNSS_F9P)
    list(APPEND srcs "gnss_f9p.c")
else()
    list(APPEND srcs "gnss_ws181.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES app_update bt esp_coex esp_driver_gpio esp_driver_i2c esp_driver_uart esp_event esp_http_server esp_netif
                                     esp_phy esp_timer esp_wifi lwip minmea nvs_flash
                       EMBED_FILES "../html/index.html")

# magnetometer.c needs to know when the WiFi driver powers the radio up and down.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_phy_enable" "-Wl,--wrap=esp_phy_disable")
