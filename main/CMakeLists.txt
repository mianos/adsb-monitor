idf_component_register(SRCS "main.cpp" "Display.cpp" "Feed.cpp" "Ui.cpp" "WebApi.cpp" "LogBuffer.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES
                         nvsstoragemanager
                         wifimanager
                         webserver
                         jsonwrapper
                         settingsbase
                         app_update
                         esp_http_client
                         mbedtls
                         esp_lcd
                         esp_driver_spi
                         esp_driver_i2c
                         esp_driver_ledc
                         esp_driver_gpio
                         esp_timer
                         esp_wifi
                         esp_netif
                         esp_event
                         nvs_flash)
