idf_component_register(SRCS "main.cpp" "Pn532I2cTransport.cpp" "RelayProtocol.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES pn532_cxx loggable loggable_espidf esp_wifi esp_driver_i2c nvs_flash esp_timer esp_event esp_netif)
