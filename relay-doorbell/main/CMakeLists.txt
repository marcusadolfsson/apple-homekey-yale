idf_component_register(SRCS "main.cpp" "Pn532I2cTransport.cpp" "RelayProtocol.cpp" "St25r3916Reader.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES pn532_cxx loggable loggable_espidf esp_wifi esp_driver_i2c nvs_flash esp_timer esp_event esp_netif esp_adc esp_driver_gpio esp_driver_usb_serial_jtag esp_hw_support mbedtls)

# Bench test of deep sleep while plugged into a computer:
#   idf.py -B build-sleeptest -DDOORBELL_SLEEP_ON_USB=1 build
# (a separate build directory, so the normal build is untouched).
if(DOORBELL_SLEEP_ON_USB)
  target_compile_definitions(${COMPONENT_LIB} PRIVATE DOORBELL_SLEEP_ON_USB=1)
endif()
# Diagnostic build that reports every N seconds on battery:
#   idf.py -B build-diag -DDOORBELL_HEARTBEAT_SLEEP_S=120 build
if(DOORBELL_HEARTBEAT_SLEEP_S)
  target_compile_definitions(${COMPONENT_LIB} PRIVATE DOORBELL_HEARTBEAT_SLEEP_S=${DOORBELL_HEARTBEAT_SLEEP_S})
endif()
