idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_netif esp_wifi nvs_flash esp_driver_uart ir_sensor ultrasonic_sensor config spiffs servo_controller
)
