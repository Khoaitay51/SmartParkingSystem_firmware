idf_component_register(
    SRCS "ir_sensor.c" "ir_task.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio config servo_controller
)
