idf_component_register(
    SRCS "ir_sensor.c" "ir_task.c"
    INCLUDE_DIRS "."
    REQUIRES driver mqtt config servo_controller
)