idf_component_register(
    SRCS "park_node_config.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash spiffs json driver
)