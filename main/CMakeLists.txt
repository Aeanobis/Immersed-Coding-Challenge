idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES bt ble_ctl nvs_flash
    PRIV_REQUIRES fs nvs_manager
)
