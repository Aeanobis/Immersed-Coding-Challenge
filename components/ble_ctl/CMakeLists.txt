idf_component_register(
    SRCS
        "ble_ctl.c"
        "ble_mb_gw.c"   
    INCLUDE_DIRS
        "include"
    REQUIRES
        log bt driver esp_hw_support nvs_manager
)
