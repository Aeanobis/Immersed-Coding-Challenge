idf_component_register(SRCS "fs.c"
                    INCLUDE_DIRS "include"
                    REQUIRES log spi_flash esp_partition esp_system log driver littlefs fs)
