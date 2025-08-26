#include <stdio.h>
#include "esp_spi_flash.h"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "soc/spi_pins.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "esp_littlefs.h"
#include "fs.h"

#define TAG                     "fs"
#define PARTITION_LABLE         "littlefs"

void fomart_partition(void)
{
    esp_littlefs_format(PARTITION_LABLE);
}

void init_fs(void)
{
    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/littlefs",
        .partition_label = PARTITION_LABLE,
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    // Use settings defined above to initialize and mount LittleFS filesystem.
    // Note: esp_vfs_littlefs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK)
    {
            if (ret == ESP_FAIL)
            {
                    ESP_LOGE(TAG, "Failed to mount or format filesystem");
            }
            else if (ret == ESP_ERR_NOT_FOUND)
            {
                    ESP_LOGE(TAG, "Failed to find LittleFS partition");
            }
            else
            {
                    ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
            }
            return;
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK)
    {
            ESP_LOGE(TAG, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
            ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
}
