#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_log.h"
#include "ble_ctl.h"
#include "nvs_manager.h"

#define TAG "ble_mb_gw"

void write_property(ble_uuid_t* uuid, uint8_t* data, uint16_t data_len)
{
    bool current_bool_val = (*data != 0) ? true : false;
    if(0 == ble_uuid_cmp(uuid, CONTROL_CHAR_UUID128(BLE_CHAR_BUTTON_UP_UUID16, BLE_SVC_CONTROL_UUID16)))
    {
       // set_coil(MB_COILS_BLE_UP, current_bool_val);
    }
    else if(0 == ble_uuid_cmp(uuid, CONTROL_CHAR_UUID128(BLE_CHAR_BUTTON_DOWN_UUID16, BLE_SVC_CONTROL_UUID16)))
    {
       // set_coil(MB_COILS_BLE_DWN, current_bool_val);
    }
    // ... rest of your existing write_property implementation
    else if(0 == ble_uuid_cmp(uuid, CONTROL_CHAR_UUID128(BLE_CHAR_DEVICE_NAME_UUID16, BLE_SVC_CONTROL_UUID16)))
    {
        nvs_manager_set_str(DEVICENAME_NAME_SPACE, DEVICENAME_KEY, (const char*)data);
    }
    else
    {
        ESP_LOGW(TAG, "Trying to write to unknown characteristics");
    }
}

void read_property(ble_uuid_t* uuid, uint8_t* data, uint16_t* data_len)
{
    if(0 == ble_uuid_cmp(uuid, CONTROL_CHAR_UUID128(BLE_CHAR_DEVICE_NAME_UUID16, BLE_SVC_CONTROL_UUID16)))
    {
        char* name_from_storage = nvs_manager_get_str(DEVICENAME_NAME_SPACE, DEVICENAME_KEY);
        if(NULL != name_from_storage)
        {
            memset(data, 0U, *data_len);
            strncpy((char*)data, name_from_storage, *data_len - 1U);
            uint16_t _len = strlen(name_from_storage);
            *data_len = (_len < (*data_len - 1U))? _len :  (*data_len - 1U);
            free(name_from_storage);
        }
        else
        {
            *data_len = 0U;
        }
    }
    // ... rest of your existing read_property implementation
    else
    {
        ESP_LOGW(TAG, "Trying to read unknown characteristics");
    }
}