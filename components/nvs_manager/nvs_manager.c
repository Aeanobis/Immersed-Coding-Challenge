#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cJSON.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "nvs_manager.h"

#define TAG		"nvs_mgr"

extern const uint8_t nvs_seed_start[] asm("_binary_config_json_start");

esp_err_t nvs_manager_set_float(const char *name_space, const char *key_name, float *val)
{
	nvs_handle_t primary_handle;
	char data_primary_str[16];

	memset(data_primary_str, 0u, sizeof(data_primary_str));
	snprintf(data_primary_str, sizeof(data_primary_str), "%.8f", *val);

	ESP_ERROR_CHECK(nvs_open(name_space, NVS_READWRITE, &primary_handle));
	ESP_ERROR_CHECK(nvs_set_str(primary_handle, key_name, data_primary_str));
	ESP_ERROR_CHECK(nvs_commit(primary_handle));
	nvs_close(primary_handle);
	return ESP_OK;
}

esp_err_t nvs_manager_set_str(const char *name_space, const char *key_name, const char *val)
{
    nvs_handle_t primary_handle;
    ESP_ERROR_CHECK(nvs_open(name_space, NVS_READWRITE, &primary_handle));
    ESP_ERROR_CHECK(nvs_set_str(primary_handle, key_name, val));
    ESP_ERROR_CHECK(nvs_commit(primary_handle));
    nvs_close(primary_handle);
    return ESP_OK;
}

esp_err_t nvs_manager_set_uint32_t(const char *name_space, const char *key_name, uint32_t val)
{
    nvs_handle_t primary_handle;
    ESP_ERROR_CHECK(nvs_open(name_space, NVS_READWRITE, &primary_handle));
    ESP_ERROR_CHECK(nvs_set_u32(primary_handle, key_name, val));
    ESP_ERROR_CHECK(nvs_commit(primary_handle));
    nvs_close(primary_handle);
    return ESP_OK;
}

esp_err_t nvs_manager_set_uint16_t(const char *name_space, const char *key_name, uint16_t val)
{
    nvs_handle_t primary_handle;
    ESP_ERROR_CHECK(nvs_open(name_space, NVS_READWRITE, &primary_handle));
    ESP_ERROR_CHECK(nvs_set_u16(primary_handle, key_name, val));
    ESP_ERROR_CHECK(nvs_commit(primary_handle));
    nvs_close(primary_handle);
    return ESP_OK;
}

esp_err_t nvs_manager_set_uint8_t(const char *name_space, const char *key_name, uint8_t val)
{
    nvs_handle_t primary_handle;
    ESP_ERROR_CHECK(nvs_open(name_space, NVS_READWRITE, &primary_handle));
    ESP_ERROR_CHECK(nvs_set_u8(primary_handle, key_name, val));
    ESP_ERROR_CHECK(nvs_commit(primary_handle));
    nvs_close(primary_handle);
    return ESP_OK;
}

esp_err_t nvs_manager_get_float(const char *name_space, const char *key_name, float* val)
{
	nvs_handle_t primary_handle;
	char data_primary_str[16];
	float primary_val = 0.0f;
	size_t strlen = sizeof(data_primary_str);
	memset(data_primary_str, 0u, sizeof(data_primary_str));
	ESP_ERROR_CHECK(nvs_open(name_space, NVS_READONLY, &primary_handle));
	ESP_ERROR_CHECK(nvs_get_str(primary_handle, key_name, data_primary_str, &strlen));
	nvs_close(primary_handle);
	primary_val = atof(data_primary_str);
	*val = primary_val;
	return ESP_OK;
}

char* nvs_manager_get_str(const char *name_space, const char *key_name)
{
	// Initialize NVS
   	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	// Retrieve key value from namespace
	nvs_handle_t primary_handle;
	size_t size_required;
	char* primary_val = NULL;
	ESP_ERROR_CHECK(nvs_open(name_space, NVS_READONLY, &primary_handle));
	ESP_ERROR_CHECK(nvs_get_str(primary_handle, key_name, NULL, &size_required));
	primary_val = (char*)malloc(size_required + 1);
	ESP_ERROR_CHECK(nvs_get_str(primary_handle, key_name, primary_val, &size_required));
	nvs_close(primary_handle);
	return primary_val;
}

uint32_t nvs_manager_get_uint32_t(const char *name_space, const char *key_name) 
{
	// Initialize NVS
   	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	// Retrieve key value from namespace
	nvs_handle_t primary_handle;
	uint32_t primary_val = 0;
	ESP_ERROR_CHECK(nvs_open(name_space, NVS_READONLY, &primary_handle));
	ESP_ERROR_CHECK(nvs_get_u32(primary_handle, key_name, &primary_val));
	nvs_close(primary_handle);
	return primary_val;
}

uint16_t nvs_manager_get_uint16_t(const char *name_space, const char *key_name) 
{
	// Initialize NVS
   	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	// Retrieve key value from namespace
	nvs_handle_t primary_handle;
	uint16_t primary_val = 0;
	ESP_ERROR_CHECK(nvs_open(name_space, NVS_READONLY, &primary_handle));
	ESP_ERROR_CHECK(nvs_get_u16(primary_handle, key_name, &primary_val));
	nvs_close(primary_handle);
	return primary_val;
}

uint8_t nvs_manager_get_uint8_t(const char *name_space, const char *key_name) 
{
	// Initialize NVS
   	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	  ESP_ERROR_CHECK(nvs_flash_erase());
	  ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	// Retrieve key value from namespace
	nvs_handle_t primary_handle;
	uint8_t primary_val = 0;
	ESP_ERROR_CHECK(nvs_open(name_space, NVS_READONLY, &primary_handle));
	ESP_ERROR_CHECK(nvs_get_u8(primary_handle, key_name, &primary_val));
	nvs_close(primary_handle);
	return primary_val;
}

esp_err_t seed_nvs(char* json)
{
	esp_err_t rc = ESP_FAIL;
	rc = nvs_flash_init();
	if (rc == ESP_ERR_NVS_NO_FREE_PAGES || rc == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		rc = nvs_flash_init();
	}
	ESP_ERROR_CHECK(rc);
	cJSON *parsed = cJSON_Parse(json);
	if (NULL != parsed)
	{
		cJSON *namespace_itr = parsed->child;
		while (namespace_itr)
		{
			ESP_LOGI(TAG, "%s", namespace_itr->string);
			nvs_handle_t nvs_handle;
			ESP_ERROR_CHECK(nvs_open(namespace_itr->string, NVS_READWRITE, &nvs_handle));
			cJSON *key_itr = namespace_itr->child;
			while(NULL != key_itr)
			{
				ESP_LOGI(TAG, "	%s", key_itr->string);
				cJSON *type_json = cJSON_GetObjectItem(key_itr, "type");
				cJSON *value_json = cJSON_GetObjectItem(key_itr, "value");
				if ((NULL != type_json) && (NULL != value_json))
				{
					if (strcmp(type_json->valuestring, "string") == 0)
					{
						size_t dummy_size = 0u;
						if(ESP_OK != nvs_get_str(nvs_handle, key_itr->string, NULL, &dummy_size))
						{
							ESP_ERROR_CHECK(nvs_set_str(nvs_handle, key_itr->string, value_json->valuestring));
							ESP_ERROR_CHECK(nvs_commit(nvs_handle));
							ESP_LOGI(TAG, "Written");
						}
						else
						{
							ESP_LOGI(TAG, "NVS key %s found...Skipping", key_itr->string);
						}
					}
					else if (strcmp(type_json->valuestring, "u8") == 0)
					{
						uint8_t dummy_val = 0u;
						if(ESP_OK != nvs_get_u8(nvs_handle, key_itr->string, &dummy_val))
						{
							ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, key_itr->string, value_json->valueint));
							ESP_ERROR_CHECK(nvs_commit(nvs_handle));
						}
					}
					else if (strcmp(type_json->valuestring, "i8") == 0)
					{
						int8_t dummy_val = 0;
						if(ESP_OK != nvs_get_i8(nvs_handle, key_itr->string, &dummy_val))
						{
							ESP_ERROR_CHECK(nvs_set_i8(nvs_handle, key_itr->string, value_json->valueint));
							ESP_ERROR_CHECK(nvs_commit(nvs_handle));
						}
					}
					else if (strcmp(type_json->valuestring, "u16") == 0)
					{
						uint16_t dummy_val = 0u;
						if(ESP_OK != nvs_get_u16(nvs_handle, key_itr->string, &dummy_val))
						{
							ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, key_itr->string, value_json->valueint));
							ESP_ERROR_CHECK(nvs_commit(nvs_handle));
						}
					}
					else if (strcmp(type_json->valuestring, "i16") == 0)
					{
						int16_t dummy_val = 0;
						if(ESP_OK != nvs_get_i16(nvs_handle, key_itr->string, &dummy_val))
						{
							ESP_ERROR_CHECK(nvs_set_i16(nvs_handle, key_itr->string, value_json->valueint));
							ESP_ERROR_CHECK(nvs_commit(nvs_handle));
						}
					}
					else if (strcmp(type_json->valuestring, "u32") == 0)
					{
						uint32_t dummy_val = 0u;
						if(ESP_OK != nvs_get_u32(nvs_handle, key_itr->string, &dummy_val))
						{
							ESP_ERROR_CHECK(nvs_set_u32(nvs_handle, key_itr->string, value_json->valueint));
							ESP_ERROR_CHECK(nvs_commit(nvs_handle));
						}
					}
					else if (strcmp(type_json->valuestring, "i32") == 0)
					{
						int32_t dummy_val = 0;
						if(ESP_OK != nvs_get_i32(nvs_handle, key_itr->string, &dummy_val))
						{
							ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, key_itr->string, value_json->valueint));
							ESP_ERROR_CHECK(nvs_commit(nvs_handle));
						}
					}
					else if (strcmp(type_json->valuestring, "u64") == 0)
					{
						uint64_t dummy_val = 0u;
						if(ESP_OK != nvs_get_u64(nvs_handle, key_itr->string, &dummy_val))
						{
							ESP_ERROR_CHECK(nvs_set_u64(nvs_handle, key_itr->string, value_json->valueint));
							ESP_ERROR_CHECK(nvs_commit(nvs_handle));
						}
					}
					else if (strcmp(type_json->valuestring, "i64") == 0)
					{
						int64_t dummy_val = 0;
						if(ESP_OK != nvs_get_i64(nvs_handle, key_itr->string, &dummy_val))
						{
							ESP_ERROR_CHECK(nvs_set_i64(nvs_handle, key_itr->string, value_json->valueint));
							ESP_ERROR_CHECK(nvs_commit(nvs_handle));
						}
					}
					else
					{
						ESP_LOGE(TAG, "can't set unknown NVS type %s", type_json->valuestring);
					}
				}
				else
				{
					ESP_LOGE(TAG, "JSON object type not supported");
				}
				key_itr = key_itr->next;
			}
			#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
			nvs_close(nvs_handle);
			#else
			nvs_close(&nvs_handle);
			#endif
			namespace_itr = namespace_itr->next;
		}
		cJSON_Delete(parsed);
		rc = ESP_OK;
	}
	return rc;
}

esp_err_t init_seed_nvs(void)
{
	return seed_nvs(nvs_seed_start);
}
