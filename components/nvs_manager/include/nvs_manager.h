/*
 * nvs_manager.h
 *
 *  Created on: Apr 3, 2022
 *      Author: samuel ramrajkar
 */

#ifndef COMPONENTS_NVS_MANAGER_INCLUDE_NVS_MANAGER_H_
#define COMPONENTS_NVS_MANAGER_INCLUDE_NVS_MANAGER_H_
#include "esp_err.h"
#include "nvs_flash.h"

esp_err_t seed_nvs(char* json);
esp_err_t init_seed_nvs(void);
esp_err_t nvs_manager_set_float(const char *name_space, const char *key_name, float* val);
esp_err_t nvs_manager_get_float(const char *name_space, const char *key_name, float* val);
char* nvs_manager_get_str(const char *name_space, const char *key_name);
esp_err_t nvs_manager_set_str(const char *name_space, const char *key_name, const char *val);
uint32_t nvs_manager_get_uint32_t(const char *name_space, const char *key_name);
esp_err_t nvs_manager_set_uint32_t(const char *name_space, const char *key_name, uint32_t val);
uint16_t nvs_manager_get_uint16_t(const char *name_space, const char *key_name);
esp_err_t nvs_manager_set_uint16_t(const char *name_space, const char *key_name, uint16_t val);
uint8_t nvs_manager_get_uint8_t(const char *name_space, const char *key_name);
esp_err_t nvs_manager_set_uint8_t(const char *name_space, const char *key_name, uint8_t val);

#endif /* COMPONENTS_NVS_MANAGER_INCLUDE_NVS_MANAGER_H_ */
