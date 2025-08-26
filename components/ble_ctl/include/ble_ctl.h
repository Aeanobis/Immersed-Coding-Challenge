/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef H_BLESPPSERVER_
#define H_BLESPPSERVER_

#include <stdbool.h>
#include "nimble/ble.h"
#include "host/ble_uuid.h"
#include "modlog/modlog.h"
#ifdef __cplusplus
extern "C" {
#endif

// --- Characteristic handle declarations ---
static uint16_t learn_mode_handle;
static uint16_t button_up_handle;
static uint16_t button_down_handle;
static uint16_t button_stop_handle;
static uint16_t button_aux_handle;
static uint16_t auto_mode_handle;
static uint16_t device_name_handle;
static uint16_t motor_status_handle;
static uint16_t limits_reached_handle;
static uint16_t device_status_handle;
static uint16_t status_reason_handle;
// Phase 2 service handles:
static uint16_t wifi_ssid_handle;
static uint16_t wifi_password_handle;
static uint16_t error_logs_trigger_handle;
static uint16_t error_logs_data_handle;
static uint16_t leveling_button_handle;

//NVS defines
#define BASE_NAME_SPACE                 "config"
#define DEVICENAME_NAME_SPACE           BASE_NAME_SPACE
#define DEVICENAME_KEY                  "name"

// 16-bit UUID definitions (16-bit portion of 128-bit UUID)
#define BLE_SVC_LEARNING_UUID16       0x09B4  /* replace with actual learn service ID */
#define BLE_CHAR_LEARN_MODE_UUID16    0x9966  /* replace with actual learn-mode char ID */

#define BLE_SVC_CONTROL_UUID16        0x53d7
#define BLE_CHAR_BUTTON_UP_UUID16     0x1121
#define BLE_CHAR_BUTTON_DOWN_UUID16   0x1122
#define BLE_CHAR_BUTTON_STOP_UUID16   0x1123
#define BLE_CHAR_BUTTON_AUX_UUID16    0x1124
#define BLE_CHAR_LEARN_MODE_CTRL_UUID16 0x1223
#define BLE_CHAR_AUTO_MODE_UUID16     0x1225
#define BLE_CHAR_DEVICE_NAME_UUID16   0x1224
#define BLE_CHAR_MOTOR_STATUS_UUID16  0x1322
#define BLE_CHAR_LIMITS_REACHED_UUID16 0x1321
#define BLE_CHAR_DEVICE_STATUS_UUID16 0x1323
#define BLE_CHAR_STATUS_REASON_UUID16 0x1324

#define BLE_SVC_EXPANDED_UUID16       0xC141
#define BLE_CHAR_WIFI_SSID_UUID16     0x0001
#define BLE_CHAR_WIFI_PWD_UUID16      0x0002
#define BLE_CHAR_ERROR_TRIGGER_UUID16 0x0003
#define BLE_CHAR_ERROR_DATA_UUID16    0x0004
#define BLE_CHAR_LEVELING_BTN_UUID16  0x0005

// Full 128-bit UUID services
#define BLE_SVC_CUSTOM_UUID128 BLE_UUID128_DECLARE( \
    0xB4, 0x09, 0x8D, 0x86, 0x61, 0xBA, 0x61, 0x4B, \
    0x3C, 0xF1, 0x5A, 0xAD, 0x66, 0x99, 0x00, 0x00)

// Empty service - 9966ad5a-f13c-4b61-ba66-0861e08d09b4
#define BLE_SVC_EMPTY_SERVICE_UUID128 BLE_UUID128_DECLARE( \
    0xB4, 0x09, 0x8D, 0xE0, 0x61, 0x08, 0x66, 0xBA, \
    0x61, 0x4B, 0x3C, 0xF1, 0x5A, 0xAD, 0x66, 0x99)

// Macros to expand 16-bit IDs into 128-bit UUIDs using BLE_UUID128_DECLARE
// Format: little-endian: [uuid16_lo, uuid16_hi, svc16_lo, svc16_hi, ...tail bytes...]
#define SERVICE_UUID128(svc16) BLE_UUID128_DECLARE(             \
    (uint8_t)((svc16) & 0xFF), (uint8_t)(((svc16) >> 8) & 0xFF), \
    0x8F, 0x0F, 0x3A, 0xD2, 0x9B, 0x95, 0xA3, 0x45, 0xDA, 0x8D, 0x00, 0x00, 0x40, 0xC1)

#define CHAR_UUID128(uuid16, svc16) BLE_UUID128_DECLARE(       \
    (uint8_t)((uuid16) & 0xFF), (uint8_t)(((uuid16) >> 8) & 0xFF), \
    (uint8_t)((svc16) & 0xFF), (uint8_t)(((svc16) >> 8) & 0xFF), \
    0x8D, 0xDA, 0x45, 0xA3, 0x95, 0x9B, 0xD2, 0x3A, 0x0F, 0x8F, 0x53, 0xD7)

#define CONTROL_CHAR_UUID128(char16, svc16) BLE_UUID128_DECLARE(             \
    (uint8_t)((svc16) & 0xFF), (uint8_t)(((svc16) >> 8) & 0xFF), \
    0x8F, 0x0F, 0x3A, 0xD2, 0x9B, 0x95, 0xA3, 0x45, 0xDA, 0x8D, (uint8_t)(char16 & 0xFF), (uint8_t)(char16 >> 8), 0x40, 0xC1)

enum
{
    BLE_STATUS_DISCONNECTED = 0,
    BLE_STATUS_CONNECTED,
    BLE_STATUS_PAIRING,
    BLE_STATUS_CONNECTION_FAILED
};

struct ble_hs_cfg;
struct ble_gatt_register_ctxt;

void read_property(ble_uuid_t* uuid, uint8_t* data, uint16_t* data_len);
void write_property(ble_uuid_t* uuid, uint8_t* data, uint16_t data_len);
void init_ble(void);
#ifdef __cplusplus
}
#endif

#endif
