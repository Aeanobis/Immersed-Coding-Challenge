// battery_monitor.h
#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Battery pack specifications (2S Li-ion)
#define BATTERY_CAPACITY_MAH    5000            // Total capacity in mAh
#define BATTERY_MAX_VOLTAGE     8600            // Maximum safe voltage (mV)
#define BATTERY_MIN_VOLTAGE     6000            // Minimum safe voltage (mV)
#define LOAD_MAX_CURRENT_MA     3000            // Maximum safe current (mA)
#define BATTERY_MAX_TEMP        600             // Maximum safe temperature (°C * 10)
#define BATTERY_MIN_TEMP        0               // Minimum safe temperature (°C * 10)

// Special values
#define BATTERY_SOC_UNKNOWN     255             // Unknown SOC indicator
#define BATTERY_TEMP_UNKNOWN    -32768          // Unknown temperature indicator

// I2C Configuration
#define I2C_MASTER_NUM          I2C_NUM_0       // I2C port number
#define I2C_MASTER_SCL_IO       22              // GPIO22 for SCL
#define I2C_MASTER_SDA_IO       21              // GPIO21 for SDA
#define I2C_MASTER_FREQ_HZ      100000          // I2C frequency

// Device I2C addresses
#define MP2672A_I2C_ADDR        0x09            // MP2672A charger IC address
#define INA219_I2C_ADDR         0x40            // INA219 current sensor address

// Battery status flags (can be OR'd together)
typedef enum {
    BATTERY_STATUS_OK           = 0x00,
    BATTERY_STATUS_LOW          = 0x01,
    BATTERY_STATUS_CRITICAL     = 0x02,
    BATTERY_STATUS_OVERVOLTAGE  = 0x04,
    BATTERY_STATUS_UNDERVOLTAGE = 0x08,
    BATTERY_STATUS_OVERCURRENT  = 0x10,
    BATTERY_STATUS_OVERTEMP     = 0x20,
    BATTERY_STATUS_UNDERTEMP    = 0x40,
    BATTERY_STATUS_FAULT        = 0x80
} battery_status_t;

// Charging status from MP2672A
typedef enum {
    CHARGE_STATUS_NOT_CHARGING = 0,
    CHARGE_STATUS_PRE_CHARGE,
    CHARGE_STATUS_FAST_CHARGE,
    CHARGE_STATUS_CHARGE_COMPLETE
} charge_status_t;

// Battery data structure
typedef struct {
    uint16_t voltage_mv;        // Battery voltage in millivolts
    int16_t  current_ma;        // Battery current in milliamps (+ = discharge, - = charge)
    int16_t  temperature_c10;   // Battery temperature in 0.1°C units
    uint8_t  soc_percent;       // State of charge percentage (0-100)
    uint16_t capacity_mah;      // Remaining capacity in mAh
    uint16_t power_mw;          // Power in milliwatts
    battery_status_t status;    // Current battery status flags
    charge_status_t charge_status; // Charging status from MP2672A
    uint32_t timestamp_ms;      // Timestamp of measurement
} battery_data_t;

// Callback function type for status changes
typedef void (*battery_status_callback_t)(const battery_data_t *data, battery_status_t old_status);

// ============= Core Battery Monitoring Functions =============

/**
 * @brief Initialize battery monitoring system
 * @return ESP_OK on success
 */
esp_err_t battery_monitor_init(void);

/**
 * @brief Deinitialize battery monitoring system
 * @return ESP_OK on success
 */
esp_err_t battery_monitor_deinit(void);

/**
 * @brief Read current battery data
 * @param data Pointer to battery_data_t structure to fill
 * @return ESP_OK on success
 */
esp_err_t battery_monitor_read(battery_data_t *data);

/**
 * @brief Check if battery is in safe operating condition
 * @param data Pointer to battery data
 * @return true if safe, false otherwise
 */
bool battery_monitor_is_safe(const battery_data_t *data);

/**
 * @brief Get LED level indicator (0-4) based on SOC
 * @param data Pointer to battery data
 * @return LED level (0=empty, 4=full)
 */
uint8_t battery_monitor_get_led_level(const battery_data_t *data);

/**
 * @brief Get human-readable battery status string
 * @param status Battery status enum value
 * @return String representation of status
 */
const char* battery_monitor_get_status_string(battery_status_t status);

/**
 * @brief Get human-readable charge status string
 * @param status Charge status enum value
 * @return String representation of charge status
 */
const char* battery_monitor_get_charge_status_string(charge_status_t status);

/**
 * @brief Register callback for status changes
 * @param callback Function to call on status change
 * @return ESP_OK on success
 */
esp_err_t battery_monitor_register_callback(battery_status_callback_t callback);

// ============= MP2672A Control Functions =============

/**
 * @brief Write a value to MP2672A register
 * @param reg Register address
 * @param value Value to write
 * @return ESP_OK on success
 */
esp_err_t mp2672a_write_register(uint8_t reg, uint8_t value);

/**
 * @brief Read a value from MP2672A register
 * @param reg Register address
 * @param value Pointer to store read value
 * @return ESP_OK on success
 */
esp_err_t mp2672a_read_register(uint8_t reg, uint8_t *value);

/**
 * @brief Modify a single bit in MP2672A register
 * @param reg Register address
 * @param bit_pos Bit position (0-7)
 * @param bit_value true to set bit, false to clear
 * @return ESP_OK on success
 */
esp_err_t mp2672a_modify_register(uint8_t reg, uint8_t bit_pos, bool bit_value);

/**
 * @brief Configure MP2672A to run from battery (disable charging, suspend boost)
 * Used when power button is pressed to ensure SYS runs from battery even if VIN present
 * @return ESP_OK on success
 */
esp_err_t mp2672a_disable_charging_enable_battery_power(void);

/**
 * @brief Restore MP2672A to normal charging mode
 * Used when power is disabled to allow normal charging operation
 * @return ESP_OK on success
 */
esp_err_t mp2672a_enable_charging_normal_mode(void);

/**
 * @brief Parse charge status from MP2672A status register
 * @param status_reg Value of REG03 status register
 * @return Charge status enum
 */
charge_status_t mp2672a_parse_charge_status(uint8_t status_reg);

#endif // BATTERY_MONITOR_H