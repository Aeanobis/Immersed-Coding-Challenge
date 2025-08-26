// battery_monitor.c - Complete battery monitoring with MP2672A control
#include "battery_monitor.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "esp_rom_sys.h"  // for ets_delay_us



static const char *TAG = "battery_monitor";

// ADC Configuration
#define VOLTAGE_ADC_CHANNEL     ADC1_CHANNEL_7   // GPIO35
#define VOLTAGE_DIVIDER_GPIO    32               // GPIO32 controls P-MOSFET
#define ADC_SAMPLES             64               // Number of samples to average
#define ADC_ATTEN              ADC_ATTEN_DB_11   // Full scale 0-3.3V
#define DEFAULT_VREF           1100              // Default VREF in mV

// Voltage divider constants (R1=220k, R2=100k)
#define DIVIDER_RATIO          0.3125f           // R2/(R1+R2) = 100k/320k

// Battery pack constants (2S Li-ion)
#define VBAT_FULL_MV          8400              // 2 * 4.2V
#define VBAT_NOMINAL_MV       7400              // 2 * 3.7V  
#define VBAT_EMPTY_MV         6000              // 2 * 3.0V

// MP2672A Registers
#define MP2672A_REG00          0x00              // Battery regulation, charge config
#define MP2672A_REG02          0x02              // Boost control
#define MP2672A_REG03          0x03              // Status register
#define MP2672A_REG04          0x04              // Fault register
#define MP2672A_REG05          0x05              // Recharge threshold
#define MP2672A_REG06          0x06              // NTC register

// MP2672A Register Bits
#define CHG_CONFIG_BIT         4                 // REG00[4]: 0=disable charging, 1=enable
#define EN_SUSP_BIT           0                 // REG02[0]: 0=suspend (battery), 1=normal
#define RCHG_BIT              7                 // REG05[7]: Auto-recharge control

// Globals
static TaskHandle_t battery_task_handle = NULL;
static battery_status_callback_t status_callback = NULL;
static battery_data_t last_battery_data = {0};
static esp_adc_cal_characteristics_t *adc_chars = NULL;

// Private function declarations
static esp_err_t i2c_master_init(void);
static esp_err_t adc_init(void);
static uint16_t read_battery_voltage_mv(void);
static uint8_t calculate_soc_from_voltage(uint16_t voltage_mv);
static int16_t read_ntc_temperature_c10(void);
static esp_err_t read_mp2672a_data(uint8_t *status_reg, uint8_t *fault_reg);
static esp_err_t read_ina219_data(int16_t *current_ma, uint16_t *power_mw);
static battery_status_t evaluate_battery_status(const battery_data_t *data);

// ---- Public API ----

esp_err_t battery_monitor_init(void)
{
    ESP_LOGI(TAG, "Init");
    
    // Initialize I2C
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize ADC and GPIO for voltage monitoring
    ret = adc_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}


esp_err_t battery_monitor_read(battery_data_t *data)
{
    if (!data) return ESP_ERR_INVALID_ARG;
    memset(data, 0, sizeof(*data));

    // Read actual battery voltage via ADC
    data->voltage_mv = read_battery_voltage_mv();
    
    // Calculate SoC from voltage
    data->soc_percent = calculate_soc_from_voltage(data->voltage_mv);
    
    // Read temperature from NTC via MP2672A
    data->temperature_c10 = read_ntc_temperature_c10();
    
    // Read current/power from INA219
    (void)read_ina219_data(&data->current_ma, &data->power_mw);
    
    // Calculate capacity based on SoC
    data->capacity_mah = (data->soc_percent == BATTERY_SOC_UNKNOWN)
                           ? 0
                           : (BATTERY_CAPACITY_MAH * data->soc_percent) / 100;

    // Read MP2672A status/fault registers for additional monitoring
    uint8_t status_reg, fault_reg;
    read_mp2672a_data(&status_reg, &fault_reg);
    
    // Store charging status in data structure
    data->charge_status = mp2672a_parse_charge_status(status_reg);
    
    // Evaluate overall battery status
    data->status = evaluate_battery_status(data);
    
    // Check MP2672A fault register and add to status if needed
    if (fault_reg != 0) {
        ESP_LOGW(TAG, "MP2672A fault detected: 0x%02X", fault_reg);
        data->status |= BATTERY_STATUS_FAULT;
    }
    
    data->timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    return ESP_OK;
}

// ---- MP2672A Control Functions ----

esp_err_t mp2672a_write_register(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MP2672A_I2C_ADDR,
                                     data, sizeof(data), pdMS_TO_TICKS(100));
}

esp_err_t mp2672a_read_register(uint8_t reg, uint8_t *value)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MP2672A_I2C_ADDR,
                                       &reg, 1, value, 1, pdMS_TO_TICKS(100));
}

esp_err_t mp2672a_modify_register(uint8_t reg, uint8_t bit_pos, bool bit_value)
{
    uint8_t current_val;
    esp_err_t ret = mp2672a_read_register(reg, &current_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X", reg);
        return ret;
    }
    
    // Modify the bit
    if (bit_value) {
        current_val |= (1 << bit_pos);   // Set bit
    } else {
        current_val &= ~(1 << bit_pos);  // Clear bit
    }
    
    ret = mp2672a_write_register(reg, current_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X", reg);
        return ret;
    }
    
    ESP_LOGD(TAG, "MP2672A REG%02X modified: bit %d = %d (new value: 0x%02X)", 
             reg, bit_pos, bit_value, current_val);
    return ESP_OK;
}

esp_err_t mp2672a_disable_charging_enable_battery_power(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Configuring MP2672A for battery power mode...");
    
    // Step 1: Disable charging - REG00[4] = 0 (CHG_CONFIG = 0)
    ret = mp2672a_modify_register(MP2672A_REG00, CHG_CONFIG_BIT, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable charging");
        return ret;
    }
    ESP_LOGI(TAG, "  ✓ Charging disabled (REG00[4] = 0)");
    
    // Step 2: Suspend boost, run from battery - REG02[0] = 0 (EN_SUSP = 0)
    ret = mp2672a_modify_register(MP2672A_REG02, EN_SUSP_BIT, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to suspend boost");
        return ret;
    }
    ESP_LOGI(TAG, "  ✓ Boost suspended, running from battery (REG02[0] = 0)");
    
    // Step 3: Optional - Check RCHG bit status (REG05[7])
    uint8_t reg05_val;
    ret = mp2672a_read_register(MP2672A_REG05, &reg05_val);
    if (ret == ESP_OK) {
        bool rchg = (reg05_val >> RCHG_BIT) & 0x01;
        ESP_LOGI(TAG, "  ℹ Auto-recharge is %s (REG05[7] = %d)", 
                rchg ? "enabled" : "disabled", rchg);
    }
    
    ESP_LOGI(TAG, "MP2672A configured: SYS powered from battery via BATFET");
    return ESP_OK;
}

esp_err_t mp2672a_enable_charging_normal_mode(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Restoring MP2672A to normal charging mode...");
    
    // Step 1: Enable normal boost operation - REG02[0] = 1 (EN_SUSP = 1)
    ret = mp2672a_modify_register(MP2672A_REG02, EN_SUSP_BIT, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable normal boost");
        return ret;
    }
    ESP_LOGI(TAG, "  ✓ Normal boost operation enabled (REG02[0] = 1)");
    
    // Step 2: Enable charging - REG00[4] = 1 (CHG_CONFIG = 1)
    ret = mp2672a_modify_register(MP2672A_REG00, CHG_CONFIG_BIT, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable charging");
        return ret;
    }
    ESP_LOGI(TAG, "  ✓ Charging enabled (REG00[4] = 1)");
    
    ESP_LOGI(TAG, "MP2672A restored to normal operation");
    return ESP_OK;
}

charge_status_t mp2672a_parse_charge_status(uint8_t status_reg)
{
    // CHG_STAT[1:0] bits are at positions 3-2 in REG03
    uint8_t chg_stat = (status_reg >> 2) & 0x03;
    
    switch (chg_stat) {
        case 0: return CHARGE_STATUS_NOT_CHARGING;
        case 1: return CHARGE_STATUS_PRE_CHARGE;
        case 2: return CHARGE_STATUS_FAST_CHARGE;
        case 3: return CHARGE_STATUS_CHARGE_COMPLETE;
        default: return CHARGE_STATUS_NOT_CHARGING;
    }
}

bool battery_monitor_is_safe(const battery_data_t *data)
{
    if (!data) return false;
    return (data->status == BATTERY_STATUS_OK ||
            data->status == BATTERY_STATUS_LOW);
}

uint8_t battery_monitor_get_led_level(const battery_data_t *data)
{
    if (!data) return 0;
    if (data->soc_percent == BATTERY_SOC_UNKNOWN) return 0;

    if (data->soc_percent >= 75) return 4;
    else if (data->soc_percent >= 50) return 3;
    else if (data->soc_percent >= 25) return 2;
    else if (data->soc_percent > 0)  return 1;
    else return 0;
}

const char* battery_monitor_get_status_string(battery_status_t status)
{
    switch (status) {
        case BATTERY_STATUS_OK:           return "OK";
        case BATTERY_STATUS_LOW:          return "Low";
        case BATTERY_STATUS_CRITICAL:     return "Critical";
        case BATTERY_STATUS_OVERVOLTAGE:  return "Overvoltage";
        case BATTERY_STATUS_UNDERVOLTAGE: return "Undervoltage";
        case BATTERY_STATUS_OVERCURRENT:  return "Overcurrent";
        case BATTERY_STATUS_OVERTEMP:     return "Overtemperature";
        case BATTERY_STATUS_UNDERTEMP:    return "Undertemperature";
        case BATTERY_STATUS_FAULT:        return "Fault";
        default:                          return "Unknown";
    }
}

const char* battery_monitor_get_charge_status_string(charge_status_t status)
{
    switch (status) {
        case CHARGE_STATUS_NOT_CHARGING:   return "Not Charging";
        case CHARGE_STATUS_PRE_CHARGE:     return "Pre-Charge";
        case CHARGE_STATUS_FAST_CHARGE:    return "Fast Charge";
        case CHARGE_STATUS_CHARGE_COMPLETE: return "Charge Complete";
        default:                            return "Unknown";
    }
}


static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t adc_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << VOLTAGE_DIVIDER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(VOLTAGE_DIVIDER_GPIO, 1);
    
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(VOLTAGE_ADC_CHANNEL, ADC_ATTEN);
    
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, 
                            DEFAULT_VREF, adc_chars);
    return ESP_OK;
}

static uint16_t read_battery_voltage_mv(void)
{
    gpio_set_level(VOLTAGE_DIVIDER_GPIO, 0);
    vTaskDelay(1);
    
    uint32_t adc_reading = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        adc_reading += adc1_get_raw(VOLTAGE_ADC_CHANNEL);
    }
    adc_reading /= ADC_SAMPLES;
    
    gpio_set_level(VOLTAGE_DIVIDER_GPIO, 1);
    
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    uint16_t battery_voltage_mv = (uint16_t)(voltage_mv / DIVIDER_RATIO);
    
    ESP_LOGD(TAG, "ADC raw=%d, ADC mV=%d, Battery mV=%d", 
             (int)adc_reading, (int)voltage_mv, battery_voltage_mv);
    
    return battery_voltage_mv;
}

static uint8_t calculate_soc_from_voltage(uint16_t voltage_mv)
{
    const struct {
        uint16_t voltage_mv;
        uint8_t soc_percent;
    } soc_table[] = {
        {8400, 100}, {8200, 95}, {8000, 90}, {7800, 80}, {7600, 70},
        {7400, 50}, {7200, 30}, {7000, 20}, {6800, 10}, {6600, 5}, {6000, 0},
    };
    
    if (voltage_mv >= soc_table[0].voltage_mv) return 100;
    if (voltage_mv <= soc_table[10].voltage_mv) return 0;
    
    for (int i = 0; i < 10; i++) {
        if (voltage_mv >= soc_table[i+1].voltage_mv) {
            float fraction = (float)(voltage_mv - soc_table[i+1].voltage_mv) / 
                           (float)(soc_table[i].voltage_mv - soc_table[i+1].voltage_mv);
            float soc = soc_table[i+1].soc_percent + 
                       fraction * (soc_table[i].soc_percent - soc_table[i+1].soc_percent);
            return (uint8_t)(soc + 0.5f);
        }
    }
    return 0;
}

static int16_t read_ntc_temperature_c10(void)
{
    uint8_t reg = MP2672A_REG06;
    uint8_t val;
    
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, MP2672A_I2C_ADDR,
                                                 &reg, 1, &val, 1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read NTC register");
        return BATTERY_TEMP_UNKNOWN;
    }
    
    uint8_t ntc_status = (val >> 3) & 0x07;
    
    int16_t temp_c10;
    switch (ntc_status) {
        case 0x00: temp_c10 = 250; break;  // Normal ~25°C
        case 0x01: temp_c10 = 500; break;  // Warm ~50°C
        case 0x02: temp_c10 = 650; break;  // Hot ~65°C
        case 0x03: temp_c10 = -50; break;  // Cold ~-5°C
        default:   temp_c10 = BATTERY_TEMP_UNKNOWN;
    }
    
    ESP_LOGD(TAG, "NTC register=0x%02X, status=%d, temp=%d.%d°C", 
             val, ntc_status, temp_c10/10, abs(temp_c10%10));
    
    return temp_c10;
}

static esp_err_t read_mp2672a_data(uint8_t *status_reg, uint8_t *fault_reg)
{
    if (status_reg) {
        uint8_t reg = MP2672A_REG03;
        esp_err_t ret = mp2672a_read_register(reg, status_reg);
        if (ret == ESP_OK) {
            ESP_LOGD(TAG, "MP2672A STATUS=0x%02X", *status_reg);
        } else {
            *status_reg = 0;
        }
    }

    if (fault_reg) {
        uint8_t reg = MP2672A_REG04;
        esp_err_t ret = mp2672a_read_register(reg, fault_reg);
        if (ret == ESP_OK && *fault_reg != 0) {
            ESP_LOGW(TAG, "MP2672A FAULT=0x%02X", *fault_reg);
        } else if (ret != ESP_OK) {
            *fault_reg = 0;
        }
    }

    return ESP_OK;
}

static esp_err_t read_ina219_data(int16_t *current_ma, uint16_t *power_mw)
{
    uint8_t reg;
    uint8_t buf[2];
    esp_err_t ret;

    if (current_ma) {
        reg = 0x04;
        ret = i2c_master_write_read_device(I2C_MASTER_NUM, INA219_I2C_ADDR,
                                          &reg, 1, buf, 2, pdMS_TO_TICKS(100));
        if (ret == ESP_OK) {
            int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
            *current_ma = raw / 10;
        } else {
            *current_ma = 0;
        }
    }

    if (power_mw) {
        reg = 0x03;
        ret = i2c_master_write_read_device(I2C_MASTER_NUM, INA219_I2C_ADDR,
                                          &reg, 1, buf, 2, pdMS_TO_TICKS(100));
        if (ret == ESP_OK) {
            uint16_t raw = (uint16_t)((buf[0] << 8) | buf[1]);
            *power_mw = raw * 2;
        } else {
            *power_mw = 0;
        }
    }

    return ESP_OK;
}

static battery_status_t evaluate_battery_status(const battery_data_t *data)
{
    battery_status_t status = BATTERY_STATUS_OK;

    if (data->voltage_mv > 0) {
        if (data->voltage_mv > BATTERY_MAX_VOLTAGE) status |= BATTERY_STATUS_OVERVOLTAGE;
        if (data->voltage_mv < BATTERY_MIN_VOLTAGE) status |= BATTERY_STATUS_UNDERVOLTAGE;
    }

    if (abs((int)data->current_ma) > LOAD_MAX_CURRENT_MA) {
        status |= BATTERY_STATUS_OVERCURRENT;
    }

    if (data->temperature_c10 != BATTERY_TEMP_UNKNOWN) {
        if (data->temperature_c10 > BATTERY_MAX_TEMP) status |= BATTERY_STATUS_OVERTEMP;
        if (data->temperature_c10 < BATTERY_MIN_TEMP) status |= BATTERY_STATUS_UNDERTEMP;
    }

    if (data->soc_percent <= 5) {
        status |= BATTERY_STATUS_CRITICAL;
    } else if (data->soc_percent <= 20) {
        status |= BATTERY_STATUS_LOW;
    }

    if (status & (BATTERY_STATUS_OVERVOLTAGE | BATTERY_STATUS_UNDERVOLTAGE |
                  BATTERY_STATUS_OVERCURRENT | BATTERY_STATUS_OVERTEMP |
                  BATTERY_STATUS_UNDERTEMP)) {
        status |= BATTERY_STATUS_FAULT;
    }

    return status;
}