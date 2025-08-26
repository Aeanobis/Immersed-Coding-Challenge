#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

// Wrap C headers:
extern "C" {
#include "../components/battery_monitor/include/battery_monitor.h"

}

static const char *TAG = "MAIN";

// ============= Pin Definitions =============
#define LED_25_GPIO       GPIO_NUM_16
#define LED_50_GPIO       GPIO_NUM_17
#define LED_75_GPIO       GPIO_NUM_18
#define LED_100_GPIO      GPIO_NUM_19
#define BUTTON_GPIO       GPIO_NUM_33
#define POWER_ENABLE_GPIO GPIO_NUM_13
#define LED_WIFI_GPIO     GPIO_NUM_25   //BLUE LED

// ============= Task Configuration =============
#define BATTERY_MONITOR_TASK_PRIORITY  5
#define LED_CONTROL_TASK_PRIORITY      4
#define BUTTON_HANDLER_TASK_PRIORITY   6
#define TASK_STACK_SIZE                4096

// ============= Timing Configuration =============
#define BATTERY_SAMPLE_PERIOD_MS    1000
#define LED_UPDATE_PERIOD_MS        500
#define BUTTON_DEBOUNCE_MS          50
#define BUTTON_HOLD_TIME_MS         1000

// ============= Global State =============
static SemaphoreHandle_t battery_data_mutex = NULL;
static battery_data_t g_battery_data = {0};
static bool g_power_enabled = false;

// ============= UI Helper Functions =============

static void update_led_pattern(uint8_t soc_percent, bool is_charging)
{
    static uint32_t blink_counter = 0;
    bool blink_state = (blink_counter++ % 2) == 0;

    if (is_charging) {
        // Charging animation with progressive blinking
        if (soc_percent < 25) {
            gpio_set_level(LED_25_GPIO, blink_state);
            gpio_set_level(LED_50_GPIO, 0);
            gpio_set_level(LED_75_GPIO, 0);
            gpio_set_level(LED_100_GPIO, 0);
        } else if (soc_percent < 50) {
            gpio_set_level(LED_25_GPIO, 1);
            gpio_set_level(LED_50_GPIO, blink_state);
            gpio_set_level(LED_75_GPIO, 0);
            gpio_set_level(LED_100_GPIO, 0);
        } else if (soc_percent < 75) {
            gpio_set_level(LED_25_GPIO, 1);
            gpio_set_level(LED_50_GPIO, 1);
            gpio_set_level(LED_75_GPIO, blink_state);
            gpio_set_level(LED_100_GPIO, 0);
        } else if (soc_percent < 100) {
            gpio_set_level(LED_25_GPIO, 1);
            gpio_set_level(LED_50_GPIO, 1);
            gpio_set_level(LED_75_GPIO, 1);
            gpio_set_level(LED_100_GPIO, blink_state);
        } else {
            // Fully charged - all solid
            gpio_set_level(LED_25_GPIO, 1);
            gpio_set_level(LED_50_GPIO, 1);
            gpio_set_level(LED_75_GPIO, 1);
            gpio_set_level(LED_100_GPIO, 1);
        }
    } else {
        // Not charging - simple level display (use soc_percent passed in)
        uint8_t led_level =
            (soc_percent >= 75) ? 4 :
            (soc_percent >= 50) ? 3 :
            (soc_percent >= 25) ? 2 :
            (soc_percent >  0 ) ? 1 : 0;

        gpio_set_level(LED_25_GPIO, led_level >= 1);
        gpio_set_level(LED_50_GPIO, led_level >= 2);
        gpio_set_level(LED_75_GPIO, led_level >= 3);
        gpio_set_level(LED_100_GPIO, led_level >= 4);
    }
}

static void flash_all_leds(int count, int delay_ms)
{
    for (int i = 0; i < count; i++) {
        gpio_set_level(LED_25_GPIO, 1);
        gpio_set_level(LED_50_GPIO, 1);
        gpio_set_level(LED_75_GPIO, 1);
        gpio_set_level(LED_100_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(LED_25_GPIO, 0);
        gpio_set_level(LED_50_GPIO, 0);
        gpio_set_level(LED_75_GPIO, 0);
        gpio_set_level(LED_100_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}


// ============= Task 1: Battery Monitor =============
static void battery_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Battery monitor task started");
    battery_data_t local_data;
    uint32_t log_counter = 0;
    static charge_status_t last_charge_status = CHARGE_STATUS_NOT_CHARGING;
    
    while (1) {
        // Read all battery data using battery_monitor module
        if (battery_monitor_read(&local_data) == ESP_OK) {
            
            // Update global state
            if (xSemaphoreTake(battery_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                memcpy(&g_battery_data, &local_data, sizeof(battery_data_t));
                xSemaphoreGive(battery_data_mutex);
            }

            // Keep charging enabled unless there are safety issues
            // Let MP2672A power path management handle charge + power distribution
            if (!(local_data.status & (BATTERY_STATUS_OVERVOLTAGE | BATTERY_STATUS_OVERTEMP | 
                                     BATTERY_STATUS_OVERCURRENT | BATTERY_STATUS_FAULT))) {
                (void)mp2672a_enable_charging_normal_mode();
            }
            
            // Log charging status changes
            if (local_data.charge_status != last_charge_status) {
                ESP_LOGI(TAG, "Charging status: %s", 
                         battery_monitor_get_charge_status_string(local_data.charge_status));
                last_charge_status = local_data.charge_status;
            }
            
            // Check for faults
            if (local_data.temperature_c10 == BATTERY_TEMP_UNKNOWN) {
                ESP_LOGE(TAG, "FAULT: NTC sensor fault detected!");
            }
            
            if (local_data.status & BATTERY_STATUS_CRITICAL) {
                ESP_LOGE(TAG, "FAULT: Battery critical - %d mV, %d%%", 
                         local_data.voltage_mv, local_data.soc_percent);
            }
            
            if (local_data.status & BATTERY_STATUS_LOW) {
                ESP_LOGW(TAG, "WARNING: Battery low - %d%%", local_data.soc_percent);
            }
            
            if (local_data.status & BATTERY_STATUS_OVERVOLTAGE) {
                ESP_LOGE(TAG, "FAULT: Overvoltage - %d mV", local_data.voltage_mv);
            }
            
            if (local_data.status & BATTERY_STATUS_OVERTEMP) {
                ESP_LOGE(TAG, "FAULT: Over-temperature - %.1f°C", 
                         local_data.temperature_c10 / 10.0f);
            }
            
            // Periodic status log
            if ((log_counter++ % 10) == 0) {
                bool is_charging = (local_data.charge_status != CHARGE_STATUS_NOT_CHARGING);
                ESP_LOGI(TAG, "Battery: %d mV, %d%%, %.1f°C, %s, Status: %s",
                         local_data.voltage_mv,
                         local_data.soc_percent,
                         local_data.temperature_c10 / 10.0f,
                         is_charging ? "CHARGING" : "DISCHARGING",
                         battery_monitor_get_status_string(local_data.status));
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(BATTERY_SAMPLE_PERIOD_MS));
    }
}

// ============= Task 2: LED Control =============
static void led_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LED control task started");

    // Configure LED GPIOs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_25_GPIO) | (1ULL << LED_50_GPIO) |
                        (1ULL << LED_75_GPIO) | (1ULL << LED_100_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
#ifdef LED_WIFI_GPIO
    io_conf.pin_bit_mask |= (1ULL << LED_WIFI_GPIO);  // optional wireless LED
#endif
    gpio_config(&io_conf);

    // Turn off all LEDs initially
    gpio_set_level(LED_25_GPIO, 0);
    gpio_set_level(LED_50_GPIO, 0);
    gpio_set_level(LED_75_GPIO, 0);
    gpio_set_level(LED_100_GPIO, 0);
#ifdef LED_WIFI_GPIO
    gpio_set_level(LED_WIFI_GPIO, 0);
    // Provided by your Wi-Fi/BLE code; define elsewhere. Default false.
    extern bool g_wireless_connected;
#endif

    // Local timing constants
    const TickType_t faultBlinkTicks = pdMS_TO_TICKS(100);  // rapid blink on fault
    const TickType_t wifiBlinkTicks  = pdMS_TO_TICKS(500);  // slow blink when disconnected

    // Fault mask for LED override
    const battery_status_t FAULT_MASK = (battery_status_t)(
        BATTERY_STATUS_CRITICAL     |
        BATTERY_STATUS_OVERVOLTAGE  |
        BATTERY_STATUS_OVERCURRENT  |
        BATTERY_STATUS_OVERTEMP     |
        BATTERY_STATUS_UNDERVOLTAGE |
        BATTERY_STATUS_UNDERTEMP
    );

    battery_data_t local_data;

    while (1) {
        // Get current battery state
        if (xSemaphoreTake(battery_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            memcpy(&local_data, &g_battery_data, sizeof(battery_data_t));
            xSemaphoreGive(battery_data_mutex);

            // Fault has highest priority: rapid blink all 4 LEDs
            bool fault_active = (local_data.status & FAULT_MASK) != 0;
            if (fault_active) {
                bool fast = ((xTaskGetTickCount() / faultBlinkTicks) & 1) == 0;
                gpio_set_level(LED_25_GPIO,  fast);
                gpio_set_level(LED_50_GPIO,  fast);
                gpio_set_level(LED_75_GPIO,  fast);
                gpio_set_level(LED_100_GPIO, fast);
            } else {
                // Normal battery display: charging animation or solid level
                bool is_charging = (local_data.charge_status != CHARGE_STATUS_NOT_CHARGING);
                update_led_pattern(local_data.soc_percent, is_charging);
            }

#ifdef LED_WIFI_GPIO
            // Wireless status LED: solid when connected, slow blink when disconnected
            bool wifi_led = g_wireless_connected
                ? true
                : (((xTaskGetTickCount() / wifiBlinkTicks) & 1) == 0);
            gpio_set_level(LED_WIFI_GPIO, wifi_led);
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(LED_UPDATE_PERIOD_MS));
    }
}


/// ============= Task 3: Button Handler =============
static void button_handler_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Button handler task started");

    // Configure button GPIO
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_conf);

    // Configure power enable GPIO
    gpio_config_t pwr_conf = {
        .pin_bit_mask = (1ULL << POWER_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pwr_conf);
    gpio_set_level(POWER_ENABLE_GPIO, 0);

    const battery_status_t FAULT_CUTOFF_MASK = (battery_status_t)(
        BATTERY_STATUS_CRITICAL     |
        BATTERY_STATUS_OVERVOLTAGE  |
        BATTERY_STATUS_OVERCURRENT  |
        BATTERY_STATUS_OVERTEMP     |
        BATTERY_STATUS_UNDERVOLTAGE |
        BATTERY_STATUS_UNDERTEMP
    );

    uint32_t press_start_ms = 0;
    bool pressed = false;
    bool enabled_this_press = false;  // prevents ON-then-immediate-OFF on same press
    battery_data_t local_data;

    while (1) {
        int level = gpio_get_level(BUTTON_GPIO);

        if (level == 0 && !pressed) {
            // New press
            pressed = true;
            enabled_this_press = false;
            press_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            ESP_LOGI(TAG, "Button pressed");
        }
        else if (level == 1 && pressed) {
            // Release
            pressed = false;
            uint32_t press_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - press_start_ms;
            ESP_LOGI(TAG, "Button released after %lu ms", (unsigned long)press_ms);

            if (press_ms >= BUTTON_HOLD_TIME_MS) {
                // Long-press => TOGGLE on release
                if (xSemaphoreTake(battery_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    memcpy(&local_data, &g_battery_data, sizeof(battery_data_t));
                    xSemaphoreGive(battery_data_mutex);
                } else {
                    // If we fail to read state, be conservative: don't enable
                    local_data.status = BATTERY_STATUS_FAULT;
                }

                if (!g_power_enabled) {
                    // Toggle ON: safety gate
                    bool safe = battery_monitor_is_safe(&local_data) &&
                                ((local_data.status & FAULT_CUTOFF_MASK) == 0) &&
                                (local_data.temperature_c10 != BATTERY_TEMP_UNKNOWN) &&
                                (local_data.voltage_mv >= BATTERY_MIN_VOLTAGE);

                    if (!safe) {
                        ESP_LOGE(TAG, "✗ Power enable DENIED");
                        flash_all_leds(3, 100);
                    } else {
                        // Enable power output - keep charging enabled for charge-and-play
                        gpio_set_level(POWER_ENABLE_GPIO, 1);
                        g_power_enabled = true;
                        enabled_this_press = true; // mark that we enabled on this press
                        ESP_LOGI(TAG, "✓ Power ENABLED (charge-and-play mode)  V=%d mV  SoC=%d%%  T=%.1f°C",
                                 local_data.voltage_mv,
                                 local_data.soc_percent,
                                 local_data.temperature_c10 / 10.0f);
                    }
                } else {
                    // Toggle OFF (but not if we just enabled on this same press)
                    if (enabled_this_press) {
                        // ignore the OFF to avoid on->off within the same long press
                        enabled_this_press = false;
                    } else {
                        gpio_set_level(POWER_ENABLE_GPIO, 0);
                        g_power_enabled = false;
                        ESP_LOGI(TAG, "Power DISABLED");
                    }
                }
            }
        }

        // Runtime fault cut-off while ON
        if (g_power_enabled) {
            if (xSemaphoreTake(battery_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                memcpy(&local_data, &g_battery_data, sizeof(battery_data_t));
                xSemaphoreGive(battery_data_mutex);

                if ((local_data.status & FAULT_CUTOFF_MASK) != 0) {
                    ESP_LOGE(TAG, "⚠️ EMERGENCY SHUTDOWN!");
                    gpio_set_level(POWER_ENABLE_GPIO, 0);
                    g_power_enabled = false;
                    flash_all_leds(10, 50);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
    }
}




// ============= Main Application =============
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=====================================");
    ESP_LOGI(TAG, "Battery Management System v1.0");
    ESP_LOGI(TAG, "=====================================");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize battery monitor module
    ESP_ERROR_CHECK(battery_monitor_init());
    ESP_LOGI(TAG, "Battery monitor initialized");
    
    // Create mutex
    battery_data_mutex = xSemaphoreCreateMutex();
    if (battery_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }
    
    // Create tasks
    xTaskCreate(battery_monitor_task, "battery_monitor", 
                TASK_STACK_SIZE, NULL, 
                BATTERY_MONITOR_TASK_PRIORITY, NULL);
    
    xTaskCreate(led_control_task, "led_control", 
                TASK_STACK_SIZE, NULL, 
                LED_CONTROL_TASK_PRIORITY, NULL);
    
    xTaskCreate(button_handler_task, "button_handler", 
                TASK_STACK_SIZE, NULL, 
                BUTTON_HANDLER_TASK_PRIORITY, NULL);
    
    ESP_LOGI(TAG, "System ready!");
    ESP_LOGI(TAG, "Hold button (GPIO%d) for 1s to enable power", BUTTON_GPIO);
    ESP_LOGI(TAG, "=====================================");
}