# ESP32 Smart Battery pack

A comprehensive battery management system for 2S Li-ion battery packs with intelligent charging control, safety monitoring, and power output management using ESP32 and MP2672A charge controller.

## Features

### Core Functionality
- **Real-time Battery Monitoring**
  - Voltage measurement via ADC with switched voltage divider for power efficiency
  - Current monitoring using INA219 sensor
  - Temperature monitoring via NTC thermistor
  - State of Charge (SoC) calculation based on voltage curves

- **Smart Charging Control**
  - Automatic USB/charger detection via MP2672A
  - Pre-charge, fast charge, and charge termination states
  - Temperature-compensated charging safety
  - Visual charging progress indication

- **Power Output Management**
  - Safe power delivery control with comprehensive safety checks
  - Battery-only operation mode during output (prevents simultaneous charge/discharge)
  - Emergency shutdown on fault conditions
  - Button-controlled power toggle with 1-second hold time

### Safety Features
- Over-voltage protection (>8.6V)
- Under-voltage protection (<6.0V)
- Over-temperature protection (>60°C)
- Under-temperature protection (<0°C)
- Over-current protection (>3A)
- NTC sensor fault detection
- Automatic emergency shutdown

### User Interface
- 4-LED battery level indicator (25%, 50%, 75%, 100%)
- Animated charging indication
- Fault condition visual alerts
- Single button power control

## Hardware Requirements

### Core Components
- **ESP32-WROOM-32E** microcontroller
- **MP2672A** 2-cell Li-ion charge controller
- **INA219** current/power sensor
- **2S Li-ion Battery Pack** (7.4V nominal, 8.4V max)

### Pin Assignments

| Function | GPIO | Description |
|----------|------|-------------|
| Battery Voltage ADC | GPIO35 | ADC1_CH7 for voltage measurement |
| Voltage Divider Control | GPIO32 | P-MOSFET control for power-efficient measurement |
| I2C SDA | GPIO21 | I2C data line |
| I2C SCL | GPIO22 | I2C clock line |
| LED 25% | GPIO16 | Battery level indicator |
| LED 50% | GPIO17 | Battery level indicator |
| LED 75% | GPIO18 | Battery level indicator |
| LED 100% | GPIO19 | Battery level indicator |
| Power Button | GPIO33 | Input with pull-up (active LOW) |
| Power Enable | GPIO13 | Output power control |

### Voltage Divider Circuit
```
Battery (8.4V) → P-MOSFET → R1(220kΩ) → GPIO35 → R2(100kΩ) → GND
                     ↑
                  GPIO32 (control)
```

### I2C Addresses
- MP2672A: `0x09`
- INA219: `0x40`

## Software Architecture

### FreeRTOS Tasks

1. **Battery Monitor Task** (Priority 5)
   - Reads all sensor data at 1Hz
   - Monitors charging status
   - Detects fault conditions
   - Updates shared battery state

2. **LED Control Task** (Priority 4)
   - Updates LED patterns at 2Hz
   - Shows charging animation or battery level
   - Indicates fault conditions with rapid blinking

3. **Button Handler Task** (Priority 6)
   - Monitors button input with debouncing
   - Performs safety checks before power enable
   - Manages MP2672A charging mode
   - Implements emergency shutdown


## Operation

### Power On/Off
- **Hold button for 1 second** to toggle power output
- System performs safety checks before enabling
- LED flash indicates denied power enable (safety fail)

### LED Indicators

#### Normal Operation (Not Charging)
- 1 LED: SoC 1-24%
- 2 LEDs: SoC 25-49%
- 3 LEDs: SoC 50-74%
- 4 LEDs: SoC 75-100%

#### Charging Animation
- SoC < 25%: LED1 blinking
- SoC 25-49%: LED1 solid, LED2 blinking
- SoC 50-74%: LED1-2 solid, LED3 blinking
- SoC 75-99%: LED1-3 solid, LED4 blinking
- SoC 100%: All LEDs solid

#### Fault Indication
- All 4 LEDs rapid blinking
