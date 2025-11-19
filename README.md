# esp32-ble-bsp
Board Support Package for ESP32 BLE (NimBLE + GATT Services +GAP+ Notifications)
# ESP32 BLE BSP (NimBLE)

Board Support Package for ESP32 BLE using NimBLE stack.
Includes:
- GATT service + characteristics
- Notifications & Indications
- BSP abstraction layer (ble_bsp_init, ble_bsp_notify, ...)
- Thread-safe queue for BLE events

## How to use
1. Add the `components/ble_bsp` directory to your ESP-IDF project.
2. Call `ble_bsp_init()` in app_main().
3. Use `ble_bsp_notify()` to send data.

## Requirements
- ESP-IDF v4.4.5

