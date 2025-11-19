# esp32-ble-bsp
Board Support Package for ESP32 BLE (NimBLE + GATT Services +GAP+ Notifications)
# ESP32 BLE BSP (NimBLE)

Board Support Package for ESP32 BLE using NimBLE stack.
📘 ESP32 BSP BLE – Board Support Package (NimBLE)

Version: 1.0.0
License: MIT


📌 Overview

This repository contains a Board Support Package (BSP) for handling Bluetooth Low Energy (BLE) communication using the ESP32 NimBLE stack.
It provides a clean abstraction layer that allows applications to control BLE operations through a unified API based on the function:

bsp_ble_control(bsp_ctrl_t ctrl, uint8_t argc, void** argv)


This BSP is designed to be easily integrated into embedded software following a modular architecture.

🎯 Features

✔ Start / stop BLE advertising
✔ Add BLE services and characteristics
✔ Read and write characteristic values
✔ Enable notifications and indications
✔ Configure advertising and scanning parameters
✔ BLE central/peripheral connection control
✔ Generic command-based API (bsp_ble_control)
✔ Lightweight and suitable for RTOS environments



⚙️ API Description
🔧 Initialization
bsp_err_t bsp_ble_init(void);
bsp_err_t bsp_ble_exit(void);

🎛️ Control Interface

All BLE actions are executed using:

bsp_err_t bsp_ble_control(bsp_ctrl_t ctrlId, uint8_t argc, void** argv);

📚 Commands (bsp_ctrl_t)

Examples:

Command	Description
BSP_COM_BLE_ADV_START	Start advertising

BSP_COM_BLE_ADV_STOP	Stop advertising

BSP_COM_BLE_ADD_SERVICE	Add a BLE service

BSP_COM_BLE_ADD_CARACTERISTIC	Add a characteristic

BSP_COM_BLE_ADD_NOTIFICATION	Enable Notify

BSP_COM_BLE_ADD_INDICATE	Enable Indicate

BSP_COM_BLE_SCAN_START	Start scanning

BSP_COM_BLE_SCAN_STOP	Stop scanning

BSP_COM_BLE_CENTRAL_WRITE	Central write

BSP_COM_BLE_CENTRAL_READ	Central read
