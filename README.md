# BME280 Driver for STM32 HAL

A professional, handle-based C library for the Bosch BME280 Temperature, Pressure, and Humidity sensor, designed for STM32 microcontrollers using the HAL library.

This library was co-developed by **Enes E.** and Gemini, focusing on robust, reusable, and efficient code for embedded systems.

## Why Use This Library? (Advantages)

This library is built with modern embedded software design principles to be both powerful and easy to use.

-   **➡️ Handle-Based Architecture (Multi-Device Ready)**
    -   No global variables are used. You can create as many `BME280_t` objects as you need to manage multiple sensors simultaneously, even on the same I2C bus. This makes the code clean and highly reusable across projects.

-   **➡️ Dual Communication Modes**
    -   **Polling Mode:** A simple, blocking `ReadSensor_Polling()` function for quick and easy implementation.
    -   **Continuous DMA Mode:** A highly efficient, non-blocking `ReadSensor_DMA_Start()` system. It continuously updates sensor data in the background using interrupts with almost zero CPU overhead, leaving your main loop free for other tasks.

-   **➡️ Smart and Robust**
    -   **Auto-Detection:** The `BME280_Init()` function can automatically scan for the sensor at its primary (0x76) or secondary (0x77) I2C address if you pass `0`.
    -   **Error Handling:** Functions return a `BME280_Status_t` code, allowing you to check for communication failures or hardware issues instead of letting your program crash silently.

-   **➡️ Clean and Well-Documented API**
    -   The public functions are intuitive and the `bme280.h` header file is fully commented in Doxygen format for clarity.

## Setup

#### 1. Add Files to Project
-   Copy `bme280.h` into your project's `Core/Inc` directory.
-   Copy `bme280.c` into your project's `Core/Src` directory.
-   Include the header in your `main.c`: `#include "bme280.h"`

#### 2. STM32CubeMX Configuration
-   Enable an **I2C** peripheral (e.g., I2C1).
-   **For DMA Mode**, you must also:
    -   In I2C Settings -> **DMA Settings** tab, add a DMA request for **I2C_RX**.
    -   In System Core -> **NVIC** tab, enable the **`I2C event interrupt`**.

## Basic Usage Workflow

The general workflow is simple:

1.  Create a `BME280_t` device object and a `BME280_Config_t` configuration object.
2.  Call `BME280_Init()` to initialize the sensor handle.
3.  Fill your `BME280_Config_t` object with your desired settings (mode, filter, etc.).
4.  Call `BME280_Configure()` to apply the settings to the sensor.
5.  Choose your reading method:
    -   Call `BME280_ReadSensor_Polling()` whenever you need a reading.
    -   Or, call `BME280_ReadSensor_DMA_Start()` **once** to begin the continuous background updates. In your main loop, simply check the `dev->data_ready` flag to see when new data is available.

**For a complete, working example, please refer to the `main.c` file in this repository.**

## License

This project is licensed under the MIT License.
