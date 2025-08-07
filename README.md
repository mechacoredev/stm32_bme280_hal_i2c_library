## Introduction

This is an **ultimate user-friendly** BME280 sensor library designed for **STM32 HAL Library** users. By abstracting the complex sensor logic, it allows even beginners to start reading sensor data within minutes. The library supports both `Polling` and `DMA` modes, offering flexibility and efficient resource management.

## Key Features

- **Effortless Initialization:** The sensor can be initialized with a single function call, abstracting all low-level configurations.
- **Dual Mode Support:** Seamlessly switch between `Polling` (blocking) and `DMA` (non-blocking) communication modes with a simple boolean flag.
- **Optimized for Performance:** The DMA-based approach offloads data transfer from the CPU, making it perfect for time-critical applications.
- **Minimalist Footprint:** The library is written to be as lightweight as possible, containing only the essential code to get the job done.
- **Easy Debugging:** The `BME280_Init` function returns a status code (`BME280_Init_Status`) that clearly indicates the cause of any initialization failure, simplifying the debugging process for the user.

## Getting Started

### Prerequisites
You need an STM32 project configured with the HAL library. Ensure that your selected `I2C` peripheral and the corresponding `DMA` stream for I2C RX are enabled and configured correctly in STM32CubeMX.
**Important:** This library is currently configured to include `stm32f4xx_hal.h`. If you are using a different STM32 family (e.g., STM32F7, STM32L4), please change the `#include` directive in the `bme280.h` file accordingly.

### Installation
1.  Copy the `bme280.h` and `bme280.c` files into your project's `Inc` and `Src` folders, respectively.
2.  Include `bme280.h` in your `main.c` file.
3.  Ensure that the `HAL_I2C_MemRxCpltCallback` function is defined only once in the project (the library already provides a definition).

### Usage Example
Below is a minimal `main.c` example demonstrating how to initialize the library in DMA mode and read sensor data in the main loop.

```c
// main.c
#include "main.h"
#include "bme280.h"

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

bool dma_state=true;
float temp,pres,humi;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();

    // The library is initialized with a single function call.
    BME280_Init_Status status = BME280_Init(&hi2c1, dma_state);
    if (status != Init_OK) {
        // Handle initialization error
        Error_Handler();
    }
    HAL_Delay(100);

    while (1) {
        BME280_ReadSensor(&temp, &pres, &humi);
        HAL_Delay(50);
    }
}
//... (Rest of the CubeMX generated code)
Contributing
If you'd like to contribute, feel free to open a pull request or report any issues.

License
This project is licensed under the MIT License

## License

MIT License

Copyright (c) 2025 Enes E.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.