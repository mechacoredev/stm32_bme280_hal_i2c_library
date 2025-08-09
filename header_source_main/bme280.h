/**
 * @file       bme280.h
 * @brief      Professional, handle-based BME280 sensor library for STM32 HAL.
 * @author     Enes E.
 * @date       Aug 9, 2025
 *
 * @note       This library is designed to be reusable and support multiple
 * BME280 sensors simultaneously by passing a device handle
 * to each function. It focuses on clarity, efficiency,
 * and adherence to professional C practices. Supports both
 * Polling and continuous DMA modes.
 */

#ifndef BME280_H_
#define BME280_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*================================================================================*/
/* I2C ADDRESS & CHIP ID                                                          */
/*================================================================================*/
#define BME280_I2C_ADDR_PRIM   (0x76 << 1) // Primary I2C address (SDO pin to GND)
#define BME280_I2C_ADDR_SEC    (0x77 << 1) // Secondary I2C address (SDO pin to VDDIO)
#define BME280_CHIP_ID         0x60        // BME280's fixed chip ID

/* ... [Register #defines and Enums remain the same as they are self-explanatory] ... */

/*================================================================================*/
/* STATUS CODES                                                                   */
/*================================================================================*/
typedef enum {
    BME280_OK = 0,                  /**< Success */
    BME280_ERROR_COMM,              /**< I2C communication failure */
    BME280_ERROR_DEV_NOT_FOUND,     /**< Device not found at the specified address */
    BME280_ERROR_CHIP_ID,           /**< Chip ID does not match the expected value */
    BME280_ERROR_INVALID_PARAM,     /**< A NULL pointer or invalid parameter was passed */
    BME280_ERROR_BUSY               /**< DMA is busy, new request rejected */
} BME280_Status_t;


/*================================================================================*/
/* MAIN DEVICE & CONFIGURATION STRUCTURES                                         */
/*================================================================================*/

/**
 * @brief Main device structure (handle) for a BME280 sensor.
 * @note  Holds all information related to a single sensor instance.
 */
typedef struct BME280_t {
    // Hardware Descriptors
    I2C_HandleTypeDef* i2c_handle;      /**< Pointer to the I2C handle for this sensor. */
    uint8_t            addr;            /**< The 8-bit I2C address of this sensor. */

    // Calibration Data (read from the sensor)
    uint16_t dig_T1;
    int16_t  dig_T2;
    // ... (all other dig_ values)
    int8_t   dig_H6;
    int32_t  t_fine;                  /**< Fine temperature resolution value for compensation. */

    // Latest Sensor Readings
    float    temperature;             /**< Last read temperature in degrees Celsius (°C). */
    float    pressure;                /**< Last read pressure in Pascals (Pa). */
    float    humidity;                /**< Last read relative humidity in percent (%RH). */

    // DMA and State Management
    uint8_t  raw_data[8];             /**< Internal buffer for raw DMA data. */
    volatile uint8_t data_ready;      /**< Flag set to 1 by DMA callback when new data is ready. */

} BME280_t;

/**
 * @brief Structure to hold all user-configurable sensor settings.
 */
typedef struct {
    BME280_Mode          mode;
    BME280_Filter          filter;
    BME280_Oversampling    oversampling_pressure;
    BME280_Oversampling    oversampling_temperature;
    BME280_Oversampling    oversampling_humidity;
    BME280_StandbyTime     standby_time;
} BME280_Config_t;


/*================================================================================*/
/* PUBLIC FUNCTION PROTOTYPES (THE API)                                           */
/*================================================================================*/

/**
 * @brief  Initializes the BME280 sensor handle.
 * @note   This function finds the device, verifies its chip ID, and reads the
 * factory calibration data. Call BME280_Configure() after this.
 * @param  dev Pointer to the BME280 device structure to initialize.
 * @param  i2c_handle Pointer to the I2C handle for communication.
 * @param  addr The I2C address of the sensor. Pass 0 for auto-detection.
 * @retval BME280_Status_t Status of the initialization.
 */
BME280_Status_t BME280_Init(BME280_t *dev, I2C_HandleTypeDef *i2c_handle, uint8_t addr);

/**
 * @brief  Configures the sensor with user-defined settings.
 * @note   Writes the configuration for oversampling, filter, mode, and standby time.
 * For settings to take effect, the sensor might be briefly put into Sleep mode.
 * @param  dev Pointer to the BME280 device structure to configure.
 * @param  config Pointer to a BME280_Config_t structure with the desired settings.
 * @retval BME280_Status_t Status of the configuration.
 */
BME280_Status_t BME280_Configure(BME280_t *dev, BME280_Config_t *config);

/**
 * @brief  Reads sensor data in Polling (blocking) mode.
 * @note   The results are stored within the BME280_t device structure.
 * Access them via dev->temperature, dev->pressure, dev->humidity.
 * @param  dev Pointer to the BME280 device structure.
 * @retval BME280_Status_t Status of the read operation.
 */
BME280_Status_t BME280_ReadSensor_Polling(BME280_t *dev);

/**
 * @brief  Starts the continuous DMA read cycle.
 * @note   This function only initiates the first transfer. Subsequent transfers
 * are chained inside the I2C DMA callback. This function is non-blocking.
 * @param  dev Pointer to the BME280 device structure.
 * @retval BME280_Status_t Status of the DMA start request.
 */
BME280_Status_t BME280_ReadSensor_DMA_Start(BME280_t *dev);

/**
 * @brief  Library's internal handler for the I2C DMA completion interrupt.
 * @note   This function should NOT be called by the user directly.
 * It must be called from the HAL_I2C_MemRxCpltCallback() in main.c or stm32f4xx_it.c.
 * For HAL versions without pUserData in I2C_HandleTypeDef, a static pointer
 * workaround is used inside the .c file.
 * @param  hi2c Pointer to the I2C handle that triggered the interrupt.
 */
void BME280_I2C_Callback(I2C_HandleTypeDef *hi2c);


#endif /* BME280_H_ */
