/*
 * bme280.h
 *
 * @file       bme280.h
 * @brief      Header file for the BME280 sensor library.
 * This library provides a user-friendly API for interfacing with the BME280 sensor
 * using the STM32 HAL Library, supporting both Polling and DMA modes.
 *
 * Created on: Aug 6, 2025
 * Author: Enes E.
 *
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_


#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define BME280_HUM_LSB 0xFE
#define BME280_HUM_MSB 0xFD
#define BME280_TEMP_XLSB 0xFC
#define BME280_TEMP_LSB 0xFB
#define BME280_TEMP_MSB 0xFA
#define BME280_PRESS_XLSB 0xF9
#define BME280_PRESS_LSB 0xF8
#define BME280_PRESS_MSB 0xF7
#define BME280_CONFIG_ADDR 0xF5
#define BME280_CTRL_MEAS_ADDR 0xF4
#define BME280_STATUS_ADDR 0xF3
#define BME280_CTRL_HUM_ADDR 0xF2
#define BME280_CALIB26_41 0xE1
#define BME280_RESET_ADDR 0xE0
#define BME280_RESET_CMMND 0xB6
#define BME280_ID 0xD0
#define BME280_CALIB00_25 0x88
#define BME280_Dig_H1_ADDR 0xA1

/**
 * @brief  Operating modes for the BME280 sensor.
 */
typedef enum{
	BME280_Mode_Sleep = 0,      /**< Sleep mode (lowest power consumption). */
	BME280_Mode_Forced = 1,     /**< Forced mode (single measurement, then returns to sleep). */
	BME280_Mode_Normal = 3,     /**< Normal mode (continuous measurements). */
}BME280_Mode;

/**
 * @brief  IIR filter coefficients for the BME280.
 */
typedef enum{
	BME280_Filter_Off = 0,      /**< IIR filter off. */
	BME280_Filter_2 = 1,        /**< IIR filter with coefficient 2. */
	BME280_Filter_4 = 2,        /**< IIR filter with coefficient 4. */
	BME280_Filter_8 = 3,        /**< IIR filter with coefficient 8. */
	BME280_Filter_16 = 4,       /**< IIR filter with coefficient 16. */
}BME280_Filter;

/**
 * @brief  Oversampling settings for pressure, temperature and humidity.
 */
typedef enum{
	BME280_Oversampling_Skip = 0,  /**< Skip measurement. */
	BME280_Oversampling_1 = 1,     /**< Oversampling x1. */
	BME280_Oversampling_2 = 2,     /**< Oversampling x2. */
	BME280_Oversampling_4 = 3,     /**< Oversampling x4. */
	BME280_Oversampling_8 = 4,     /**< Oversampling x8. */
	BME280_Oversampling_16 = 5,    /**< Oversampling x16. */
}BME280_Oversampling;

/**
 * @brief  Standby time settings in Normal mode.
 */
typedef enum{
	BME280_StandbyTime_05 = 0,  /**< 0.5 ms standby. */
	BME280_StandbyTime_62 = 1,  /**< 62.5 ms standby. */
	BME280_StandbyTime_125 = 2, /**< 125 ms standby. */
	BME280_StandbyTime_250 = 3, /**< 250 ms standby. */
	BME280_StandbyTime_500 = 4, /**< 500 ms standby. */
	BME280_StandbyTime_1000 = 5,/**< 1000 ms standby. */
	BME280_StandbyTime_2000 = 6,/**< 2000 ms standby. */
	BME280_StandbyTime_4000 = 7,/**< 4000 ms standby. */
}BME280_StandbyTime;

/**
 * @brief  Communication modes for the BME280 library.
 */
typedef enum{
    BME280_Polling=0,   /**< Blocking communication using HAL Polling functions. */
    BME280_DMA=1,       /**< Non-blocking communication using DMA with a blocking wait loop. */
}BME280_ComMode;

/**
 * @brief  Configuration struct for the BME280 sensor's config register.
 */
typedef struct{
	BME280_StandbyTime standbytime; /**< Standby time. */
    BME280_Filter Filter;           /**< IIR filter coefficient. */
    bool spi_en;                    /**< SPI enable flag. */
}BME280_Config_t;

/**
 * @brief  Configuration struct for the BME280 sensor's ctrl_meas register.
 */
typedef struct{
	BME280_Oversampling oversamp_temp_osrs_t;  /**< Temperature oversampling setting. */
	BME280_Oversampling oversamp_pres_osrs_p;  /**< Pressure oversampling setting. */
	BME280_Mode mode;                          /**< Operating mode. */
}BME280_Ctrl_Meas_t;

/**
 * @brief  Configuration struct for the BME280 sensor's ctrl_hum register.
 */
typedef struct{
	BME280_Oversampling oversamp_humi_osrs_h;  /**< Humidity oversampling setting. */
}BME280_Ctrl_Hum_t;

/**
 * @brief  Combined parameter struct for user-defined configurations.
 */
typedef struct{
	BME280_Config_t Config;        /**< Configuration register settings. */
	BME280_Ctrl_Meas_t Ctrl_Meas;  /**< Control and measurement register settings. */
	BME280_Ctrl_Hum_t Ctrl_Hum;    /**< Humidity control register settings. */
    BME280_ComMode commode;        /**< Communication mode (DMA or Polling). */
    bool dma_transfer_complete;    /**< Flag to indicate DMA transfer completion. */
}BME280_Params_t;

/**
 * @brief  BME280 calibration parameters and device handle.
 * This struct holds all necessary information for sensor operation,
 * including calibration data, I2C handle, and communication status.
 */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    /* Humidity compensation for BME280 */
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;

    uint16_t addr;                  /**< I2C address of the sensor. */
    I2C_HandleTypeDef* i2c;         /**< Pointer to the I2C handle. */
    uint8_t  id;                    /**< Chip ID, for verification. */

} BME280_HandleTypedef;

/**
 * @brief  Status codes for BME280 initialization.
 */
typedef enum{
	Init_OK=0,                     /**< Initialization successful. */
	Device_not_found = 1,          /**< BME280 device could not be found. */
	Register_Config_unavailable =2,/**< Failed to write to the config register. */
	Register_Ctrl_Meas_unavailable =3,/**< Failed to write to the ctrl_meas register. */
	Register_Ctrl_Hum_unavailable =4, /**< Failed to write to the ctrl_hum register. */
	Calibration_unavailable=5,     /**< Failed to read calibration data. */
	Undefined_Parameter=6,         /**< An undefined parameter was passed. */
}BME280_Init_Status;

/**
 * @brief  Initializes the BME280 sensor with default settings.
 *
 * @param  hi2cx: Pointer to the I2C handle used for communication.
 * @param  is_dma_open: Boolean flag to select communication mode (true for DMA, false for Polling).
 * @retval BME280_Init_Status: Status of the initialization process.
 */
BME280_Init_Status BME280_Init(I2C_HandleTypeDef *hi2cx, bool is_dma_open);

/**
 * @brief  I2C Memory Receive Complete Callback.
 * @note   This is a weak function from the HAL library.
 * It is defined here to handle DMA transfer completion.
 *
 * @param  hi2c: Pointer to the I2C handle that triggered the callback.
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Initializes the BME280 sensor with user-defined configurations.
 *
 * @param  hi2cx: Pointer to the I2C handle.
 * @param  config: Pointer to a BME280_Params_t struct with desired settings.
 * @retval BME280_Init_Status: Status of the initialization process.
 */
BME280_Init_Status BME280_Init_With_Config(I2C_HandleTypeDef *hi2cx, const BME280_Params_t* config);

/**
 * @brief  Reads the temperature, pressure, and humidity from the BME280 sensor.
 *
 * @param  temp: Pointer to a float variable to store the temperature in °C.
 * @param  pres: Pointer to a float variable to store the pressure in hPa.
 * @param  humi: Pointer to a float variable to store the humidity in %.
 * @retval bool: Returns true if the read operation is successful, otherwise false.
 */
bool BME280_ReadSensor(float *temp, float *pres, float *humi);

#endif /* INC_BME280_H_ */
