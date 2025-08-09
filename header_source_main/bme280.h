/**
 * @file       bme280.h
 * @brief      Professional, handle-based BME280 sensor library for STM32 HAL.
 * @author     Enes E.
 * @date       Aug 9, 2025
 *
 * @note       This library is designed to be reusable and support multiple
 * BME280 sensors simultaneously by passing a device handle
 * to each function. It focuses on clarity, efficiency,
 * and adherence to professional C practices.
 */

#ifndef BME280_H_
#define BME280_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define BME280_I2C_ADDR_PRIM   (0x76 << 1)
#define BME280_I2C_ADDR_SEC    (0x77 << 1)
#define BME280_CHIP_ID         0x60

#define BME280_REG_HUM_LSB     0xFE
#define BME280_REG_HUM_MSB     0xFD
#define BME280_REG_TEMP_XLSB   0xFC
#define BME280_REG_TEMP_LSB    0xFB
#define BME280_REG_TEMP_MSB    0xFA
#define BME280_REG_PRESS_XLSB  0xF9
#define BME280_REG_PRESS_LSB   0xF8
#define BME280_REG_PRESS_MSB   0xF7
#define BME280_REG_CONFIG      0xF5
#define BME280_REG_CTRL_MEAS   0xF4
#define BME280_REG_STATUS      0xF3
#define BME280_REG_CTRL_HUM    0xF2
#define BME280_REG_RESET       0xE0
#define BME280_REG_ID          0xD0
#define BME280_REG_CALIB_00    0x88
#define BME280_REG_CALIB_26    0xE1

typedef enum {
    BME280_MODE_SLEEP  = 0x00,
    BME280_MODE_FORCED = 0x01,
    BME280_MODE_NORMAL = 0x11
} BME280_Mode;

typedef enum {
    BME280_FILTER_OFF = 0x00,
    BME280_FILTER_2   = 0x01,
    BME280_FILTER_4   = 0x02,
    BME280_FILTER_8   = 0x03,
    BME280_FILTER_16  = 0x04
} BME280_Filter;

typedef enum {
    BME280_OVERSAMPLING_SKIPPED = 0x00,
    BME280_OVERSAMPLING_X1      = 0x01,
    BME280_OVERSAMPLING_X2      = 0x02,
    BME280_OVERSAMPLING_X4      = 0x03,
    BME280_OVERSAMPLING_X8      = 0x04,
    BME280_OVERSAMPLING_X16     = 0x05
} BME280_Oversampling;

typedef enum {
    BME280_STANDBY_0_5_MS  = 0x00,
    BME280_STANDBY_62_5_MS = 0x01,
    BME280_STANDBY_125_MS  = 0x02,
    BME280_STANDBY_250_MS  = 0x03,
    BME280_STANDBY_500_MS  = 0x04,
    BME280_STANDBY_1000_MS = 0x05,
    BME280_STANDBY_2000_MS = 0x06,
    BME280_STANDBY_4000_MS = 0x07
} BME280_StandbyTime;

typedef enum {
    BME280_OK = 0,
    BME280_ERROR_COMM,
    BME280_ERROR_DEV_NOT_FOUND,
    BME280_ERROR_CHIP_ID,
    BME280_ERROR_INVALID_PARAM,
	BME280_ERROR_MANY_DEVICES,
} BME280_Status_t;


typedef struct {
    I2C_HandleTypeDef* i2c_handle;
    uint8_t            addr;

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
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
    int32_t  t_fine;

    float    temperature;
    float    pressure;
    float    humidity;

    uint8_t  raw_data[8];
    volatile uint8_t data_ready;

} BME280_t;

typedef struct {
    BME280_Mode          mode;
    BME280_Filter          filter;
    BME280_Oversampling    oversampling_pressure;
    BME280_Oversampling    oversampling_temperature;
    BME280_Oversampling    oversampling_humidity;
    BME280_StandbyTime     standby_time;
} BME280_Config_t;

BME280_Status_t BME280_Init(BME280_t *dev, I2C_HandleTypeDef *i2c_handle, uint8_t addr);

BME280_Status_t BME280_Configure(BME280_t *dev, BME280_Config_t *config);

BME280_Status_t BME280_ReadSensor_Polling(BME280_t *dev);

BME280_Status_t BME280_ReadSensor_DMA_Start(BME280_t *dev); // Sadece ilk okumayı başlatır

void BME280_I2C_Callback(I2C_HandleTypeDef *hi2c); // Kesme fonksiyonumuz

void _bme280_calculate_values(BME280_t *dev);

uint32_t compensate_humidity(BME280_t *dev, int32_t adc_H);

uint32_t compensate_pressure(BME280_t *dev, int32_t adc_P);

int32_t compensate_temperature(BME280_t *dev, int32_t adc_T);

uint8_t BME280_AutoDetect(BME280_t *dev, I2C_HandleTypeDef *i2c_handle);

BME280_Status_t BME280_ReadSensor_DMA(BME280_t *dev);

#endif /* BME280_H_ */
