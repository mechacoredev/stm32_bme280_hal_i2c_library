/**
 * @file       bme280.c
 * @brief      Source file for the BME280 sensor library.
 * @author     Enes E.
 * @date       Aug 9, 2025
 */

#include "bme280.h"

/*================================================================================*/
/* PRIVATE (STATIC) HELPER FUNCTIONS                                              */
/*================================================================================*/

static BME280_Status_t _bme280_read_register(BME280_t *dev, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Mem_Read(dev->i2c_handle, dev->addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY) == HAL_OK)
    {
        return BME280_OK;
    }
    return BME280_ERROR_COMM;
}

static BME280_Status_t _bme280_read_register_dma(BME280_t *dev, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Mem_Read_DMA(dev->i2c_handle, dev->addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len) == HAL_OK)
    {
        return BME280_OK;
    }
    return BME280_ERROR_COMM;
}

static BME280_Status_t _bme280_write_register(BME280_t *dev, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Mem_Write(dev->i2c_handle, dev->addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY) == HAL_OK)
    {
        return BME280_OK;
    }
    return BME280_ERROR_COMM;
}

/*================================================================================*/
/* COMPENSATION FORMULAS (FROM BOSCH DATASHEET)                                   */
/*================================================================================*/

int32_t inline compensate_temperature(BME280_t *dev, int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dev->dig_T1 << 1))) * ((int32_t)dev->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dev->dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->dig_T1))) >> 12) * ((int32_t)dev->dig_T3)) >> 14;
    dev->t_fine = var1 + var2;
    T = (dev->t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t inline compensate_pressure(BME280_t *dev, int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int64_t)dev->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->dig_P1) >> 33;
    if (var1 == 0) { return 0; }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->dig_P7) << 4);
    return (uint32_t)p;
}

uint32_t inline compensate_humidity(BME280_t *dev, int32_t adc_H)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (dev->t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - ((((int32_t)dev->dig_H4) << 20) + (((int32_t)dev->dig_H5) * v_x1_u32r))) +
                 ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dev->dig_H6)) >> 10) *
                 (((v_x1_u32r * ((int32_t)dev->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                 ((int32_t)2097152)) * ((int32_t)dev->dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dev->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

void inline _bme280_calculate_values(BME280_t *dev)
{
    int32_t adc_P = (int32_t)(((uint32_t)dev->raw_data[0] << 12) | ((uint32_t)dev->raw_data[1] << 4) | ((uint32_t)dev->raw_data[2] >> 4));
    int32_t adc_T = (int32_t)(((uint32_t)dev->raw_data[3] << 12) | ((uint32_t)dev->raw_data[4] << 4) | ((uint32_t)dev->raw_data[5] >> 4));
    int32_t adc_H = (int32_t)(((uint32_t)dev->raw_data[6] << 8) | dev->raw_data[7]);

    dev->temperature = (float)compensate_temperature(dev, adc_T) / 100.0f;
    dev->pressure = (float)compensate_pressure(dev, adc_P) / 25600.0f;
    dev->humidity = (float)compensate_humidity(dev, adc_H) / 1024.0f;
}

/*================================================================================*/
/* PUBLIC (API) FUNCTIONS                                                         */
/*================================================================================*/

BME280_Status_t BME280_Init(BME280_t *dev, I2C_HandleTypeDef *i2c_handle, uint8_t addr)
{

	dev->i2c_handle = i2c_handle;
    dev->addr=addr;

    if(HAL_I2C_IsDeviceReady(dev->i2c_handle, dev->addr, 2, HAL_MAX_DELAY)!=HAL_OK){
    	return BME280_ERROR_DEV_NOT_FOUND;
    }

    uint8_t chip_id = 0;
    if (_bme280_read_register(dev, BME280_REG_ID, &chip_id, 1) != BME280_OK)
    {
        return BME280_ERROR_COMM;
    }
    if (chip_id != BME280_CHIP_ID)
    {
        return BME280_ERROR_CHIP_ID;
    }

    uint8_t temp_pres_calib[24];
    if (_bme280_read_register(dev, BME280_REG_CALIB_00, temp_pres_calib, 24) != BME280_OK)
    {
        return BME280_ERROR_COMM;
    }

    if (_bme280_read_register(dev, 0xA1, &dev->dig_H1, 1) != BME280_OK)
    {
         return BME280_ERROR_COMM;
    }

    uint8_t hum_calib_data[7];
    if (_bme280_read_register(dev, BME280_REG_CALIB_26, hum_calib_data, 7) != BME280_OK)
    {
        return BME280_ERROR_COMM;
    }

    dev->dig_T1 = (uint16_t)((temp_pres_calib[1] << 8) | temp_pres_calib[0]);
    dev->dig_T2 = (int16_t)((temp_pres_calib[3] << 8) | temp_pres_calib[2]);
    dev->dig_T3 = (int16_t)((temp_pres_calib[5] << 8) | temp_pres_calib[4]);
    dev->dig_P1 = (uint16_t)((temp_pres_calib[7] << 8) | temp_pres_calib[6]);
    dev->dig_P2 = (int16_t)((temp_pres_calib[9] << 8) | temp_pres_calib[8]);
    dev->dig_P3 = (int16_t)((temp_pres_calib[11] << 8) | temp_pres_calib[10]);
    dev->dig_P4 = (int16_t)((temp_pres_calib[13] << 8) | temp_pres_calib[12]);
    dev->dig_P5 = (int16_t)((temp_pres_calib[15] << 8) | temp_pres_calib[14]);
    dev->dig_P6 = (int16_t)((temp_pres_calib[17] << 8) | temp_pres_calib[16]);
    dev->dig_P7 = (int16_t)((temp_pres_calib[19] << 8) | temp_pres_calib[18]);
    dev->dig_P8 = (int16_t)((temp_pres_calib[21] << 8) | temp_pres_calib[20]);
    dev->dig_P9 = (int16_t)((temp_pres_calib[23] << 8) | temp_pres_calib[22]);

    dev->dig_H2 = (int16_t)((hum_calib_data[1] << 8) | hum_calib_data[0]);
    dev->dig_H3 = hum_calib_data[2];
    dev->dig_H4 = (int16_t)((hum_calib_data[3] << 4) | (hum_calib_data[4] & 0x0F));
    dev->dig_H5 = (int16_t)((hum_calib_data[5] << 4) | (hum_calib_data[4] >> 4));
    dev->dig_H6 = (int8_t)hum_calib_data[6];

    return BME280_OK;
}

BME280_Status_t BME280_Configure(BME280_t *dev, BME280_Config_t *config)
{
    if (dev->i2c_handle == NULL)
    {
        return BME280_ERROR_INVALID_PARAM;
    }

    uint8_t reg_data_hum, reg_data_meas, reg_data_config;

    // Ayarları geçici değişkenlerde birleştir
    reg_data_hum = config->oversampling_humidity;
    reg_data_config = (uint8_t)((config->standby_time << 5) | (config->filter << 2));
    reg_data_meas = (uint8_t)((config->oversampling_temperature << 5) | (config->oversampling_pressure << 2) | (config->mode));

    // Değişiklikleri uygulamak için önce SLEEP moduna geçmek en güvenlisidir.
    uint8_t current_mode = reg_data_meas & 0x03;
    if (current_mode != BME280_MODE_SLEEP)
    {
        uint8_t temp_meas = reg_data_meas & 0xFC; // Mod bitlerini sıfırla
        if (_bme280_write_register(dev, BME280_REG_CTRL_MEAS, &temp_meas, 1) != BME280_OK)
        {
            return BME280_ERROR_COMM;
        }
    }

    // Şimdi ayarları yaz
    if (_bme280_write_register(dev, BME280_REG_CTRL_HUM, &reg_data_hum, 1) != BME280_OK)
    {
        return BME280_ERROR_COMM;
    }
    if (_bme280_write_register(dev, BME280_REG_CONFIG, &reg_data_config, 1) != BME280_OK)
    {
        return BME280_ERROR_COMM;
    }
    if (_bme280_write_register(dev, BME280_REG_CTRL_MEAS, &reg_data_meas, 1) != BME280_OK)
    {
        return BME280_ERROR_COMM;
    }

    return BME280_OK;
}

BME280_Status_t BME280_ReadSensor_Polling(BME280_t *dev)
{
    if (_bme280_read_register(dev, BME280_REG_PRESS_MSB, dev->raw_data, 8) != BME280_OK)
    {
        return BME280_ERROR_COMM;
    }

    _bme280_calculate_values(dev); // Hesaplamayı yap
    return BME280_OK;
}

BME280_Status_t BME280_ReadSensor_DMA_Start(BME280_t *dev)
{
	if(_bme280_read_register_dma(dev, BME280_REG_PRESS_MSB, dev->raw_data, 8) != BME280_OK)
	{
		return BME280_ERROR_COMM;
	}
	return BME280_OK;
}

BME280_Status_t inline BME280_ReadSensor_DMA(BME280_t *dev){
	_bme280_calculate_values(dev);
	if(_bme280_read_register_dma(dev, BME280_REG_PRESS_MSB, dev->raw_data, 8) != BME280_OK)
	{
		return BME280_ERROR_COMM;
	}
	return BME280_OK;
}

