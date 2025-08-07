/*
 * bme280.c
 *
 * @file       bme280.c
 * @brief      Source file for the BME280 sensor library.
 * This file contains the low-level functions for I2C communication,
 * data compensation, and sensor initialization.
 *
 * Created on: Aug 6, 2025
 * Author: Enes E.
 *
 */

#include "bme280.h"

BME280_Params_t BME280_parameters;
BME280_HandleTypedef BME280_Handle;

const uint32_t timeout=1000;

/**
 * @brief  Reads data from a BME280 register using either Polling or DMA mode.
 *
 * @param  MemAddress: The register address to read from.
 * @param  pData: Pointer to the buffer where the read data will be stored.
 * @param  Size: The number of bytes to read.
 * @retval bool: True if the read operation is successful, false otherwise.
 */

static bool bme280_readregister(uint16_t MemAddress, uint8_t *pData, uint16_t Size){
	if(!BME280_parameters.commode){
		if(HAL_I2C_Mem_Read(BME280_Handle.i2c, BME280_Handle.addr, MemAddress, 1, pData, Size, timeout)==HAL_OK){
			return true;
		}
		return false;
	}else{
		if(HAL_I2C_Mem_Read_DMA(BME280_Handle.i2c, BME280_Handle.addr, MemAddress, 1, pData, Size)==HAL_OK){
			BME280_parameters.dma_transfer_complete=0;
			while(!BME280_parameters.dma_transfer_complete);
			return true;
		}
		return false;
	}
}

/**
 * @brief  Writes data to a BME280 register.
 *
 * @param  MemAddress: The register address to write to.
 * @param  pData: Pointer to the buffer containing the data to be written.
 * @param  Size: The number of bytes to write.
 * @retval bool: True if the write operation is successful, false otherwise.
 */

static bool bme280_writeregister(uint16_t MemAddress, uint8_t *pData, uint16_t Size){
	if(HAL_I2C_Mem_Write(BME280_Handle.i2c, BME280_Handle.addr, MemAddress, 1, pData, Size, timeout)==HAL_OK){
		return true;
	}
	return false;
}

/**
 * @brief  Reads the calibration data from the BME280 sensor and stores it in the handle struct.
 * @retval bool: True if all calibration data is read successfully, false otherwise.
 */

static bool bme280_readcalibrationdata(){
	uint8_t temp_pres_data[24];
	uint8_t humi_data[7];
	if(!bme280_readregister(BME280_CALIB00_25, temp_pres_data, 24)){
		return false;
	}
	if(!bme280_readregister(BME280_Dig_H1_ADDR, &BME280_Handle.dig_H1, 1)){
		return false;
	}
	if(!bme280_readregister(BME280_CALIB26_41, humi_data, 7)){
		return false;
	}
	BME280_Handle.dig_T1=(uint16_t)(temp_pres_data[1]<<8)|(temp_pres_data[0]);
	BME280_Handle.dig_T2=(int16_t)(temp_pres_data[3]<<8)|(temp_pres_data[2]);
	BME280_Handle.dig_T3=(int16_t)(temp_pres_data[5]<<8)|(temp_pres_data[4]);
	BME280_Handle.dig_P1=(uint16_t)(temp_pres_data[7]<<8)|(temp_pres_data[6]);
	BME280_Handle.dig_P2=(int16_t)(temp_pres_data[9]<<8)|(temp_pres_data[8]);
	BME280_Handle.dig_P3=(int16_t)(temp_pres_data[11]<<8)|(temp_pres_data[10]);
	BME280_Handle.dig_P4=(int16_t)(temp_pres_data[13]<<8)|(temp_pres_data[12]);
	BME280_Handle.dig_P5=(int16_t)(temp_pres_data[15]<<8)|(temp_pres_data[14]);
	BME280_Handle.dig_P6=(int16_t)(temp_pres_data[17]<<8)|(temp_pres_data[16]);
	BME280_Handle.dig_P7=(int16_t)(temp_pres_data[19]<<8)|(temp_pres_data[18]);
	BME280_Handle.dig_P8=(int16_t)(temp_pres_data[21]<<8)|(temp_pres_data[20]);
	BME280_Handle.dig_P9=(int16_t)(temp_pres_data[23]<<8)|(temp_pres_data[22]);
	BME280_Handle.dig_H2=(int16_t)(humi_data[1]<<8)|(humi_data[0]);
	BME280_Handle.dig_H3=(uint8_t)humi_data[2];
	BME280_Handle.dig_H4=(int16_t)(humi_data[3]<<4)|(humi_data[4] & 0x0F);
	BME280_Handle.dig_H5=(int16_t)(humi_data[5]<<4)|((humi_data[4]>>4) & 0x0F);
	BME280_Handle.dig_H6=(int8_t)humi_data[6];
	return true;
}

/**
 * @brief  Compensates the raw temperature value using calibration data.
 * This is the 32-bit compensation formula provided by Bosch.
 *
 * @param  adc_temp: The raw ADC temperature value.
 * @param  fine_temp: Pointer to a variable to store the compensated fine temperature.
 * @retval int32_t: The compensated temperature value in 0.01 °C units.
 */

static inline int32_t compensate_temperature(int32_t adc_temp,
		int32_t *fine_temp) {
	int32_t var1, var2;

	var1 = ((((adc_temp >> 3) - ((int32_t) BME280_Handle.dig_T1 << 1)))
			* (int32_t) BME280_Handle.dig_T2) >> 11;
	var2 = (((((adc_temp >> 4) - (int32_t) BME280_Handle.dig_T1)
			* ((adc_temp >> 4) - (int32_t) BME280_Handle.dig_T1)) >> 12)
			* (int32_t) BME280_Handle.dig_T3) >> 14;

	*fine_temp = var1 + var2;
	return (*fine_temp * 5 + 128) >> 8;
}

/**
 * @brief  Compensates the raw pressure value using calibration data.
 * This is the 32-bit compensation formula provided by Bosch.
 *
 * @param  adc_press: The raw ADC pressure value.
 * @param  fine_temp: The compensated fine temperature value.
 * @retval uint32_t: The compensated pressure value in 256/100 Pa units.
 */

static inline uint32_t compensate_pressure(int32_t adc_press,
		int32_t fine_temp) {
	int64_t var1, var2, p;

	var1 = (int64_t) fine_temp - 128000;
	var2 = var1 * var1 * (int64_t) BME280_Handle.dig_P6;
	var2 = var2 + ((var1 * (int64_t) BME280_Handle.dig_P5) << 17);
	var2 = var2 + (((int64_t) BME280_Handle.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) BME280_Handle.dig_P3) >> 8)
			+ ((var1 * (int64_t) BME280_Handle.dig_P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) BME280_Handle.dig_P1) >> 33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}

	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) BME280_Handle.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) BME280_Handle.dig_P8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + ((int64_t) BME280_Handle.dig_P7 << 4);
	return p;
}

/**
 * @brief  Compensates the raw humidity value using calibration data.
 * This is the 32-bit compensation formula provided by Bosch.
 *
 * @param  adc_hum: The raw ADC humidity value.
 * @param  fine_temp: The compensated fine temperature value.
 * @retval uint32_t: The compensated humidity value in 1024/100 % units.
 */

static inline uint32_t compensate_humidity(int32_t adc_hum,
		int32_t fine_temp) {
	int32_t v_x1_u32r;

	v_x1_u32r = fine_temp - (int32_t) 76800;
	v_x1_u32r = ((((adc_hum << 14) - ((int32_t) BME280_Handle.dig_H4 << 20)
			- ((int32_t) BME280_Handle.dig_H5 * v_x1_u32r)) + (int32_t) 16384) >> 15)
			* (((((((v_x1_u32r * (int32_t) BME280_Handle.dig_H6) >> 10)
					* (((v_x1_u32r * (int32_t) BME280_Handle.dig_H3) >> 11)
							+ (int32_t) 32768)) >> 10) + (int32_t) 2097152)
					* (int32_t) BME280_Handle.dig_H2 + 8192) >> 14);
	v_x1_u32r = v_x1_u32r
			- (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
					* (int32_t) BME280_Handle.dig_H1) >> 4);
	v_x1_u32r = v_x1_u32r < 0 ? 0 : v_x1_u32r;
	v_x1_u32r = v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r;
	return v_x1_u32r >> 12;
}

/**
 * @brief  Reads raw sensor data (temp, pressure, humidity) from the BME280 registers.
 *
 * @param  temperature: Pointer to store the compensated temperature value.
 * @param  pressure: Pointer to store the compensated pressure value.
 * @param  humidity: Pointer to store the compensated humidity value.
 * @retval bool: True if the read operation is successful, false otherwise.
 */

static bool bme280_read_sensor_register(int32_t *temperature, uint32_t *pressure, uint32_t *humidity){
	int32_t adctemperature = 0;
	int32_t adcpressure = 0;
	int32_t adchumidity = 0;
	uint8_t data[8] = { 0 };
	int32_t fine_temp;
	if(!bme280_readregister(BME280_PRESS_MSB, data, 8)){
		return false;
	}
	adcpressure = (data[0] << 12)|(data[1] << 4)|(data[2] >> 4);
	adctemperature = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
	adchumidity = (data[6] << 8) | (data[7]);
	*temperature=compensate_temperature(adctemperature, &fine_temp);
	*pressure=compensate_pressure(adcpressure, fine_temp);
	*humidity=compensate_humidity(adchumidity, fine_temp);
	return true;
}

BME280_Init_Status BME280_Init(I2C_HandleTypeDef *hi2cx, bool is_dma_open){
	uint8_t registerbuffer;
	for(uint8_t add=0; add<2; add++){
		if(HAL_I2C_IsDeviceReady(hi2cx, (236 + add), 1, timeout)==HAL_OK){
			BME280_Handle.addr=(236+add);
			BME280_Handle.i2c=hi2cx;
			break;
		}
	}
	if(BME280_Handle.addr!=236 && BME280_Handle.addr!=237){
		return Device_not_found;
	}
	if(!bme280_writeregister(BME280_RESET_ADDR, (uint8_t*)BME280_RESET_CMMND, 1)){
		return Device_not_found;
	}
	if(!bme280_readregister(BME280_ID, &BME280_Handle.id, 1)){
		return Device_not_found;
	}
	BME280_parameters.commode=is_dma_open;
	BME280_parameters.Ctrl_Hum.oversamp_humi_osrs_h=BME280_Oversampling_4;
	registerbuffer=(uint8_t)(BME280_parameters.Ctrl_Hum.oversamp_humi_osrs_h);
	if(!bme280_writeregister(BME280_CTRL_HUM_ADDR, &registerbuffer, 1)){
		return Register_Ctrl_Hum_unavailable;
	}
	BME280_parameters.Ctrl_Meas.mode=BME280_Mode_Normal;
	BME280_parameters.Ctrl_Meas.oversamp_pres_osrs_p=BME280_Oversampling_4;
	BME280_parameters.Ctrl_Meas.oversamp_temp_osrs_t=BME280_Oversampling_4;
	registerbuffer=(uint8_t)((BME280_parameters.Ctrl_Meas.oversamp_temp_osrs_t<<5)|(BME280_parameters.Ctrl_Meas.oversamp_pres_osrs_p<<2)|(BME280_parameters.Ctrl_Meas.mode));
	if(!bme280_writeregister(BME280_CTRL_MEAS_ADDR, &registerbuffer, 1)){
		return Register_Ctrl_Meas_unavailable;
	}
	BME280_parameters.Config.Filter=BME280_Filter_Off;
	BME280_parameters.Config.spi_en=0;
	BME280_parameters.Config.standbytime=BME280_StandbyTime_250;
	registerbuffer=(uint8_t)(BME280_parameters.Config.standbytime<<5)|(BME280_parameters.Config.Filter<<2)|(BME280_parameters.Config.spi_en);
	if(!bme280_writeregister(BME280_CONFIG_ADDR, &registerbuffer, 1)){
		return Register_Config_unavailable;
	}
	if(!bme280_readcalibrationdata()){
		return Calibration_unavailable;
	}
	return Init_OK;
}

BME280_Init_Status BME280_Init_With_Config(I2C_HandleTypeDef *hi2cx, const BME280_Params_t* config){
	uint8_t registerbuffer;
	for(uint8_t add=0; add<2; add++){
		if(HAL_I2C_IsDeviceReady(hi2cx, (236 + add), 1, timeout)==HAL_OK){
			BME280_Handle.addr=(236+add);
			BME280_Handle.i2c=hi2cx;
			break;
		}
	}
	if(BME280_Handle.addr!=236 && BME280_Handle.addr!=237){
		return Device_not_found;
	}
	if(!bme280_writeregister(BME280_RESET_ADDR, (uint8_t*)BME280_RESET_CMMND, 1)){
		return Device_not_found;
	}
	if(!bme280_readregister(BME280_ID, &BME280_Handle.id, 1)){
		return Device_not_found;
	}
	BME280_parameters.commode=config->commode;
	BME280_parameters.Ctrl_Hum.oversamp_humi_osrs_h=config->Ctrl_Hum.oversamp_humi_osrs_h;
	registerbuffer=(uint8_t)(BME280_parameters.Ctrl_Hum.oversamp_humi_osrs_h);
	if(!bme280_writeregister(BME280_CTRL_HUM_ADDR, &registerbuffer, 1)){
		return Register_Ctrl_Hum_unavailable;
	}
	BME280_parameters.Ctrl_Meas.mode=config->Ctrl_Meas.mode;
	BME280_parameters.Ctrl_Meas.oversamp_pres_osrs_p=config->Ctrl_Meas.oversamp_pres_osrs_p;
	BME280_parameters.Ctrl_Meas.oversamp_temp_osrs_t=config->Ctrl_Meas.oversamp_temp_osrs_t;
	registerbuffer=(uint8_t)((BME280_parameters.Ctrl_Meas.oversamp_temp_osrs_t<<5)|(BME280_parameters.Ctrl_Meas.oversamp_pres_osrs_p<<2)|(BME280_parameters.Ctrl_Meas.mode));
	if(!bme280_writeregister(BME280_CTRL_MEAS_ADDR, &registerbuffer, 1)){
		return Register_Ctrl_Meas_unavailable;
	}
	BME280_parameters.Config.Filter=config->Config.Filter;
	BME280_parameters.Config.spi_en=0;
	BME280_parameters.Config.standbytime=config->Config.standbytime;
	registerbuffer=(uint8_t)(BME280_parameters.Config.standbytime<<5)|(BME280_parameters.Config.Filter<<2)|(BME280_parameters.Config.spi_en);
	if(!bme280_writeregister(BME280_CONFIG_ADDR, &registerbuffer, 1)){
		return Register_Config_unavailable;
	}
	if(!bme280_readcalibrationdata()){
		return Calibration_unavailable;
	}
	BME280_parameters.commode=config->commode;
	return Init_OK;
}

bool BME280_ReadSensor(float *temp, float *pres, float *humi){
	int32_t fix_temp;
	uint32_t fix_pres;
	uint32_t fix_humi;
	if(!bme280_read_sensor_register(&fix_temp, &fix_pres, &fix_humi)){
		return false;
	}
	*temp=(float)fix_temp/100;
	*pres=(float)(fix_pres/256)/100;
	*humi=(float)fix_humi/1024;
	return true;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == BME280_Handle.i2c->Instance)
    {
        BME280_parameters.dma_transfer_complete=1;
    }
}
