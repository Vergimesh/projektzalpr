// File: bmp2.c
#include "bmp2.h"

void BMP280_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t config[2];

    // Configuration for control measurement register
    config[0] = 0xF4; // Address of control register
    config[1] = 0x27; // Normal mode, oversampling x1 for temperature
    HAL_I2C_Master_Transmit(hi2c, BMP280_I2C_ADDRESS, config, 2, HAL_MAX_DELAY);

    // Configuration for configuration register
    config[0] = 0xF5; // Address of configuration register
    config[1] = 0xA0; // Filter x4, standby time 1000 ms
    HAL_I2C_Master_Transmit(hi2c, BMP280_I2C_ADDRESS, config, 2, HAL_MAX_DELAY);
}

void BMP280_ReadCalibrationData(I2C_HandleTypeDef *hi2c, BMP280_CalibrationData *calib_data) {
    uint8_t calib_raw[24];

    // Read 24 bytes of calibration data starting at register 0x88
    HAL_I2C_Mem_Read(hi2c, BMP280_I2C_ADDRESS, 0x88, 1, calib_raw, 24, HAL_MAX_DELAY);

    // Parse calibration data
    calib_data->dig_T1 = (uint16_t)(calib_raw[1] << 8 | calib_raw[0]);
    calib_data->dig_T2 = (int16_t)(calib_raw[3] << 8 | calib_raw[2]);
    calib_data->dig_T3 = (int16_t)(calib_raw[5] << 8 | calib_raw[4]);
}

float BMP280_ReadTemperature(I2C_HandleTypeDef *hi2c, BMP280_CalibrationData *calib_data) {
    uint8_t temp_raw[3];
    int32_t temp_adc;
    float var1, var2, temperature;

    // Read 3 bytes of temperature data starting from register 0xFA
    HAL_I2C_Mem_Read(hi2c, BMP280_I2C_ADDRESS, 0xFA, 1, temp_raw, 3, HAL_MAX_DELAY);

    // Combine bytes into 20-bit raw temperature value
    temp_adc = ((int32_t)(temp_raw[0] << 12)) | ((int32_t)(temp_raw[1] << 4)) | ((temp_raw[2] >> 4));

    // Compensate temperature using calibration data
    var1 = (((float)temp_adc) / 16384.0 - ((float)calib_data->dig_T1) / 1024.0) * ((float)calib_data->dig_T2);
    var2 = ((((float)temp_adc) / 131072.0 - ((float)calib_data->dig_T1) / 8192.0) * (((float)temp_adc) / 131072.0 - ((float)calib_data->dig_T1) / 8192.0)) * ((float)calib_data->dig_T3);
    temperature = (var1 + var2) / 5120.0;

    return temperature;
}
