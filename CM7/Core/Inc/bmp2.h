// File: bmp2.h
#ifndef BMP2_H
#define BMP2_H

#include "main.h"

#define BMP280_I2C_ADDRESS (0x76 << 1) // Default I2C address (can be 0x76 or 0x77)

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
} BMP280_CalibrationData;

// Function prototypes
void BMP280_Init(I2C_HandleTypeDef *hi2c);
void BMP280_ReadCalibrationData(I2C_HandleTypeDef *hi2c, BMP280_CalibrationData *calib_data);
float BMP280_ReadTemperature(I2C_HandleTypeDef *hi2c, BMP280_CalibrationData *calib_data);

#endif // BMP2_H
