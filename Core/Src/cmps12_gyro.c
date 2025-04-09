/*
 * cmps12_gyro.c
 *
 *  Created on: Oct 11, 2024
 *      Author: zalv
 */

#include "main.h"
#include "i2c.h"
#include "gpio.h"
#include "cmps12_gyro.h"
#include "math.h"
float _gyroScale = 1.0f/16.f; // 1 Dps = 16 LSB
float _accelScale = 1.0f/100.f; // 1 m/s^2 = 100 LSB
float deltaTime = 0.01f;
extern float gyroOffsetZ;
extern int cnt;
#define CALIBRATION_SAMPLES 1000
#define DEG_TO_RAD (M_PI/180.0f)
#define CMG_TO_MS2 (9.81f/100);

// float integrateGyro(float currentAngle)
// {
//     return currentAngle gyroDps * deltaTime);
// }

float convertGyroToRad(float gyro_dps) {
    return gyro_dps * DEG_TO_RAD;
}

float convertAccelToMs2(float accel_centi_g) {
    return accel_centi_g * CMG_TO_MS2;
}

void calibrateGyro(){
    int32_t gyroZ = 0;
    int32_t gyroZSum = 0;

    for (cnt = 0; cnt < CALIBRATION_SAMPLES; cnt++)
    {
        gyroZ = getGyroZ();
        gyroZSum += gyroZ;
        HAL_Delay(1);
    }

    gyroOffsetZ = (float)gyroZSum / (float)CALIBRATION_SAMPLES;
}

float integrateGyro(int16_t gyro_dps){
	return (float)gyro_dps*0.01f;
}

uint8_t getCalibrationState()
{
    uint8_t reg = CALLIB_GYRO;
    uint8_t calibrationState;

    if (HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDRESS << 1, &reg, ONE_BYTE, 100) != HAL_OK)
    {
        return 0;
    }

    if (HAL_I2C_Master_Receive(&hi2c3, I2C_ADDRESS << 1, &calibrationState, ONE_BYTE, 100) != HAL_OK)
    {
        return 0;
    }

    return calibrationState;
}

int16_t getGyroX()
{
    uint8_t data[2];
    uint8_t reg = REGISTER_GYRO_X;
    int16_t gyroX;

    if (HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDRESS << 1, &reg, ONE_BYTE, 100) != HAL_OK)
    {
        return 0;
    }

    if (HAL_I2C_Master_Receive(&hi2c3, I2C_ADDRESS << 1, data, TWO_BYTES, 100) != HAL_OK)
    {
        return 0;
    }

    gyroX = (int16_t)(data[0] << 8 | data[1]);

    return gyroX * _gyroScale;
}

int16_t getGyroY()
{
    uint8_t data[2];
    uint8_t reg = REGISTER_GYRO_Y;
    int16_t gyroY;

    if (HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDRESS << 1, &reg, ONE_BYTE, 100) != HAL_OK)
    {
        return 0;
    }

    if (HAL_I2C_Master_Receive(&hi2c3, I2C_ADDRESS << 1, data, TWO_BYTES, 100) != HAL_OK)
    {
        return 0;
    }

    gyroY = (int16_t)(data[0] << 8 | data[1]);

    return gyroY * _gyroScale;
}

int16_t getGyroZ()
{
    uint8_t data[2];
    uint8_t reg = REGISTER_GYRO_Z;
    int16_t gyroZ;

    if (HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDRESS << 1, &reg, ONE_BYTE, 100) != HAL_OK)
    {
        return 0;
    }

    if (HAL_I2C_Master_Receive(&hi2c3, I2C_ADDRESS << 1, data, TWO_BYTES, 50) != HAL_OK)
    {
        return 0;
    }

    gyroZ = (int16_t)(data[0] << 8 | data[1]);

    // gyroZ = gyroZ / 16;

    return gyroZ * _gyroScale - gyroOffsetZ;
}

int16_t getAcceleroX()
{
    uint8_t data[2];
    uint8_t reg = ACCELERO_X_Register;
    int16_t accelX;

    if (HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDRESS << 1, &reg, ONE_BYTE, 50) != HAL_OK)
    {
        return 0;
    }

    if (HAL_I2C_Master_Receive(&hi2c3, I2C_ADDRESS << 1, data, TWO_BYTES, 50) != HAL_OK)
    {
        return 0;
    }

    accelX = (int16_t)(data[0] << 8 | data[1]);

    return accelX;
}

int16_t getAcceleroY()
{
    uint8_t data[2];
    uint8_t reg = ACCELERO_Y_Register;
    int16_t accelY;

    if (HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDRESS << 1, &reg, ONE_BYTE, 50) != HAL_OK)
    {
        return 0;
    }

    if (HAL_I2C_Master_Receive(&hi2c3, I2C_ADDRESS << 1, data, TWO_BYTES, 50) != HAL_OK)
    {
        return 0;
    }

    accelY = (int16_t)(data[0] << 8 | data[1]);

    return accelY;
}

int16_t getAcceleroZ()
{
    uint8_t data[2];
    uint8_t reg = ACCELERO_Z_Register;
    int16_t accelZ;

    if (HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDRESS << 1, &reg, ONE_BYTE, 50) != HAL_OK)
    {
        return 0;
    }

    if (HAL_I2C_Master_Receive(&hi2c3, I2C_ADDRESS << 1, data, TWO_BYTES, 50) != HAL_OK)
    {
        return 0;
    }

    accelZ = (int16_t)(data[0] << 8 | data[1]);

    return accelZ;
}

int16_t getMagnetX()
{
    uint8_t data[2];
    uint8_t reg = MAGNET_X_Register;
    int16_t magnetX;

    if (HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDRESS << 1, &reg, ONE_BYTE, 50) != HAL_OK)
    {
        return 0;
    }

    if (HAL_I2C_Master_Receive(&hi2c3, I2C_ADDRESS << 1, data, TWO_BYTES, 50) != HAL_OK)
    {
        return 0;
    }

    magnetX = (int16_t)(data[0] << 8 | data[1]);

    return magnetX;
}

int16_t getMagnetY()
{
    uint8_t data[2];
    uint8_t reg = MAGNET_Y_Register;
    int16_t magnetY;

    if (HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDRESS << 1, &reg, ONE_BYTE, 50) != HAL_OK)
    {
        return 0;
    }

    if (HAL_I2C_Master_Receive(&hi2c3, I2C_ADDRESS << 1, data, TWO_BYTES, 50) != HAL_OK)
    {
        return 0;
    }

    magnetY = (int16_t)(data[0] << 8 | data[1]);

    return magnetY;
}

int16_t getMagnetZ()
{
    uint8_t data[2];
    uint8_t reg = MAGNET_Z_Register;
    int16_t magnetZ;

    if (HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDRESS << 1, &reg, ONE_BYTE, 50) != HAL_OK)
    {
        return 0;
    }

    if (HAL_I2C_Master_Receive(&hi2c3, I2C_ADDRESS << 1, data, TWO_BYTES, 50) != HAL_OK)
    {
        return 0;
    }

    magnetZ = (int16_t)(data[0] << 8 | data[1]);

    return magnetZ;
}

int16_t getBearing()
{
    uint8_t data[2];
    uint8_t reg = BEARING_Register;
    int16_t bearing;

    if (HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDRESS << 1, &reg, ONE_BYTE, 50) != HAL_OK)
    {
        return 0;
    }

    if (HAL_I2C_Master_Receive(&hi2c3, I2C_ADDRESS << 1, data, TWO_BYTES, 50) != HAL_OK)
    {
        return 0;
    }

    bearing = (int16_t)(data[0] << 8 | data[1]) / 10;

    return bearing;
}

uint8_t getPitch()
{
    uint8_t data[1];
    uint8_t reg = PITCH_Register;
    uint8_t pitch;

    if (HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDRESS << 1, &reg, ONE_BYTE, 50) != HAL_OK)
    {
        return 0;
    }

    if (HAL_I2C_Master_Receive(&hi2c3, I2C_ADDRESS << 1, data, ONE_BYTE, 50) != HAL_OK)
    {
        return 0;
    }

    pitch = data[0];

    return pitch;
}

uint8_t getRoll()
{
    uint8_t data[1];
    uint8_t reg = ROLL_Register;
    uint8_t roll;

    if (HAL_I2C_Master_Transmit(&hi2c3, I2C_ADDRESS << 1, &reg, ONE_BYTE, 50) != HAL_OK)
    {
        return 0;
    }

    if (HAL_I2C_Master_Receive(&hi2c3, I2C_ADDRESS << 1, data, ONE_BYTE, 50) != HAL_OK)
    {
        return 0;
    }

    roll = data[0];

    return roll;
}

void changeAddress(uint8_t i2cAddress, uint8_t newi2cAddress)
{
    uint8_t data[2];
    data[0] = CONTROL_Register;
    data[1] = 0xA0;

    if (HAL_I2C_Master_Transmit(&hi2c3, i2cAddress << 1, data, 2, 50) != HAL_OK)
    {
        return;
    }

    HAL_Delay(100);

    data[1] = 0xAA;
    if (HAL_I2C_Master_Transmit(&hi2c3, i2cAddress << 1, data, 2, 50) != HAL_OK)
    {
        return;
    }
    HAL_Delay(100);

    data[1] = 0xA5;
    if (HAL_I2C_Master_Transmit(&hi2c3, i2cAddress << 1, data, 2, 50) != HAL_OK)
    {
        return;
    }

    HAL_Delay(100);

    data[1] = newi2cAddress;
    if (HAL_I2C_Master_Transmit(&hi2c3, i2cAddress << 1, data, 2, 50) != HAL_OK)
    {
        return;
    }
}
