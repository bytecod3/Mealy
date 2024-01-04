/*
*
* STM32 MPU6050 I2C driver
* Author: Edwin Mwiti
*
*/

#ifndef MPU6050_H
#define MPU6050_H

/* STM32 essential libraries includes  */
#include "stm32f1xx_hal.h"

/* Register addresses */
#define SELF_TEST_X			0x0D
#define SELF_TEST_Y			0x0E
#define SELF_TEST_Z			0x0F
#define SELF_TEST_A			0x10
#define SMPLTR_DIV			0x19
#define CONFIG				0x1A
#define GYRO_CONFIG			0x1B
#define ACCEL_CONFIG		0x1C
#define FIFO_EN				0x23
#define I2C_MST_CTRL		0x24
#define I2C_SLV0_ADDR		0x25
#define I2C_SLV0_REG		0x26
#define I2C_SLV0_CTRL		0x27
#define I2C_SLV1_ADDR		0x28
#define I2C_SLV1_REG		0x29
#define I2C_SLV1_CTRL		0x2A
#define I2C_SLV2_ADDR		0x2B
#define I2C_SLV2_REG		0x2C
#define I2C_SLV2_CTRL		0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60
#define I2C_SLV0_D0         0x63
#define I2C_SLV1_D0         0x64
#define I2C_SLV2_D0         0x65
#define I2C_SLV3_D0         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define USER_CTRL           0x6A
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C
#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I            0x75
#define MPU6050_I2C_ADDR    0xD0 // 0x68 shifted left 7-bit address - bit 0 is set can be 0 or 1
#define MPU6050_DEVICE_ID	0x68

/* device init struct  */
typedef struct {
    /* i2c handle */
    I2C_HandleTypeDef* i2cHandle;

    /* accelerometer data (x, y, z) */
    float acc_mps2[3]; // accelerometer is in meters per second

    /* gyroscope data */
    float gyro_data[3]; // gyro values in deg

    /* temperature data in deg */
    float temp_C;

} MPU6050;

// Initialization function
uint8_t MPU6050_Initialise(MPU6050* dev, I2C_HandleTypeDef* i2cHandle);

// data acquisition functions
HAL_StatusTypeDef MPU6050_ReadTemperature(MPU6050* dev);
void MPU6050_ReadTemperatureTest(MPU6050* dev);
void MPU6050_ReadAcceleration(MPU6050* dev);
void MPU6050_ReadGyroscope(MPU6050* dev);

/* Register read function prototypes  */
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050* dev, uint8_t reg, uint8_t* data);
HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050* dev, uint8_t reg, uint8_t* data, uint8_t length);
HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050* dev, uint8_t reg, uint8_t* data);

#endif // end MPU6050_H
