#include "MPU6050.h"
#include <stdio.h>

// initialisation function


/**
 * Initialise the MPU6050 device
 * @param dev device struct
 * @param i2cHandle i2c handle from stm32's I2C
 * @return
 */
uint8_t MPU6050_Initialise(MPU6050* dev, I2C_HandleTypeDef* i2cHandle){
    dev->i2cHandle = i2cHandle;

    // zero out initial data
    dev->acc_mps2[0] = 0.0;
    dev->acc_mps2[1] = 0.0;
    dev->acc_mps2[2] = 0.0;

    // to hold init status
    uint8_t status;

    // check device ID
    uint8_t reg_data;

    //MPU6050_ReadRegister(dev, WHO_AM_I, &reg_data); // datasheet page 45
    HAL_I2C_Mem_Read(i2cHandle, MPU6050_I2C_ADDR, WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 1);

    if(reg_data == 0x68){
    	printf("WHO AM I: 0x%02x..", reg_data);

    	// wake up the device, set the internal oscillator to 8MHz
    	reg_data = 0x00;
    	MPU6050_WriteRegister(dev, PWR_MGMT_1, &reg_data);

    	// set the sample rate to 1kHz by dividing gyro rate by 8
    	reg_data = 0x07;
    	MPU6050_WriteRegister(dev, SMPLTR_DIV, &reg_data);

    	// configure gyroscope to max +- 250 deg/s
    	reg_data = 0x00;
    	MPU6050_WriteRegister(dev, GYRO_CONFIG, &reg_data);

    	// configure the accelerometer to max +- 16g
    	reg_data = 0x18;
    	MPU6050_WriteRegister(dev, ACCEL_CONFIG, &reg_data);

    	status = 0; // successful initialization
    } else {
    	status = 255; // failed to initialize mpu6050
    }

    return status;

}

/* low level functions */
/**
 * Reads register data
 * @param dev i2c device handle
 * @param reg  register to read from
 * @param data pointer to store the data that has been read
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050* dev, uint8_t reg, uint8_t* data){
    return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}

/**
 * Reads register data from many registers
 * @param dev i2c device handle
 * @param reg  register to read from
 * @param data pointer to store the data that has been read
 * @param length number of bytes to read from registers
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050* dev, uint8_t reg, uint8_t* data, uint8_t length){
    return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);

}


/**
 * Write data to registers
 * @param dev i2c device handle
 * @param reg  register to read from
 * @param data pointer to store the data that has been read
 * @return
 */
HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050* dev, uint8_t reg, uint8_t * data){
    return HAL_I2C_Mem_Write(dev->i2cHandle, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/**
 * Read temperature
 * @param dev i2c device handle
 * @return
 */
HAL_StatusTypeDef MPU6050_ReadTemperature(MPU6050* dev){
    uint8_t reg_data[2];

    //HAL_StatusTypeDef status = MPU6050_ReadRegister(dev, TEMP_OUT_H, reg_data[0]); // read MSB temp register
    //HAL_StatusTypeDef status = MPU6050_ReadRegister(dev, TEMP_OUT_L, reg_data[1]); // read LSB temp register

    MPU6050_ReadRegister(dev, TEMP_OUT_H, reg_data[0]); // read MSB temp register
    MPU6050_ReadRegister(dev, TEMP_OUT_L, reg_data[1]); // read LSB temp register

    uint16_t raw_temp; // to hold raw data temperature from the 2 registers above
    raw_temp = ( (reg_data[0] << 8 ) | reg_data[1] ); // join the two values into one 

    // convert to deg C
    // offset at 35 degC = -521, slope = 340 LSB/degC
    // TODO: use the correct forumla for temperature
    dev->temp_C = 0.0029411 * (float) ((raw_temp + 521) + 35);

    return 0; // TODO: return correct status here
}

void MPU6050_ReadTemperatureTest(MPU6050* dev){
	uint8_t reg_data[2];
	MPU6050_ReadRegister(dev, TEMP_OUT_H, reg_data[0]); // read MSB temp register
	MPU6050_ReadRegister(dev, TEMP_OUT_L, reg_data[1]); // read LSB temp register

	uint16_t raw_temp; // to hold raw data temperature from the 2 registers above
	raw_temp = ( (reg_data[0] << 8 ) | reg_data[1] ); // join the two values into one

	// convert to deg C
	// offset at 35 degC = -521, slope = 340 LSB/degC
	dev->temp_C = 0.0029411 * (float) ((raw_temp + 521) + 35);

}

/**
 * @brief read  accelerometer
 * @param dev pointer to device being read
 *
 */

void MPU6050_ReadAcceleration(MPU6050* dev){
	uint8_t data[6];

	HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_I2C_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY); // read 6 bytes from accel_h register

	// get acceleration x axis
	int16_t accel_x_raw = (int16_t) (data[0]<<8 | data[1]);

	// get y acceleration
	int16_t accel_y_raw = (int16_t) (data[2]<<8 | data[3]);

	// get z acceleration
	int16_t accel_z_raw = (int16_t) (data[4]<<8 | data[5]);

	// update the struct
	// here, the accelerometer is set for max 16g,
	// so we divide the raw value by 2048, as listed in the datasheet
	// remember to multiply this value by 9.81 to convert to m/s^2
	dev->acc_mps2[0] = accel_x_raw / 2048.0;
	dev->acc_mps2[1] = accel_y_raw / 2048.0;
	dev->acc_mps2[2] = accel_z_raw / 2048.0;

}

/**
 * @brief read gyroscope
 * @param dev pointer to device being read
 *
 */

void MPU6050_ReadGyroscope(MPU6050* dev){
	uint8_t data[6];

	HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_I2C_ADDR, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY); // read 6 bytes from accel_h register

	int16_t gyro_x_raw = (int16_t) (data[0]<<8 | data[1]);
	int16_t gyro_y_raw = (int16_t) (data[2]<<8 | data[3]);
	int16_t gyro_z_raw = (int16_t) (data[4]<<8 | data[5]);

	// update mpu 6050 struct
	// we divide by 250 to convert gyro values to deg/sec
	dev->gyro_data[0] = gyro_x_raw / 250.0; // from datasheet
	dev->gyro_data[1] = gyro_y_raw / 250.0;
	dev->gyro_data[2] = gyro_z_raw / 250.0;

}
