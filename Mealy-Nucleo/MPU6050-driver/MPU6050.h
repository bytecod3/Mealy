/*
*
* STM32 MPU6050 I2C driver
* Author: Edwin Mwiti
*
*/

#ifndef MPU6050_H
#define MPU6050_H

/* todo: STM32 essential libraries includes  */

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
#define INT_ENABLE
#define INT_STATUS
#define ACCEL_XOUT_H
#define ACCEL_XOUT_L
#define ACCEL_YOUT_H
#define ACCEL_YOUT_L
#define ACCEL_ZOUT_H
#define ACCEL_ZOUT_L
#define TEMP_OUT_H
#define TEMP_OUT_L
#define GYRO_XOUT_H
#define GYRO_XOUT_L
#define 

/* device init struct  */
typedef struct {
} MPU6050;

/* todo: Driver function prototypes */

/* todo: Register read function prototypes  */

#endif // end MPU6050_H
