/*
 * mpu6050.h
 *
 *  Created on: 09/03/2015
 *      Author: Victor
 */

#ifndef I2C_MPU6050_MPU6050_H_
#define I2C_MPU6050_MPU6050_H_

#include <inttypes.h>

#define MPU6050_ADDRESS       		0x68

// Registradores
#define MPU6050_RA_WHO_AM_I    		0x75
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG    	0x1C
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

// BITS
#define MPU6050_WHO_AM_I_BIT       			6

#define MPU6050_PWR1_CLKSEL_BIT     		2
#define MPU6050_PWR1_CLKSEL_LENGTH  		3

#define MPU6050_GCONFIG_FS_SEL_BIT      	4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   	2

#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3


// PWR_MGMT_1 - clock source setting
#define MPU6050_CLOCK_INTERNAL    	0x00
#define MPU6050_CLOCK_PLL_XGYRO     0x01
#define MPU6050_CLOCK_PLL_YGYRO    	0x02
#define MPU6050_CLOCK_PLL_ZGYRO    	0x03
#define MPU6050_CLOCK_PLL_EXT32K   	0x04
#define MPU6050_CLOCK_PLL_EXT19M   	0x05
#define MPU6050_CLOCK_KEEP_RESET   	0x07

// GYRO_CONFIG - scale gyroscope range
#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

// ACCEL_CONFIG - scale accelerometer range
#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03




uint8_t mpu6050_detect(void); 		// Verificar a presença do MPU6050
void mpu6050_config(void);			// Calibração do MPU6050
void mpu6050_getAcceleration(int16_t*, int16_t*, int16_t*);
void mpu6050_getRotation(int16_t*, int16_t*, int16_t*);
void mpu6050_getTemperature(int16_t*);


#endif /* I2C_MPU6050_MPU6050_H_ */
