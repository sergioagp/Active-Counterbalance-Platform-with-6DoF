#ifndef __MPU6050DEF_H__
#define __MPU6050DEF_H__
#pragma once

#include <cstdint>
#define RANGE				65535
#define RESOLUTION	32768

#define	I2C					I2C1

/* Default I2C address */
#define I2C_ADDRESS				0xD0

/* Who I am register value */
#define I_AM						0x68

/* MPU6050 registers */
#define AUX_VDDIO				0x01
#define SMPLRT_DIV				0x19
#define CONFIG					0x1A
#define GYRO_CONFIG				0x1B
#define ACCEL_CONFIG			0x1C
#define MOT_THR					0x1F
#define MOT_DUR					0x20
#define ZMOT_THR				0x21
#define ZMOT_DUR				0x22
#define INT_PIN_CFG				0x37
#define INT_ENABLE				0x38
#define INT_STATUS				0x3A
#define ACCEL_XOUT_H			0x3B
#define ACCEL_XOUT_L			0x3C
#define ACCEL_YOUT_H			0x3D
#define ACCEL_YOUT_L			0x3E
#define ACCEL_ZOUT_H			0x3F
#define ACCEL_ZOUT_L			0x40
#define TEMP_OUT_H				0x41
#define TEMP_OUT_L				0x42
#define GYRO_XOUT_H				0x43
#define GYRO_XOUT_L				0x44
#define GYRO_YOUT_H				0x45
#define GYRO_YOUT_L				0x46
#define GYRO_ZOUT_H				0x47
#define GYRO_ZOUT_L				0x48
#define MOT_DETECT_STATUS		0x61
#define SIGNAL_PATH_RESET		0x68
#define MOT_DETECT_CTRL			0x69
#define USER_CTRL				0x6A
#define PWR_MGMT_1				0x6B
#define PWR_MGMT_2				0x6C
#define FIFO_COUNTH				0x72
#define FIFO_COUNTL				0x73
#define FIFO_R_W				0x74
#define WHO_AM_I				0x75

/* Gyro sensitivities in °/s */
#define GYRO_SENS_250			((float) 131)
#define GYRO_SENS_500			((float) 65.5)
#define GYRO_SENS_1000			((float) 32.8)
#define GYRO_SENS_2000			((float) 16.4)

/* Acce sensitivities in g */
#define ACCE_SENS_2				((float) 16384)
#define ACCE_SENS_4				((float) 8192)
#define ACCE_SENS_8				((float) 4096)
#define ACCE_SENS_16			((float) 2048)

#define USER_TIMEOUT			((uint32_t)0xFFFFF)

enum AccDHPF_t {
	AccDHPF_Off   =	0, 
	AccDHPF_5Hz   =	1, 
	AccDHPF_2Hz5  = 2, 
	AccDHPF_1Hz25 = 3, 
	AccDHPF_0Hz63 = 4,
	AccDHPF_Hold  = 7
};

/**
 * @brief  Parameters for accelerometer range
 */
enum Accelerometer_t {
	Accelerometer_2G  = 0x00  << 3, /*!< Range is +- 2G */
	Accelerometer_4G  = 0x01  << 3, /*!< Range is +- 4G */
	Accelerometer_8G  = 0x02  << 3, /*!< Range is +- 8G */
	Accelerometer_16G = 0x03  << 3/*!< Range is +- 16G */
};

/**
 * @brief  Parameters for gyroscope range
 */
enum Gyroscope_t {
	Gyroscope_250s	=	0x00	<<	3,  /*!< Range is +- 250 degrees/s */
	Gyroscope_500s	=	0x01	<<	3,  /*!< Range is +- 500 degrees/s */
	Gyroscope_1000s	=	0x02	<<	3,	/*!< Range is +- 1000 degrees/s */
	Gyroscope_2000s	=	0x03	<<	3 	/*!< Range is +- 2000 degrees/s */
};

struct MPU6050_t {
	std::uint8_t INT_Status;					//Interrupt Status
	std::int16_t Accelerometer_X;	// Accelerometer value X axis */
	std::int16_t Accelerometer_Y;	// Accelerometer value Y axis */
	std::int16_t Accelerometer_Z;	// Accelerometer value Z axis */
	std::int16_t Gyroscope_X;			// Gyroscope value X axis */
	std::int16_t Gyroscope_Y;     // Gyroscope value Y axis */
	std::int16_t Gyroscope_Z;     // Gyroscope value Z axis */
	float Temperature;
	
};

#endif //__MPU6050DEF_H__
