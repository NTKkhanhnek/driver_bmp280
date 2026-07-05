/*
 * bmp280.h
 *
 *  Created on: Jul 3, 2026
 *      Author: PC
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "stm32f4xx_hal.h" // chua cau truc, trang thai
#include <stdint.h> // thu vien chuan, chua cac kieu du lieu
#include <math.h>

#define BMP280_ADDR_LOW		(0x76<<1) // dia chi i2c
#define BMP280_ADDR_HIGH 	(0x77<<1) // dia chi i2c

#define BMP280_ADDR BMP280_ADDR_LOW // chon dia chi i2c

#define BMP280_REG_ID 		 0xD0 // kiem tra chip
#define BMP280_CHIP_ID 		 0x58

#define BMP280_REG_RESET     	 0xE0
#define BMP280_REG_STATUS    	 0xF3
#define BMP280_REG_CTRL_MEAS 	 0xF4
#define BMP280_REG_CONFIG    	 0xF5
#define BMP280_REG_PRESS_MSB 	 0xF7
#define BMP280_REG_PRESS_LSB 	 0xF8
#define BMP280_REG_PRESS_XLSB	 0xF9
#define BMP280_REG_TEMP_MSB  	 0xFA
#define BMP280_REG_TEMP_LSB  	 0xFB
#define BMP280_REG_TEMP_XLSB  	 0xFC

#define BMP280_RESET_VALUE    	 0xB6

#define BMP280_REG_DIG_T1 	 0x88 // he so sua loi, xu ly du lieu tho
#define BMP280_REG_DIG_T2	 0x8A
#define BMP280_REG_DIG_T3	 0x8C
#define BMP280_REG_DIG_P1	 0x8E
#define BMP280_REG_DIG_P2	 0x90
#define BMP280_REG_DIG_P3	 0x92
#define BMP280_REG_DIG_P4	 0x94
#define BMP280_REG_DIG_P5	 0x96
#define BMP280_REG_DIG_P6	 0x98
#define BMP280_REG_DIG_P7	 0x9A
#define BMP280_REG_DIG_P8	 0x9C
#define BMP280_REG_DIG_P9	 0x9E

HAL_StatusTypeDef BMP280_Init(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef BMP280_Altitude_Init(void);

HAL_StatusTypeDef BMP280_ReadTemperatureAndPressureAndAltitude( float *temperature, float *pressure, float *altitude);

#endif /* INC_BMP280_H_ */
