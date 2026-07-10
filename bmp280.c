#include "bmp280.h"
#include "delay.h"
#include "i2c.h"
#include <math.h>

typedef struct
{
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

} BMP280_CalibData;

static BMP280_CalibData calib;

static int32_t t_fine; // t_fine = var1+ var2

static int32_t adc_P;
static int32_t adc_T;

//3 ham sau la 3 cong cu giao tiep i2c
static BMP280_StatusTypeDef BMP280_WriteReg(uint8_t reg, uint8_t data)
{
    if (i2c_mem_write(BMP280_ADDR, reg, &data, 1) != I2C_OK)
    {
        return BMP280_ERROR;
    }

    return BMP280_OK;
}

static BMP280_StatusTypeDef BMP280_ReadReg(uint8_t reg, uint8_t *data)
{
    if (i2c_mem_read(BMP280_ADDR, reg, data, 1) != I2C_OK)
    {
        return BMP280_ERROR;
    }

    return BMP280_OK;
}

static BMP280_StatusTypeDef BMP280_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    if (i2c_mem_read(BMP280_ADDR, reg, data, len) != I2C_OK)
    {
        return BMP280_ERROR;
    }

    return BMP280_OK;
}

static BMP280_StatusTypeDef BMP280_CheckID(void)
{
	uint8_t BMP280_ID;
	if (BMP280_ReadReg(BMP280_REG_ID, &BMP280_ID) != BMP280_OK)
	{
		return BMP280_ERROR;
	}
	if (BMP280_ID == BMP280_CHIP_ID)
    	{
        	return BMP280_OK;
    	}

    	return BMP280_ERROR;
}

static BMP280_StatusTypeDef BMP280_Reset(void)
{
	if (BMP280_WriteReg(BMP280_REG_RESET, 0XB6) != BMP280_OK)
	{
		return BMP280_ERROR;
	}
	return BMP280_OK;
}

static BMP280_StatusTypeDef BMP280_ReadCalibration(void)
{
    uint8_t buffer[24];
    if (BMP280_ReadRegs(BMP280_REG_DIG_T1, buffer, 24) != BMP280_OK)
    {
        return BMP280_ERROR;
    }
    calib.dig_T1 = (uint16_t)(((uint16_t)buffer[1]  << 8) | buffer[0]);
    calib.dig_T2 = (int16_t) (((uint16_t)buffer[3]  << 8) | buffer[2]);
    calib.dig_T3 = (int16_t) (((uint16_t)buffer[5]  << 8) | buffer[4]);
    calib.dig_P1 = (uint16_t)(((uint16_t)buffer[7]  << 8) | buffer[6]);
    calib.dig_P2 = (int16_t) (((uint16_t)buffer[9]  << 8) | buffer[8]);
    calib.dig_P3 = (int16_t) (((uint16_t)buffer[11] << 8) | buffer[10]);
    calib.dig_P4 = (int16_t) (((uint16_t)buffer[13] << 8) | buffer[12]);
    calib.dig_P5 = (int16_t) (((uint16_t)buffer[15] << 8) | buffer[14]);
    calib.dig_P6 = (int16_t) (((uint16_t)buffer[17] << 8) | buffer[16]);
    calib.dig_P7 = (int16_t) (((uint16_t)buffer[19] << 8) | buffer[18]);
    calib.dig_P8 = (int16_t) (((uint16_t)buffer[21] << 8) | buffer[20]);
    calib.dig_P9 = (int16_t) (((uint16_t)buffer[23] << 8) | buffer[22]);
    return BMP280_OK;
}

static BMP280_StatusTypeDef BMP280_SetCtrlMeas(void)
{
	if (BMP280_WriteReg(BMP280_REG_CTRL_MEAS, 0x2F) != BMP280_OK) // 00101111 tempx1, pressx4, normal
	{
		return BMP280_ERROR;
	}
	return BMP280_OK;
}

static BMP280_StatusTypeDef BMP280_SetConfig(void)
{
	if (BMP280_WriteReg(BMP280_REG_CONFIG, 0x08) != BMP280_OK) // 000 010 00 tsb=0,5ms, filter=4, spi off
	{
		return BMP280_ERROR;
	}
	return BMP280_OK;
}

BMP280_StatusTypeDef BMP280_Init(void)
{
	i2c_init();

	if (BMP280_CheckID() != BMP280_OK)
	{
		return BMP280_ERROR;
	}
	if (BMP280_Reset() != BMP280_OK)
	{
		return BMP280_ERROR;
	}

	delay_ms(2);

	if (BMP280_ReadCalibration() != BMP280_OK)
	{
		return BMP280_ERROR;
	}
    if (BMP280_SetConfig() != BMP280_OK)
	{
		return BMP280_ERROR;
	}
	if (BMP280_SetCtrlMeas() != BMP280_OK)
	{
		return BMP280_ERROR;
	}
	return BMP280_OK;
}

static BMP280_StatusTypeDef BMP280_ReadRawData(void)
{
	uint8_t rawData[6];
	if (BMP280_ReadRegs(BMP280_REG_PRESS_MSB, rawData, 6) != BMP280_OK)
	{
		return BMP280_ERROR;
	}
	adc_P= ((int32_t)rawData[0] << 12) | ((int32_t)rawData[1] << 4) | ((int32_t)rawData[2] >> 4);
	adc_T= ((int32_t)rawData[3] << 12) | ((int32_t)rawData[4] << 4) | ((int32_t)rawData[5] >> 4);
	return BMP280_OK;
}

static int32_t BMP280_CompensateTemperature(void)
{
    int32_t var1;
    int32_t var2;
    int32_t Final_Temp;

    var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1)) * ((int32_t)calib.dig_T2)) >> 11);

    var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    Final_Temp = (t_fine * 5 + 128) >> 8;

    return Final_Temp;
}

static uint32_t BMP280_CompensatePressure(void)
{
    int64_t var1;
    int64_t var2;
    int64_t Final_Press;

    var1 = ((int64_t)t_fine) - 128000;

    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);

    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);

    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)calib.dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0;
    }

    Final_Press = 1048576 - adc_P;
    Final_Press = (((Final_Press << 31) - var2) * 3125) / var1;

    var1 = (((int64_t)calib.dig_P9) * (Final_Press >> 13) * (Final_Press >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * Final_Press) >> 19;

    Final_Press = ((Final_Press + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);

    return (uint32_t)Final_Press;
}

static float P0 = 0;
static uint8_t altitude_initialized = 0;

BMP280_StatusTypeDef BMP280_SetReferencePressure(void)
{
    if (altitude_initialized)
    {
	return BMP280_OK;
    }

    if (BMP280_ReadRawData() != BMP280_OK)
    {
	return BMP280_ERROR;
    }

    BMP280_CompensateTemperature();
    P0 = BMP280_CompensatePressure() / 256.0f;
    altitude_initialized = 1;
	return BMP280_OK;
}

BMP280_StatusTypeDef BMP280_ReadTemperatureAndPressureAndAltitude( float *temperature, float *pressure, float *altitude)
{
    if (BMP280_ReadRawData() != BMP280_OK)
    {
	return BMP280_ERROR;
    }

    *temperature = BMP280_CompensateTemperature() / 100.0f;

    *pressure    = BMP280_CompensatePressure() / 256.0f;

    if (!altitude_initialized || P0 == 0)
    {
        *altitude = 0;
        return BMP280_OK;
    }

    *altitude = 44330.0f * (1.0f - powf(*pressure / P0, 0.1903f));

    return BMP280_OK;
}
