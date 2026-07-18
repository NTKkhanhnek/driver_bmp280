#include "bmp280.h"
#include "delay.h"
#include "clock.h"

volatile float temperature = 0.0f;
volatile float pressure = 0.0f;
volatile float altitude = 0.0f;

volatile BMP280_StatusTypeDef bmp280_status = BMP280_ERROR;

static void app_error_handler(void)
{
    while (1)
    {
    }
}

int main(void)
{
    rcc_init();
    delay_init(RCC_SYS_CLOCK_HZ);

    bmp280_status = BMP280_Init();
    if (bmp280_status != BMP280_OK)
    {
        app_error_handler();
    }

    bmp280_status = BMP280_SetReferencePressure();
    if (bmp280_status != BMP280_OK)
    {
        app_error_handler();
    }
    delay_ms(2000);
    while (1)
    {
        float current_temperature = 0.0f;
        float current_pressure = 0.0f;
        float current_altitude = 0.0f;

        bmp280_status = BMP280_ReadTemperatureAndPressureAndAltitude(&current_temperature, &current_pressure, &current_altitude);
        if (bmp280_status != BMP280_OK)
        {
            app_error_handler();
        }

        temperature = current_temperature;
        pressure = current_pressure;
        altitude = current_altitude;

        delay_ms(200);
    }
}
