#include "bmp280.h"
#include "delay.h"
#include "clock.h"

volatile float temperature = 0.0f;
volatile float pressure = 0.0f;
volatile float altitude = 0.0f;

int main(void)
{
    clock_init();
    delay_init(RCC_SYS_CLOCK_HZ);
    BMP280_Init();
    delay_ms(20000);
    BMP280_SetReferencePressure();
    while (1)
    {
        float current_temperature = 0.0f;
        float current_pressure = 0.0f;
        float current_altitude = 0.0f;

        BMP280_ReadTemperatureAndPressureAndAltitude(&current_temperature, &current_pressure, &current_altitude);
        
        temperature = current_temperature;
        pressure = current_pressure;
        altitude = current_altitude;

        delay_ms(200);
    }
}
