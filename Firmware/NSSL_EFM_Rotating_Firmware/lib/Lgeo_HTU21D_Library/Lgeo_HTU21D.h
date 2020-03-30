/*
 * Library to allow non-blocking reads of the HTU21D Sensor
 * 
 */

#ifndef _LGEO_HTU21D
#define _LGEO_HTU21D

#include "Arduino.h"
#include "Wire.h"

#define HTU21D_ADDR (0x40)
#define HTU21D_TRIGGER_TEMPERATURE (0xF3)
#define HTU21D_TRIGGER_HUMIDITY (0xF5)
#define HTU21D_READ_REGISTER (0xE7)
#define HTU21D_WRITE_REGISTER (0xE6)
#define HTU21D_RESET (0xFE)
#define HTU21D_HUMIDITY_REGISTER (0xE5)
#define HTU21D_TEMPERATURE_REGISTER (0xE3)

class Lgeo_HTU21D
{
    public:
        Lgeo_HTU21D();

        char begin(void);
        char startTemperatureConversion(void);
        char startHumidityConversion(void);
        uint16_t readTemperature(void);
        uint16_t readHumidity(void);
        void reset(void);
};

#endif