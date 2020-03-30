/*
 * Library to allow non-blocking reads of the HTU21D Sensor
 * 
 */

#include "Lgeo_HTU21D.h"

Lgeo_HTU21D::Lgeo_HTU21D()
{
    // Constructor if we need it
}

char Lgeo_HTU21D::begin(void)
{
    Wire.begin();
    reset();
    Wire.beginTransmission(HTU21D_ADDR);
    Wire.write(HTU21D_READ_REGISTER);
    Wire.endTransmission();
    Wire.requestFrom(HTU21D_ADDR, 1);
    return (Wire.read() == 0x2);  // If we get the ID, we're good!
}

void Lgeo_HTU21D::reset(void)
{
    Wire.beginTransmission(HTU21D_ADDR);
    Wire.write(HTU21D_RESET);
    Wire.endTransmission();
    delay(15);
}

char Lgeo_HTU21D::startTemperatureConversion(void)
{
    Wire.beginTransmission(HTU21D_ADDR);
    Wire.write(HTU21D_TRIGGER_TEMPERATURE);
    Wire.endTransmission();
    return 1;
}

char Lgeo_HTU21D::startHumidityConversion(void)
{
    Wire.beginTransmission(HTU21D_ADDR);
    Wire.write(HTU21D_TRIGGER_HUMIDITY);
    Wire.endTransmission();
    return 1;
}

uint16_t Lgeo_HTU21D::readTemperature(void)
{
    uint8_t count = Wire.requestFrom(HTU21D_ADDR, 3);

    /* Make sure we got 3 bytes back. */
    if (count != 3) {
        return 0;
    }

    /* Read 16 bits of data, dropping the last two status bits. */
    uint16_t t = Wire.read();
    t <<= 8;
    t |= Wire.read() & 0b11111100;

    Wire.read();
    //uint8_t crc = Wire.read();
    //(void)crc;

    //float temperature = t;
    //temperature *= 175.72f;
    //temperature /= 65536.0f;
    //temperature -= 46.85f;

    return t;
}

uint16_t Lgeo_HTU21D::readHumidity(void)
{
    /* Read the conversion results. */
    uint8_t count = Wire.requestFrom(HTU21D_ADDR, 3);

    /* Make sure we got 3 bytes back. */
    if (count != 3) {
        return 0;
    }

    /* Read 16 bits of data, dropping the last two status bits. */
    uint16_t h = Wire.read();
    h <<= 8;
    h |= Wire.read() & 0b11111100;

    uint8_t crc = Wire.read();
    (void)crc;

    //float hum = h;
    //hum *= 125.0f;
    //hum /= 65536.0f;
    //hum -= 6.0f;

    return h;
}