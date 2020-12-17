
/*
 * NSSL EFM Rotating Module Firmware
 * 
 * This board and firmware read the electric field, temperature/humidity,
 * and orientation information and reports it via the fiber optic link to
 * the control paddle electronics.
 */

#include <Arduino.h>
#include <Wire.h>
#include "Lgeo_HTU21D.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "ADS1220.h"
#include "pins.h"
#include "utility/imumaths.h"
#include <SoftwareSerial.h>

Lgeo_HTU21D htu = Lgeo_HTU21D();
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
ADS1220 adc;
SoftwareSerial debugSerial(PB12, PB13); // RX, TX

void setup()
{
  // Setup serial communications
  Serial.begin(115200);
  debugSerial.begin(19200);

  debugSerial.println("NSSL Rotating Electronics");

  // Setup the IMU
  if(!bno.begin())
  {
    debugSerial.println("Error starting BNO055 IMU.");
  }

  // Setup the Temperature/Humidity sensor
  if(!htu.begin())
  {
    debugSerial.println("Error starting HTU21D sensor.");
  }

  // Setup the ADC
  adc.begin(PIN_ADC_CS, PIN_ADC_DRDY);
  // Set to normal operating mode
  adc.setOpMode(0);
  // Set to continuous conversion mode
  adc.setConversionMode(1);
  // Set to 20 Hz data rate
  adc.setDataRate(0x00);
  // Use the internal 2.048V reference
  adc.setVoltageRef(0);
  // Turn off the FIR filters
  adc.setFIR(0);
  // Set gain to 1 (000)
  adc.setGain(1);
  // AIN1 is positive signal, AIN2 is negative signal
  adc.setMultiplexer(0x03);
  // Disable the PGA
  adc.setPGAbypass(1);
}


void serialWriteFloat(float val)
{
  // Write a floating point value out as binary
  byte * b = (byte *) &val;
  Serial.write(b, 4);
}

void serialWrite32(uint32_t data)
{
  byte buf[4];
  buf[0] = data & 255;
  buf[1] = (data >> 8) & 255;
  buf[2] = (data >> 16) & 255;
  buf[3] = (data >> 24) & 255;
  Serial.write(buf, sizeof(buf));
}

void serialWrite16(uint16_t data)
{
  byte buf[2];
  buf[0] = data & 255;
  buf[1] = (data >> 8) & 255;
  Serial.write(buf, sizeof(buf));
}

void readandsend()
{
  static uint8_t loop_counter = 0;
  static uint16_t temperature_degC = 0;
  static uint16_t relative_humidity = 0;

  while (!adc.isDataReady()){} // Spin until we have new data

  // Get the time closest to when we have data ready
  uint32_t adc_ready_time = millis();

  // Read the IMU as close to the ADC time as possible
  sensors_event_t accelerometer_data, magnetometer_data, gyroscope_data;

  // Get the VECTOR_ACCELEROMETER data (m/s/s)
  bno.getEvent(&accelerometer_data, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Get the VECTOR_MAGNETOMETER data (uT)
  bno.getEvent(&magnetometer_data, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Get the VECTOR_GYROSCOPE data(rad/s)
  bno.getEvent(&gyroscope_data, Adafruit_BNO055::VECTOR_GYROSCOPE);  

  delay(1);

  // Read the ADC
  uint32_t adc_reading = adc.readADC();

  // Read the temperature or humidity - this is a very primitive scheduler that keeps things
  // fast enough that we can sample as fast as we want.
  if (loop_counter == 0)
  {
    htu.startTemperatureConversion();
  }
  if (loop_counter == 50)
  { 
    htu.startHumidityConversion();
  }

  if (loop_counter == 25)
  {
    temperature_degC = htu.readTemperature();
  }
  if (loop_counter == 75)
  {  
    relative_humidity = htu.readHumidity();
  }
 
  
  delay(1); // Without the delay we don't have populated things for sending
  
  // Send the data (use write for binary) total of 50 bytes
  debugSerial.println("Sending new packet");
  Serial.write(0xBE);  // Packet Byte 0
  serialWrite32(adc_ready_time);  // Packet Byte 1-4
  serialWrite32(adc_reading);  // Packet Byte 5-8
  serialWrite16(temperature_degC);  // Packet Byte 9-10
  serialWrite16(relative_humidity); // Packet Byte 11-12
  serialWriteFloat(accelerometer_data.acceleration.x); // Packet Byte 13-16
  serialWriteFloat(accelerometer_data.acceleration.y); // Packet Byte 17-20
  serialWriteFloat(accelerometer_data.acceleration.z); // Packet Byte 21-24
  serialWriteFloat(magnetometer_data.magnetic.x); // Packet Byte 25-28
  serialWriteFloat(magnetometer_data.magnetic.y); // Packet Byte 29-32
  serialWriteFloat(magnetometer_data.magnetic.z); // Packet Byte 33-36
  serialWriteFloat(gyroscope_data.gyro.x); // Packet Byte 37-40
  serialWriteFloat(gyroscope_data.gyro.y); // Packet Byte 41-44
  serialWriteFloat(gyroscope_data.gyro.z); // Packet Byte 45-48
  Serial.write(0xEF); // Packet Byte 49

  loop_counter += 1;
  if (loop_counter == 100)
  {
    loop_counter = 0;
  }
}


void loop()
{
  // Really quite simple - just keep reading and sending as fast as we can!
  readandsend();
}
