
/*
 * NSSL EFM Rotating Module Firmware
 * 
 * This board and firmware read the electric field, temperature/humidity,
 * and orientation information and reports it via the fiber optic link to
 * the control paddle electronics.
 */

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_BME280.h>
#include "ADS1220.h"
#include "pins.h"
#include "utility/imumaths.h"
#include <SoftwareSerial.h>

Adafruit_BME280 bme;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
ADS1220 adc;
SoftwareSerial debugSerial(PB12, PB13); // RX, TX

void setup()
{
  // Setup serial communications
  Serial.begin(38400);
  debugSerial.begin(19200);

  debugSerial.println("NSSL Rotating Electronics");

  // Setup the IMU
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
  if(!lsm.begin())
  {
    debugSerial.println("Error starting IMU.");
  }

  // Setup the Temperature/Humidity sensor
  if(!bme.begin(0x76, &Wire))
  {
    debugSerial.println("Error starting BME280 sensor.");
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
  static uint16_t pressure_pa = 0;

  while (!adc.isDataReady()){} // Spin until we have new data

  // Get the time closest to when we have data ready
  uint32_t adc_ready_time = millis();

  // Read the IMU as close to the ADC time as possible
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  // Read the ADC
  uint32_t adc_reading = adc.readADC();

  // Read the temperature or humidity - this is a very primitive scheduler that keeps things
  // fast enough that we can sample as fast as we want.
  if (loop_counter == 0)
  {
    relative_humidity = bme.readHumidity();
    //debugSerial.print("RH: ");
    //debugSerial.println(relative_humidity);
  }

  if (loop_counter == 25)
  { 
    pressure_pa = bme.readPressure() / 10;
    //debugSerial.print("P: ");
    //debugSerial.println(pressure_pa);
  }

  if (loop_counter == 50)
  {
    temperature_degC = bme.readTemperature() * 10;
    //debugSerial.print("T: ");
    //debugSerial.println(temperature_degC);
  }

  delay(1); // Without the delay we don't have populated things for sending
  
  // Send the data (use write for binary) total of 51 bytes
  //debugSerial.println("Sending new packet");
  //debugSerial.println(millis());
  //debugSerial.print(adc_ready_time);
  //debugSerial.print(",");
  //debugSerial.println(adc_reading);


  Serial.write(0xBE);  // Packet Byte 0
  serialWrite32(adc_ready_time);  // Packet Byte 1-4
  serialWrite32(adc_reading);  // Packet Byte 5-8
  serialWriteFloat(a.acceleration.x); // Packet Byte 9-12
  serialWriteFloat(a.acceleration.y); // Packet Byte 13-16
  serialWriteFloat(a.acceleration.z); // Packet Byte 17-20
  serialWriteFloat(m.magnetic.x); // Packet Byte 21-24
  serialWriteFloat(m.magnetic.y); // Packet Byte 25-28
  serialWriteFloat(m.magnetic.z); // Packet Byte 29-32
  serialWriteFloat(g.gyro.x); // Packet Byte 33-36
  serialWriteFloat(g.gyro.y); // Packet Byte 37-40
  serialWriteFloat(g.gyro.z); // Packet Byte 41-44
  serialWrite16(temperature_degC);  // Packet Byte 45-46
  serialWrite16(relative_humidity); // Packet Byte 47-48
  serialWrite16(pressure_pa); // Packet Byte 49-50
  Serial.write(0xEF); // Packet Byte 51
  /*
  debugSerial.print(a.acceleration.x);
  debugSerial.print("\t");
  debugSerial.print(a.acceleration.y);
  debugSerial.print("\t");
  debugSerial.print(a.acceleration.z);
  debugSerial.print("\t");
  debugSerial.print(m.magnetic.x);
  debugSerial.print("\t");
  debugSerial.print(m.magnetic.y);
  debugSerial.print("\t");
  debugSerial.print(m.magnetic.z);
  debugSerial.print("\t");
  debugSerial.print(g.gyro.x);
  debugSerial.print("\t");
  debugSerial.print(g.gyro.y);
  debugSerial.print("\t");
  debugSerial.println(g.gyro.z);
  */



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
