
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

  // Setup the IMU
  if(!bno.begin())
  {
    Serial.print("Error starting BNO055 IMU.");
  }

  // Setup the Temperature/Humidity sensor
  if(!htu.begin())
  {
    Serial.print("Error starting HTU21D sensor.");
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


void readandsend()
{
  static uint8_t loop_counter = 0;
  static uint16_t temperature_degC = 0;
  static uint16_t relative_humidity = 0;
  //static char data_counter = 0; // try to wait a cycle in between conversion and reading see if that helps

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

  // Read the temperature or humidity
  if (loop_counter == 0)
  {
    //htu.startTemperatureConversion();
  }
  if (loop_counter == 50)
  { 
    //htu.startHumidityConversion();
  }

  if (loop_counter == 25)
  {
    //temperature_degC = htu.readTemperature();
  }
  if (loop_counter == 75)
  {  
    //relative_humidity = htu.readHumidity();
  }
 
  
  //delay(1); // Without the delay we don't have populated things for sending
  
  // Send the data (use write for binary)
  Serial.println("HI");
  debugSerial.print(adc_ready_time);
  debugSerial.print(",");
  debugSerial.print(adc_reading);
  debugSerial.print(",");
  debugSerial.print(temperature_degC);
  //debugSerial.print("22");
  debugSerial.print(",");
  debugSerial.println(relative_humidity);
  /*
  //debugSerial.print("45");
  debugSerial.print(",");
  debugSerial.print(accelerometer_data.acceleration.x);
  debugSerial.print(",");
  debugSerial.print(accelerometer_data.acceleration.y);
  debugSerial.print(",");
  debugSerial.print(accelerometer_data.acceleration.z);
  debugSerial.print(",");
  debugSerial.print(magnetometer_data.magnetic.x);
  debugSerial.print(",");
  debugSerial.print(magnetometer_data.magnetic.y);
  debugSerial.print(",");
  debugSerial.print(magnetometer_data.magnetic.z);
  debugSerial.print(",");
  debugSerial.print(gyroscope_data.gyro.x);
  debugSerial.print(",");
  debugSerial.print(gyroscope_data.gyro.y);
  debugSerial.print(",");
  debugSerial.println(gyroscope_data.gyro.z);
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
