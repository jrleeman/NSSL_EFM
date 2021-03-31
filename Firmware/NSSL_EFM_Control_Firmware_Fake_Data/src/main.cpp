#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include "pins.h"
#include <SPI.h>
#include <SDFat.h>
#include <CircularBuffer.h>

HardwareSerial FiberSer(PIN_FIBER_SERIAL_RX, PIN_FIBER_SERIAL_TX);  // RX, TX
HardwareSerial GPSSer(PIN_GPS_SERIAL_RX, PIN_GPS_SERIAL_TX);  // RX, TX
HardwareSerial DataSer(PIN_DATA_SERIAL_RX, PIN_DATA_SERIAL_TX);  // RX, TX

struct GPSPacket
{
  uint8_t start_byte;
  uint32_t millis;
  float lat;
  float lon;
  float alt;
  uint32_t time;
  uint8_t satellites;
  uint8_t end_byte;
};
TinyGPSPlus gps;
CircularBuffer<GPSPacket, 10> GPSPacketBuffer;


uint8_t sendData(void)
{
  /*
   * Send data to the other microcontrollers via serial
   */
  digitalWrite(PIN_LED_RUN, HIGH);

  //DataSer.print("ABCDEFGHIJABCDEFGHIJABCDEFGHIJABCDEFGHIJABCDEFGHIJ\r\n");
  
  
  DataSer.print("D");
  DataSer.print(",");
  DataSer.print(millis());
  DataSer.print(",");
  DataSer.print("1239875");
  DataSer.print(",");
  DataSer.print("1239875");
  DataSer.print(",");
  DataSer.print("1239875");
  DataSer.print(",");
  DataSer.print("1239875");
  DataSer.print(",");
  DataSer.print("1239875");
  DataSer.print(",");
  DataSer.print("1239875");
  DataSer.print(",");
  DataSer.print("1239875");
  DataSer.print(",");
  DataSer.print("1239875");
  DataSer.print(",");
  DataSer.print("1239875");
  DataSer.print(",");
  DataSer.print("1239875");
  DataSer.print(",");
  DataSer.print("12.21");
  DataSer.print(",");
  DataSer.print("78.5");
  DataSer.print(",");
  DataSer.println("998.52");
  digitalWrite(PIN_LED_RUN, LOW);
  return 1; 
}

void GPSReadISR(void)
{
  digitalWrite(PIN_LED_RUN, HIGH);
  DataSer.print(millis());
  DataSer.print(",");
  DataSer.print(gps.location.lat(), 3);
  DataSer.print(",");
  DataSer.print(gps.location.lng(), 3);
  DataSer.print(",");
  DataSer.print(gps.altitude.meters());
  DataSer.print(",");
  DataSer.print(gps.time.value());
  DataSer.print(",");
  DataSer.println(gps.satellites.value());
  digitalWrite(PIN_LED_RUN, LOW);
}

void setup()
{
  // Setup the serial ports for everything
  FiberSer.begin(9600);
  GPSSer.begin(9600);
  DataSer.begin(57600);

  // Pin setup
  pinMode(PIN_LED_RUN, OUTPUT);
  pinMode(PIN_LED_ERROR, OUTPUT);
  pinMode(PIN_GPS_PPS, INPUT);

  // Setup the GPS receiver interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), GPSReadISR, FALLING);
}

void loop()
{
  /*
   * Main State Machine Loop
   */
  if (GPSSer.available())
  {
    char c = GPSSer.read();
    gps.encode(c);
  }

  //delay(45);
}