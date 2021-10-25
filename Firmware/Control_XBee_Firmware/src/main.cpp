/*
 * Control Board XBee Processor
 * Simply echos the serial data coming in right out through the XBee. ID switches are there
 * if we need to get more advanced with multiple balloons in flight.
 */
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "pins.h"

SoftwareSerial ss(PIN_RADIO_SERIAL_RX, PIN_RADIO_SERIAL_TX);

void setup()
{
  Serial.begin(57600);
  ss.begin(57600);
  pinMode(PIN_ID_0, INPUT);
  pinMode(PIN_ID_1, INPUT);
  pinMode(PIN_ID_2, INPUT);
  pinMode(PIN_LED_ACTIVITY, OUTPUT);
  pinMode(PIN_LED_ERROR, OUTPUT);
}

void loop()
{
  
  while (Serial.available())
  {
    digitalWrite(PIN_LED_ACTIVITY, HIGH);
    ss.write(Serial.read()); 
  }
  digitalWrite(PIN_LED_ACTIVITY, LOW);
  //delay(10);
  
  
  /*
  // Sample data send
  digitalWrite(PIN_LED_ACTIVITY, HIGH);
  ss.write(0xDE);
  ss.write(0xAD);
  ss.write(0xBE);
  ss.write(0xEF);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0x01);
  ss.write(0xDE);
  ss.write(0xAD);
  ss.write(0xBE);
  ss.write(0xEF);
  digitalWrite(PIN_LED_ACTIVITY, LOW);
  delay(50);
  */
  
}