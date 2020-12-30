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
  Serial.begin(115200);
  ss.begin(115200);
  pinMode(PIN_ID_0, INPUT);
  pinMode(PIN_ID_1, INPUT);
  pinMode(PIN_ID_2, INPUT);
  pinMode(PIN_LED_ACTIVITY, OUTPUT);
  pinMode(PIN_LED_ERROR, OUTPUT);
}

void loop()
{
  if (Serial.available())
  {
    digitalWrite(PIN_LED_ACTIVITY, HIGH);
    ss.write(Serial.read());
    digitalWrite(PIN_LED_ACTIVITY, LOW);
  }
}