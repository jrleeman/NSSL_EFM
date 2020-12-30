/*
 * Control PCB SD Processor
 * Writes incoming data to the SD card and manages the file names.
 */

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <CircularBuffer.h>
#include "pins.h"

char log_file_name[] = "EFM00.TXT";
CircularBuffer<char, 500> CharBuffer;
File dataFile;

void setLogFileName(void)
{
  /*
   * Look on the SD card and find the next available file name. Set the global.
   * If allowed to run on it will overwite the 99th file over and over.
   */
  for (uint8_t i = 0; i < 100; i++)
  {
    log_file_name[3] = i/10 + '0';
    log_file_name[4] = i%10 + '0';
    if (! SD.exists(log_file_name))
    {
      break; // leave the loop!
    }
  }
}

void error()
{
  for (int i=0; i<10; i++)
  {
    digitalWrite(PIN_LED_ERROR, HIGH);
    delay(250);
    digitalWrite(PIN_LED_ERROR, LOW);
    delay(250);
  }
  // Restart!
}

void setup()
{
  Serial.begin(115200);
  if (!SD.begin(PIN_SD_CS))
  {
    error();
  }
  setLogFileName();

}

void loop()
{
  if(Serial.available())
  {
    CharBuffer.push(Serial.read());
  }

  if(CharBuffer.size() > 100)
  {
    dataFile = SD.open(log_file_name, FILE_WRITE);
    while(CharBuffer.size() > 0)
    {
      dataFile.write(CharBuffer.pop());
    }
    dataFile.close();
  }
}