/*
 * Control PCB SD Processor
 * Writes incoming data to the SD card and manages the file names.
 */

#include <Arduino.h>
#include <CircularBuffer.h>
#include "pins.h"

#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"

// Change SPI_SPEED to SD_SCK_MHZ(50) for best performance. 4 for slow testing
#define SPI_SPEED SD_SCK_MHZ(50)


char log_file_name[] = "EFM00.TXT";
CircularBuffer<char, 700> CharBuffer;
File dataFile;
SdFat sd;
uint32_t last_write = 0;

void(* resetFunc) (void) = 0; //declare reset function @ address 0

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
    if (! sd.exists(log_file_name))
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
    delay(500);
    digitalWrite(PIN_LED_ERROR, LOW);
    delay(500);
  }
  // Restart!
  resetFunc(); 
}

void setup()
{
  pinMode(PIN_LED_ACTIVITY, OUTPUT);
  pinMode(PIN_LED_ERROR, OUTPUT);
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_LED_ACTIVITY, HIGH);
  digitalWrite(PIN_LED_ERROR, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_ACTIVITY, LOW);
  digitalWrite(PIN_LED_ERROR, LOW);
  Serial.begin(38400);
  // We're not going to use CS since this is the only device, set it high
  //digitalWrite(PIN_SD_CS, HIGH);
  if (!sd.begin(PIN_SD_CS, SPI_SPEED))
  {
    error();
  }
  setLogFileName();
  //dataFile = sd.open(log_file_name, FILE_WRITE);
}

void loop()
{
  static uint8_t file_is_open = 1;
  static uint32_t write_cycles = 6000000;
  //dataFile = sd.open(log_file_name, FILE_WRITE);
  /*
  if ((millis() - last_write) > 10000)
  {
    digitalWrite(PIN_LED_ACTIVITY, HIGH);
    delay(20); 
    //dataFile.flush();
    dataFile.close();
    dataFile = sd.open(log_file_name, FILE_WRITE);
    digitalWrite(PIN_LED_ACTIVITY, LOW);
    last_write = millis();
  }
  */
  /*
  if(Serial.available())
  {   
    digitalWrite(PIN_LED_ACTIVITY, HIGH);
    dataFile = sd.open(log_file_name, FILE_WRITE);
    uint16_t buffer_size = Serial.available();
    char buff[500];
    for (uint16_t i=0; i<buffer_size; i++)
    {
      buff[i] = Serial.read();
    } 
    dataFile.write(buff, buffer_size);  
    dataFile.close();
    digitalWrite(PIN_LED_ACTIVITY, LOW);
  }
  */
 while(Serial.available())
  {   
    //digitalWrite(PIN_LED_ERROR, HIGH);
    CharBuffer.push(Serial.read());
    //digitalWrite(PIN_LED_ERROR, LOW);
  }

  if (write_cycles > 600000)
  {
    dataFile.close();
    setLogFileName();
    dataFile = sd.open(log_file_name, FILE_WRITE);
    write_cycles = 0;
  }

  if (CharBuffer.isFull())
  {digitalWrite(PIN_LED_ERROR, HIGH);}

  if (CharBuffer.size() > 10)
  {
    //digitalWrite(PIN_LED_ACTIVITY, HIGH);
    //while (!CharBuffer.isEmpty())
    for (int j=0; j<CharBuffer.size(); j++)
    {
      dataFile.write(CharBuffer.shift());  
      write_cycles += 1;
    }
    //digitalWrite(PIN_LED_ACTIVITY, LOW);
    
  }
}