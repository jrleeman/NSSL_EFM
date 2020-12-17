#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include "pins.h"
#include <SPI.h>
#include <SDFat.h>
#include <CircularBuffer.h>


// Data packet
struct DataPacket
{
  uint8_t start_byte;
  uint32_t adc_ready_time;
  uint32_t adc_reading;
  float acceleration_x;
  float acceleration_Y;
  float acceleration_Z;
  float magnetometer_x;
  float magnetometer_y;
  float magnetometer_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float gps_lat;
  float gps_lon;
  float gps_alt;
  uint16_t temperature;
  uint16_t humidity;
  uint8_t gps_hour;
  uint8_t gps_minute;
  uint8_t gps_second;
  uint8_t gps_centisecond;
  uint8_t end_byte;
};


HardwareSerial FiberSer(PIN_FIBER_SERIAL_RX, PIN_FIBER_SERIAL_TX);  // RX, TX
HardwareSerial GPSSer(PIN_GPS_SERIAL_RX, PIN_GPS_SERIAL_TX);  // RX, TX
HardwareSerial RadioSer(PIN_RADIO_SERIAL_RX, PIN_RADIO_SERIAL_TX);  // RX, TX

// State machine states
enum states{S_READ_FIBER_DATA, S_LOG_DATA, S_TX_DATA};

// Global variables
uint32_t last_pps_time = 0;
uint8_t current_state = S_READ_FIBER_DATA;

// Instance creation
TinyGPSPlus gps;
SdFat SD;
File logFile;
CircularBuffer<char, 2000> DataPacketBuffer;
char log_file_name[] = "EFM00.BIN";

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

void GPSReadISR(void)
{
  /*
   * Reads the GPS data when a PPS is received and logs that PPS Time
   */
  last_pps_time = millis();
  while(GPSSer.available())
  {
    char c = GPSSer.read();
    if (gps.encode(c))
    {
      // New data is ready, break out of the while loop
      break;
    }
  }
}


uint8_t readFiberData(void)
{
  /*
   * Read the data from the fiber-optic cable.
   * 
   * Operation:
   * - Read until we get a 0xBE (start packet) throwing awaying everything until then
   * - Read the next 49 bytes
   * - If we got 50 bytes total, ending in 0xEF, we'll assume it's a good packet and
   *   add it to the buffer of packets to write to card
   * - If it is a packet to be transmitted, add it to that buffer as well
   */

  // If there's nothing here - go back with the intention of checking again!
  if (!FiberSer.available() > 40)
  {
    return S_READ_FIBER_DATA;
  }

  byte buffer[55];

  // Start reading and looking for 0xBE
  char c = FiberSer.read();
  
  uint8_t buffer_idx = 0;
  if (c == 0xBE)
  {
    digitalWrite(PB6, HIGH);
    // We hit the start of the packet, write it to buffer and read the next 49 bytes or until
    // we find an EF.
    buffer[buffer_idx] = c;
    buffer_idx +=1;

    // Read the next 49 bytes or break when we hit end of packet
    for (uint8_t i=0; i<55; i++)
    {
      c = FiberSer.read();
      buffer[buffer_idx] = c;
      buffer_idx += 1;
      if (c == 0xEF)
      {
        digitalWrite(PB6, LOW);
        break;
      }
    }
    //digitalWrite(PB6, LOW);
    // Check if we've got a full packet that appears valid, if so add it to the buffer
    //if (buffer_idx == 49)
    //{
      for (int i=0; i<buffer_idx; i++)
      {
        DataPacketBuffer.push(buffer[i]);
        // Send it via radio while we are here?
      } // for
    //}  // if 
  }  // if
  return S_LOG_DATA; // Go to the log state
}

uint8_t logData(void)
{
  /*
   * Log the data to the SD card
   */
  //return S_READ_FIBER_DATA; 
  static uint8_t tx_data_counter = 0; // Counts how many packets to decimate the transmit
  uint8_t tx_decimate_factor = 20;

  static uint8_t flush_counter = 0; // Counts how many times we've entered this func without flushing
  
  flush_counter += 1;
  //logFile = SD.open(log_file_name, FILE_WRITE);
  // Write the packet
  while (!DataPacketBuffer.isEmpty())
  {
    logFile.write(DataPacketBuffer.pop());
  }
  //logFile.write(0xBE);
  //logFile.write(0xEF);
  //logFile.write(DataPacketBuffer.pop());

  if (flush_counter > 200)
  {
    logFile.flush();
    flush_counter = 0;
  }

  // Close the log file - only if necessary to prevent corruption
  //logFile.close();

  // If it's time to transmit, we go there, otherwise go get more data
  //if (tx_data_counter%tx_decimate_factor == 0)
  //{
  //  tx_data_counter = 0;
  //  return S_TX_DATA;
  //}
  //else
  //{
    return S_READ_FIBER_DATA; 
  //}
}

uint8_t transmitData(void)
{
  /*
   * Transmit data to the ground for tracking and analysis.
   */
  return S_READ_FIBER_DATA;
}

void setup()
{
  // Setup the serial ports for everything
  FiberSer.begin(115200); // SET A TIMEOUT!
  GPSSer.begin(9600);
  RadioSer.begin(1200);

  // Setup the SD Card
  // Startup SD Card
  SD.begin(PIN_SD_CS, SD_SCK_MHZ(18));

  // Setup the GPS receiver - Nothing necessary!

  // Configure the radio

  // Get a file name for writing and open
  setLogFileName();
  logFile = SD.open(log_file_name, FILE_WRITE);

  // Test to blink a light
  pinMode(PB6, OUTPUT);
  digitalWrite(PB6, HIGH);
  delay(1000);
  digitalWrite(PB6, LOW);
}

void loop()
{
  /*
   * Main State Machine Loop
   */
  switch (current_state)
  {
    case S_READ_FIBER_DATA:
      current_state = readFiberData();
      break;
    case S_LOG_DATA:
      current_state = logData();
      break;
    case S_TX_DATA:
      current_state = transmitData();
      break;
    default:
      while(1){}
  }
}