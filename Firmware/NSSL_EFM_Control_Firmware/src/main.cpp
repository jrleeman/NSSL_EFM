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
  float acceleration_y;
  float acceleration_z;
  float magnetometer_x;
  float magnetometer_y;
  float magnetometer_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  uint16_t temperature;
  uint16_t humidity;
  uint16_t pressure;
  uint8_t end_byte;
};

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

HardwareSerial FiberSer(PIN_FIBER_SERIAL_RX, PIN_FIBER_SERIAL_TX);  // RX, TX
HardwareSerial GPSSer(PIN_GPS_SERIAL_RX, PIN_GPS_SERIAL_TX);  // RX, TX
HardwareSerial DataSer(PIN_DATA_SERIAL_RX, PIN_DATA_SERIAL_TX);  // RX, TX

// State machine states
enum states{S_READ_FIBER_DATA, S_SEND_DATA, S_ERROR};

// Global variables
uint8_t current_state = S_READ_FIBER_DATA;

// Instance creation
TinyGPSPlus gps;
CircularBuffer<DataPacket, 100> DataPacketBuffer;
CircularBuffer<GPSPacket, 10> GPSPacketBuffer;

void GPSReadISR(void)
{
  /*
   * Reads the GPS data when a PPS is received and logs that PPS Time
   */
  while(GPSSer.available())
  {
    char c = GPSSer.read();
    if (gps.encode(c))
    {
      // New data is ready, break out of the while loop
      GPSPacket gps_packet;
      gps_packet.start_byte = 0xFE;
      gps_packet.millis = millis();
      gps_packet.lat = gps.location.lat();
      gps_packet.lon = gps.location.lng();
      gps_packet.alt = gps.altitude.meters();
      gps_packet.time = gps.time.value();
      gps_packet.satellites = gps.satellites.value();
      gps_packet.end_byte = 0xED;
      GPSPacketBuffer.push(gps_packet);
      break;
    }
  }
}

uint32_t readSerialint32()
{
  uint32_t result = 0;
  result = FiberSer.read();
  result = (result << 8) | FiberSer.read();
  result = (result << 8) | FiberSer.read();
  result = (result << 8) | FiberSer.read();
  return result;
}

float readSerialfloat()
{
  uint32_t result = 0;
  result = FiberSer.read();
  result = (result << 8) | FiberSer.read();
  result = (result << 8) | FiberSer.read();
  result = (result << 8) | FiberSer.read();
  return float(result);
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
  if (FiberSer.available() < 40)
  {
    return S_READ_FIBER_DATA;
  }

  DataPacket new_fiber_data;

  // Start reading and looking for 0xBE
  char c = FiberSer.read();
  
  if (c == 0xBE)  // Start of packet - read it in!
  {
    new_fiber_data.start_byte = c;
    new_fiber_data.adc_ready_time = readSerialint32();
    new_fiber_data.adc_reading = readSerialint32();
    new_fiber_data.acceleration_x = readSerialfloat();
    new_fiber_data.acceleration_y = readSerialfloat();
    new_fiber_data.acceleration_z = readSerialfloat();
    new_fiber_data.magnetometer_x = readSerialfloat();
    new_fiber_data.magnetometer_y = readSerialfloat();
    new_fiber_data.magnetometer_z = readSerialfloat();
    new_fiber_data.gyro_x = readSerialfloat();
    new_fiber_data.gyro_y = readSerialfloat();
    new_fiber_data.gyro_z = readSerialfloat();
    new_fiber_data.temperature = readSerialfloat();
    new_fiber_data.humidity = readSerialfloat();
    new_fiber_data.pressure = readSerialfloat();
  }

  new_fiber_data.end_byte = FiberSer.read();

  if (new_fiber_data.end_byte == 0xEF)
  {
    DataPacketBuffer.push(new_fiber_data);
  }
  return S_SEND_DATA; // Go to the send state
}

uint8_t sendData(void)
{
  /*
   * Send data to the other microcontrollers via serial
   */
  digitalWrite(PIN_LED_RUN, HIGH);

  while(!GPSPacketBuffer.isEmpty())
  {
    GPSPacket gps_packet = GPSPacketBuffer.pop();
    DataSer.print("G");
    DataSer.print(",");
    DataSer.print(gps_packet.millis);
    DataSer.print(",");
    DataSer.print(gps_packet.lat);
    DataSer.print(",");
    DataSer.print(gps_packet.lon);
    DataSer.print(",");
    DataSer.print(gps_packet.alt);
    DataSer.print(",");
    DataSer.print(gps_packet.time);
    DataSer.print(",");
    DataSer.println(gps_packet.satellites);
  }

  while (!DataPacketBuffer.isEmpty())
  {
    DataPacket data_packet = DataPacketBuffer.pop();
    DataSer.print("D");
    DataSer.print(",");
    DataSer.print(data_packet.adc_ready_time);
    DataSer.print(",");
    DataSer.print(data_packet.adc_reading);
    DataSer.print(",");
    DataSer.print(data_packet.acceleration_x);
    DataSer.print(",");
    DataSer.print(data_packet.acceleration_y);
    DataSer.print(",");
    DataSer.print(data_packet.acceleration_z);
    DataSer.print(",");
    DataSer.print(data_packet.magnetometer_x);
    DataSer.print(",");
    DataSer.print(data_packet.magnetometer_y);
    DataSer.print(",");
    DataSer.print(data_packet.magnetometer_z);
    DataSer.print(",");
    DataSer.print(data_packet.gyro_x);
    DataSer.print(",");
    DataSer.print(data_packet.gyro_y);
    DataSer.print(",");
    DataSer.print(data_packet.gyro_z);
    DataSer.print(",");
    DataSer.print(data_packet.temperature);
    DataSer.print(",");
    DataSer.print(data_packet.humidity);
    DataSer.print(",");
    DataSer.println(data_packet.pressure);
  }

  digitalWrite(PIN_LED_RUN, LOW);
  return S_READ_FIBER_DATA; 
}

uint8_t error()
{
  // Something has gone wrong, spin and blink the error light! The restart
 for (int i=0; i<10; i++)
  {
    digitalWrite(PIN_LED_ERROR, HIGH);
    delay(250);
    digitalWrite(PIN_LED_ERROR, LOW);
    delay(250);
  }
  return S_READ_FIBER_DATA;
}

void setup()
{
  // Setup the serial ports for everything
  FiberSer.begin(9600);
  GPSSer.begin(9600);
  DataSer.begin(115200);

  // Setup the GPS receiver interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), GPSReadISR, FALLING);

  // Pin setup
  pinMode(PIN_LED_RUN, OUTPUT);
  pinMode(PIN_LED_ERROR, OUTPUT);
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
    case S_SEND_DATA:
      current_state = sendData();
      break;
    case S_ERROR:
      current_state = error();
      break;
    default:
      current_state = error();
  }
}