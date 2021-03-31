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
//CircularBuffer<DataPacket, 100> DataPacketBuffer;
CircularBuffer<uint8_t, 500> DataPacketBuffer;
CircularBuffer<GPSPacket, 10> GPSPacketBuffer;

typedef union {
 float floatingPoint;
 byte binary[4];
} binaryFloat;


void GPSReadISR(void)
{
  /*
   * Reads the GPS data when a PPS is received and logs that PPS Time
   */
  digitalWrite(PIN_LED_RUN, HIGH);
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
  digitalWrite(PIN_LED_RUN, LOW);
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
  while (FiberSer.available())
  {
   char c = FiberSer.read();
   DataPacketBuffer.push(c);
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
    binaryFloat binfloat;
    GPSPacket gps_packet = GPSPacketBuffer.shift();
    DataSer.write(0x47);
    DataSer.write(gps_packet.millis);
    binfloat.floatingPoint = gps_packet.lat;
    DataSer.write(binfloat.binary, 4);
    binfloat.floatingPoint = gps_packet.lon;
    DataSer.write(binfloat.binary, 4);
    binfloat.floatingPoint = gps_packet.alt;
    DataSer.write(binfloat.binary, 4);
    DataSer.write(gps_packet.time);
    DataSer.write(gps_packet.satellites);
    DataSer.write(0x48);
  }

  while (!DataPacketBuffer.isEmpty())
  {
    //DataPacket data_packet = DataPacketBuffer.pop();
    DataSer.write(DataPacketBuffer.shift());
    /*
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
    */
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
  FiberSer.begin(38400);
  GPSSer.begin(9600);
  DataSer.begin(38400);

  // Setup the GPS receiver interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), GPSReadISR, FALLING);

  // Pin setup
  pinMode(PIN_LED_RUN, OUTPUT);
  pinMode(PIN_LED_ERROR, OUTPUT);
}

void loop()
{
  uint8_t fiberbuffer[256];
  static uint8_t fiberbuffer_index = 0;
  static uint8_t inside_packet = 0;
  /*
   * Main State Machine Loop
   */
  /*
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
  */
 // Encode any available GPS characters
 if (GPSSer.available())
 {
   gps.encode(GPSSer.read());
 }

 // If there's data on the fiber
 if (FiberSer.available()>40)
 {
    inside_packet = 1;
    fiberbuffer[fiberbuffer_index] = FiberSer.read();
    if (fiberbuffer[fiberbuffer_index] == 0xEF)
    {
      inside_packet = 0;
    }
    fiberbuffer_index += 1;
 }


 // If we're not in a packet, send the last fiber packet and reset the pointer
 if (inside_packet == 0 && fiberbuffer_index > 0)
 {
   DataSer.write(fiberbuffer, fiberbuffer_index);
   fiberbuffer_index = 0;
 }

 // If we have items in the GPS buffer, send them
 while(!GPSPacketBuffer.isEmpty())
  {
    GPSPacket gps_packet = GPSPacketBuffer.shift();
    binaryFloat binfloat;
    /*
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
    DataSer.println(gps_packet.satellites);*/
    DataSer.write(0x47);
    DataSer.write(gps_packet.millis);
    binfloat.floatingPoint = gps_packet.lat;
    DataSer.write(binfloat.binary, 4);
    binfloat.floatingPoint = gps_packet.lon;
    DataSer.write(binfloat.binary, 4);
    binfloat.floatingPoint = gps_packet.alt;
    DataSer.write(binfloat.binary, 4);
    DataSer.write(gps_packet.time);
    DataSer.write(gps_packet.satellites);
    DataSer.write(0x48);
  }
}