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
  int32_t lat;
  int32_t lon;
  uint16_t alt;
  uint32_t time;
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
  gps_packet.lat = int32_t(gps.location.lat() * 10000);
  gps_packet.lon = int32_t(gps.location.lng() * 10000);
  gps_packet.alt = uint16_t(gps.altitude.meters());
  gps_packet.time = gps.time.value();
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

  /*
  while(!GPSPacketBuffer.isEmpty())
  {

    GPSPacket gps_packet = GPSPacketBuffer.shift();
    DataSer.write((const char*)&gps_packet, sizeof(gps_packet));
  }
  */
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

void sendUBX(uint8_t *MSG, uint8_t len)
{
  /*
   * Send a byte array of UBX protocol to the GPS.
   */
  //DataSer.println("SENDING:");
  for(int i=0; i<len; i++) {
    GPSSer.write(MSG[i]);
   // DataSer.println(MSG[i], HEX);
  }
  //DataSer.println("COMPLETE");
  GPSSer.println();
}
 
boolean getUBX_ACK(uint8_t *MSG)
{
  /*
   * Calculate expected UBX ACK packet and parse UBX response from GPS.
   */
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  //DataSer.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      //DataSer.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      //DataSer.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    //DataSer.println("GETTING ACK");
    if (GPSSer.available()) {
      b = GPSSer.read();
      //DataSer.println(b, HEX);
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        //DataSer.print(b, HEX);
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }
 
    }
  }
}

bool setDynamicMode6()
{
  /*
   * Sets up the GPS in high altitude dynamic mode 6.
   */
  static byte gps_set_sucess = 0 ;
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00,
                      0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,
                      0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
  uint32_t start_millis = millis();
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setNav);
    // Timeout after 10 seconds
    if ((millis() - start_millis) > 10000)
    {
      return false;
    }
  }
  gps_set_sucess=0;
  return gps_set_sucess;
}
void setup()
{
  // Pin setup
  pinMode(PIN_LED_RUN, OUTPUT);
  pinMode(PIN_LED_ERROR, OUTPUT);

  // Setup the serial ports for everything
  FiberSer.begin(38400);
  GPSSer.begin(9600);
  DataSer.begin(57600);

  delay(5000);
  digitalWrite(PIN_LED_RUN, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_RUN, LOW);
  delay(1000);

  setDynamicMode6();

  digitalWrite(PIN_LED_RUN, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_RUN, LOW);
  delay(1000);

  // Setup the GPS receiver interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), GPSReadISR, FALLING);

}

void loop()
{
  uint8_t fiberbuffer[256];
  static uint8_t fiberbuffer_index = 0;
  static uint8_t inside_packet = 0;

 // Encode any available GPS characters
 if (GPSSer.available())
 {
   char c = GPSSer.read();
   gps.encode(c);
 }

 // If there's data on the fiber
 if (FiberSer.available()>40)
 {
    digitalWrite(PIN_LED_ERROR, HIGH);
    inside_packet = 1;
    fiberbuffer[fiberbuffer_index] = FiberSer.read();
    if (fiberbuffer[fiberbuffer_index] == 0xEF)
    {
      inside_packet = 0;
    }
    fiberbuffer_index += 1;
 }
 digitalWrite(PIN_LED_ERROR, LOW);


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
    DataSer.write(gps_packet.start_byte);
    DataSer.write((byte*)&gps_packet.millis, sizeof(uint32_t));
    DataSer.write((byte*)&gps_packet.lat, sizeof(int32_t));
    DataSer.write((byte*)&gps_packet.lon, sizeof(int32_t));
    DataSer.write((byte*)&gps_packet.alt, sizeof(uint16_t));
    DataSer.write((byte*)&gps_packet.time, sizeof(uint32_t));
    DataSer.write(gps_packet.end_byte);
  }
}