#define CIRCULAR_BUFFER_INT_SAFE // Keep this first!

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
  int32_t lat;
  int32_t lon;
  uint16_t alt;
  uint32_t time;
  uint8_t end_byte;
};

// Global variables
uint32_t last_pps = 0;

// Instance creation
TinyGPSPlus gps;
CircularBuffer<uint8_t, 500> DataBuffer;
CircularBuffer<GPSPacket, 10> GPSPacketBuffer;


void GPSReadISR(void)
{
  /*
   * Reads the GPS data when a PPS is received and logs that PPS Time
   */
  GPSPacket gps_packet;
  last_pps = millis();
  gps_packet.start_byte = 0xFE;
  gps_packet.millis = last_pps;
  gps_packet.lat = int32_t(gps.location.lat() * 10000);
  gps_packet.lon = int32_t(gps.location.lng() * 10000);
  gps_packet.alt = uint16_t(gps.altitude.meters());
  gps_packet.time = gps.time.value();
  gps_packet.end_byte = 0xED;
  GPSPacketBuffer.push(gps_packet);
}


void sendUBX(uint8_t *MSG, uint8_t len)
{
  /*
   * Send a byte array of UBX protocol to the GPS.
   */
  for(int i=0; i<len; i++) {
    GPSSer.write(MSG[i]);
  }
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
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      return false;
    }
 
    // Make sure data is available to read
    //DataSer.println("GETTING ACK");
    if (GPSSer.available()) {
      b = GPSSer.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
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
  pinMode(PIN_LED_GPS, OUTPUT);

  // Setup the serial ports for everything
  FiberSer.begin(38400);
  GPSSer.begin(9600);
  DataSer.begin(57600);

  delay(5000);
  digitalWrite(PIN_LED_RUN, HIGH);
  digitalWrite(PIN_LED_GPS, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_RUN, LOW);
  digitalWrite(PIN_LED_GPS, LOW);
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

 // If there's data on the fiber
 while (FiberSer.available())
 {
    DataBuffer.push(FiberSer.read());
 }

 // Encode any available GPS characters
 
 while (GPSSer.available())
 {
   char c = GPSSer.read();
   gps.encode(c);
 }


 // Dump the data buffer
 while(!DataBuffer.isEmpty())
 {
   digitalWrite(PIN_LED_RUN, HIGH);
   DataSer.write(DataBuffer.shift());
 }
 digitalWrite(PIN_LED_RUN, LOW);

 // If we have items in the GPS buffer, send them
 while(!GPSPacketBuffer.isEmpty())
  {
    digitalWrite(PIN_LED_GPS, !digitalRead(PIN_LED_GPS));
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