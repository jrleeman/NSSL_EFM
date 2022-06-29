#define CIRCULAR_BUFFER_INT_SAFE // Keep this first!

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include "pins.h"
#include <SPI.h>
#include <SDFat.h>
#include <CircularBuffer.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

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
char intermediate_buffer[55];

// Instance creation
TinyGPSPlus gps;
CircularBuffer<uint8_t, 500> DataBuffer;
CircularBuffer<GPSPacket, 10> GPSPacketBuffer;
SFE_UBLOX_GNSS GNSS;


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


void setup()
{
  // Pin setup
  pinMode(PIN_LED_RUN, OUTPUT);
  pinMode(PIN_LED_GPS, OUTPUT);
  digitalWrite(PIN_LED_RUN, LOW);
  digitalWrite(PIN_LED_GPS, LOW);

  // Setup the serial ports for everything
  FiberSer.begin(38400);
  GPSSer.begin(9600);
  DataSer.begin(57600);

  // Setup the GNSS
  
  if (!GNSS.begin(GPSSer))
  {
    while(1);
  }
  GNSS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1); //Several of these are on by default on ublox board so let's disable them
  GNSS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
  GNSS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
  GNSS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
  GNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1); //Only leaving GGA & VTG enabled at current navigation rate
  GNSS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
  if (!GNSS.setDynamicModel(DYN_MODEL_AIRBORNE2g))
  {
    while(1);
  }
  

  delay(5000);
  digitalWrite(PIN_LED_RUN, HIGH);
  digitalWrite(PIN_LED_GPS, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_RUN, LOW);
  digitalWrite(PIN_LED_GPS, LOW);
  delay(1000);

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

 // If there's data on the fiber - read it in packet form
 static bool in_packet = false;
 static uint8_t i = 0;
 while (FiberSer.available())
 {
   // Always read the byte
   byte c = FiberSer.read();

   // If we are not in a packet and we hit a 0xBE we likley are at the packet start
   if ((!in_packet) && (c == 0xBE))
   {
     in_packet = true;
     i = 0;
     intermediate_buffer[i] = c;
     i += 1;
   }

   // We are in a packet, but about to over-run
   if (i > 53)
   {
     in_packet=false;
     i = 0;
   }

   // We are in a packet and just reading, looking for the end
   if (in_packet)
   {
     intermediate_buffer[i] = c;
     i += 1;

     // If it's the end, bounce out
     if ((c == 0xEF) && (i = 34))
     {
       in_packet = false;
       //digitalWrite(PIN_LED_RUN, LOW);
       for (int j=0; j <= i; j++)
       {
         DataBuffer.push(intermediate_buffer[j]);
       }
       break;
     }
   }
 }

 // Encode any available GPS characters
 
 while (GPSSer.available())
 {
   char c = GPSSer.read();
   if (gps.encode(c))
   {
     //digitalWrite(PIN_LED_GPS, !digitalRead(PIN_LED_GPS));
   }
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
