/*
 * Balloon Tracker
 * Sends the position of the instrument every 5 minutes and at that time checks the satellite
 * mailbox. If there is a valid message there we transmit it via the short range XBee to any
 * cutdown in range. Cutdowns are responsible for determining if the message is valid and
 * is intended for them.
 * 
 * The unit ID of the tracker is unrelated to the cutdown and is simply for keeping track of
 * the instruments.
 */

// Uncomment the next line to send all messages out via XBee for debugging/testing
//#define ENABLE_DEBUG

// This line should be uncomments for flight as it enables iridium, but can be disabled
// for troubleshooting to save data costs.
#define ENABLE_IRIDIUM

// Iridium library diagnostic messages - LEAVE ON thanks to a bug in the library
#define DIAGNOSTICS true

// Set ms to send iridium messages
#define TELEMETRY_MS 60000//300000

#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <IridiumSBD.h>
#include "pins.h"

SoftwareSerial xbeeSerial(PIN_XBEE_SERIAL_RX, PIN_XBEE_SERIAL_TX);
SoftwareSerial gpsSerial(PIN_GPS_SERIAL_RX, PIN_GPS_SERIAL_TX);
TinyGPSPlus gps;
IridiumSBD modem(Serial);

uint8_t unit_id = 2;
char tx_buffer[55];
uint8_t rx_buffer[55];

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void clear_buffers()
{
  // Clear the tx and rx buffers
  memset(&tx_buffer[0], 0, sizeof(tx_buffer));
  memset(&rx_buffer[0], 0, sizeof(rx_buffer));
}

void update_gps()
{
  // Read in characters from the GPS and parse into nice objects!
  while(gpsSerial.available())
  {
    digitalWrite(PIN_LED_ACTIVITY, HIGH);
    char c = gpsSerial.read();
    gps.encode(c);
  }
  digitalWrite(PIN_LED_ACTIVITY, LOW);
}

void setup()
{
  // Pin Setup
  pinMode(PIN_LED_ACTIVITY, OUTPUT);
  pinMode(PIN_GPS_PPS, INPUT);
  pinMode(PIN_RB_NETWORK, INPUT);

  // Turn activity on to show we're booting
  digitalWrite(PIN_LED_ACTIVITY, HIGH);

  // Serial
  Serial.begin(19200);
  xbeeSerial.begin(9600);
  gpsSerial.begin(9600);

  // Spin for about 30 seconds to let the GPS get a lock
  uint32_t start_warmup = millis();
  while ((millis() - start_warmup < 30000))
  {
    update_gps();
  }

  #ifdef ENABLE_DEBUG
  xbeeSerial.println("Setting up Iridium...");
  #endif

  // Iridium Setup
  #ifdef ENABLE_IRIDIUM
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // High current profile - we've got the power
  if (modem.begin() != ISBD_SUCCESS)
  {
    for (int i=0; i<20; i++)
    {
      digitalWrite(PIN_LED_ACTIVITY, HIGH);
      delay(100);
      digitalWrite(PIN_LED_ACTIVITY, LOW);
      delay(100);
    }
    //resetFunc();
  }
  #endif
  

  digitalWrite(PIN_LED_ACTIVITY, LOW);

  #ifdef ENABLE_DEBUG
  xbeeSerial.println("Iridium Setup Complete");
  #endif
}

void tx_rx_tracking()
{
  int err;

  // Clear buffers
  clear_buffers();

  // If it's time to send tracking (every 5 min) gather what we need and send it
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  double alt = gps.altitude.meters();

  sprintf(tx_buffer, "%d,%.4f,%.4f,%.1f\r", unit_id, lat, lon, alt);
  size_t rx_buffer_size = sizeof(rx_buffer);

  #ifdef ENABLE_DEBUG
  xbeeSerial.print("Iridium sending: ");
  xbeeSerial.println(tx_buffer);
  #endif

  #ifdef ENABLE_IRIDIUM
  err = modem.sendReceiveSBDText(tx_buffer, rx_buffer, rx_buffer_size);
  if (err != ISBD_SUCCESS)
  {
    #ifdef ENABLE_DEBUG
    xbeeSerial.print("Send error ");
    xbeeSerial.println(err);
    #endif
  }
  #endif

  #ifdef ENABLE_DEBUG
  xbeeSerial.print("Iridium send complete");
  #endif

  // Transmit anything that was in the rx buffer via the xBee link - repeat it
  // 3 times to be sure it goes through in a noisy environment.
  for (int j=0; j<3; j++)
    {
      for (size_t i=0; i<rx_buffer_size; i++)
      {
        xbeeSerial.write(rx_buffer[i]);
      }
      delay(15000);
    }
}

void loop()
{

  // The main loop updates the GPS and every set interval runs the satelitte modem
  // transmit/receive function to send position and check messages.
  static uint32_t last_tx_millis = 0;
  static char first_tx = 1;

  // Feed the GPS characters to the parser so we are always up to date
  update_gps();

  // If it's been more than the set interval since we last sent telemetry - we do it!
  if (((millis() - last_tx_millis) > TELEMETRY_MS) || first_tx)
  {
    last_tx_millis = millis();
    first_tx = 0;
    tx_rx_tracking();
  }

}

#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  //xbeeSerial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  //xbeeSerial.write(c);
}
#endif
