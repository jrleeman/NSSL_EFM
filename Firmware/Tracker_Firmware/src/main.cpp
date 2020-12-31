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

#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <IridiumSBD.h>
#include "pins.h"

SoftwareSerial xbeeSerial(PIN_XBEE_SERIAL_RX, PIN_XBEE_SERIAL_TX);
SoftwareSerial gpsSerial(PIN_GPS_SERIAL_RX, PIN_GPS_SERIAL_TX);
TinyGPSPlus gps;
IridiumSBD modem(Serial);

uint8_t unit_id = 1;
char tx_buffer[55];
uint8_t rx_buffer[55];

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void clear_buffers()
{
  // Clear the tx and rx buffers
  memset(tx_buffer, 0, sizeof tx_buffer);
  memset(rx_buffer, 0, sizeof rx_buffer);
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
  xbeeSerial.begin(115200);
  gpsSerial.begin(9600);

  // Iridium Setup
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // High current profile - we've got the power
  if (modem.begin() != ISBD_SUCCESS)
  {
    digitalWrite(PIN_LED_ACTIVITY, LOW);
    delay(1000);
    digitalWrite(PIN_LED_ACTIVITY, HIGH);
    delay(1000);
    digitalWrite(PIN_LED_ACTIVITY, LOW);
    delay(1000);
    digitalWrite(PIN_LED_ACTIVITY, HIGH);
    delay(1000);
    digitalWrite(PIN_LED_ACTIVITY, LOW);
    resetFunc();
  }
  digitalWrite(PIN_LED_ACTIVITY, LOW);
}

void update_gps()
{
  // Read in characters from the GPS and parse into nice objects!
  while(gpsSerial.available())
  {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated())
  {
    digitalWrite(PIN_LED_ACTIVITY, HIGH);
    delay(100);
    digitalWrite(PIN_LED_ACTIVITY, LOW);
    delay(100);
  }
}

void tx_rx_tracking()
{
  // Clear buffers
  clear_buffers();

  // If it's time to send tracking (every 5 min) gather what we need and send it
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  double alt = gps.altitude.meters();
  sprintf(tx_buffer, "%d,%.3f,%.3f,%.3f", unit_id, lat, lon, alt);
  size_t rx_buffer_size = sizeof(rx_buffer);
  modem.sendReceiveSBDText(tx_buffer, rx_buffer, rx_buffer_size);

  // Transmit anything that was in the rx buffer via the xBee link - repeat it
  // 3 times to be sure it goes through in a noisy environment.
  for (int j=0; j<3; j++)
    {
      for (size_t i=0; i<rx_buffer_size; i++)
      {
        xbeeSerial.write(rx_buffer[i]);
      }
      delay(1000);
    }
}

void loop()
{
  // The main loop updates the GPS and every 5 mintues runs the satelitte modem
  // transmit/receive function to send position and check messages.
  static uint32_t last_tx_millis = 0;

  // Feed the GPS characters to the parser so we are always up to date
  update_gps();

  // If it's been more than 5 minutes since we last sent telemetry - we do it!
  if ((millis() - last_tx_millis) > 300000)
  {
    last_tx_millis = millis();
    tx_rx_tracking();
  }
}
