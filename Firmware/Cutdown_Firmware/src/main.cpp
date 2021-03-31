/*
 * Cutdown Firmware
 * 
 * Terminates a balloon flight if any of these conditions are true:
 * - Flight has been longer than a set maximum duration and is still ascending
 * - Flight has reached a set pressure threshold and is still ascending
 * - External cutdown command pin is pulled low
 * - The XBee receives a valid cutdown message with this unit number identified
 */ 

// Uncomment the next line to send all messages out via XBee for debugging/testing
#define ENABLE_DEBUG

// Uncomment the next line to simulate ascent after boot to arm the system for testing
#define FORCE_ARM

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "pins.h"
#include "Cmd.h"
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

SoftwareSerial xbeeSerial(PIN_XBEE_SERIAL_RX, PIN_XBEE_SERIAL_TX);
Adafruit_BMP3XX bmp;

// EEPROM Addresses
const uint8_t EEPROM_BASE_ADDRESS = 0;

// Firmware Version
const uint8_t FIRMWARE_MAJOR_VERSION = 1;
const uint8_t FIRMWARE_MINOR_VERSION = 1;

struct PersistentData
{
  uint8_t system_first_boot;
  uint16_t cutdown_pressure_hpa;
  uint8_t cutdown_time_minutes;
  uint8_t cutdown_duration_seconds;
  uint8_t unit_id;
  uint16_t cutdown_arm_hpa;
  uint8_t firmware_major_version;
  uint8_t firmware_minor_version;
};

// Create instance to hold settings
PersistentData device_settings;

float GetPressureReading()
{
    bmp.performReading();
    return bmp.pressure / 100;
}

void SetDefaults()
{
  PersistentData defaults = {
    42,
    10,
    120,
    30,
    255,
    10,
    FIRMWARE_MAJOR_VERSION,
    FIRMWARE_MINOR_VERSION
  };

  EEPROM.put(EEPROM_BASE_ADDRESS, defaults);
  EEPROM.get(EEPROM_BASE_ADDRESS, device_settings);
  Serial.println(F("Default settings restored."));
}

uint8_t CheckFirstBoot()
{
  // Checks if this is the devices first boot of if the EEPROM settings
  // appear to be incorrect. Checks for the first boot value and the 
  // firmware version to see if they make sense.
  char first_boot = 0;
  if ((device_settings.system_first_boot != 42) ||
      (device_settings.firmware_major_version != FIRMWARE_MAJOR_VERSION) ||
      (device_settings.firmware_minor_version != FIRMWARE_MINOR_VERSION)
     )
  {
    first_boot = 1;
  }
 return first_boot;
}

// Flight states
enum flight_states {FLIGHT_START, FLIGHT_ASCENDING, FLIGHT_COMPLETE};

// Globals
uint16_t flight_duration_seconds = 0;
float starting_pressure = 0;
float lowest_pressure = 9000;
uint8_t current_flight_state = FLIGHT_START;
float current_pressure = 0;

void Help(int arg_cnt, char **args)
{
  // Show some basic help information
  Serial.println(F("Enter commands followed by newline character"));
  Serial.println(F("SETPRES XXXX - Set the pressure below which the flight will cutdown in hPa (integer)."));
  Serial.println(F("SETTIME XXXX - Set the time after which the flight will be cutdown in minutes (integer)."));
  Serial.println(F("SETDUR XXXX - Set the duration of the cutdown cycle in seconds (integer)."));
  Serial.println(F("SETID XXX - Set the ID of the cutdown 0-255 (integer)."));
  Serial.println(F("SETARM XXXX - Set the change in pressure in hPa required before the cutdown arms (integer)."));
  Serial.println(F("HELP - Show this screen"));
  Serial.println(F("SHOW - Show current settings and information"));
  Serial.println(F("DEFAULTS - Reset to factory defaults"));
}

void SetArm(int arg_cnt, char **args)
{
  if (arg_cnt > 0)
  {
    device_settings.cutdown_arm_hpa = cmdStr2Num(args[1], 10);
    EEPROM.put(EEPROM_BASE_ADDRESS, device_settings);
    EEPROM.get(EEPROM_BASE_ADDRESS, device_settings);
  }
}

void SetDuration(int arg_cnt, char **args)
{
  if (arg_cnt > 0)
  {
    device_settings.cutdown_duration_seconds = cmdStr2Num(args[1], 10);
    EEPROM.put(EEPROM_BASE_ADDRESS, device_settings);
    EEPROM.get(EEPROM_BASE_ADDRESS, device_settings);
  }
}

void SetId(int arg_cnt, char **args)
{
  if (arg_cnt > 0)
  {
    device_settings.unit_id = cmdStr2Num(args[1], 10);
    EEPROM.put(EEPROM_BASE_ADDRESS, device_settings);
    EEPROM.get(EEPROM_BASE_ADDRESS, device_settings);
  }
}

void SetPressure(int arg_cnt, char **args)
{
  if (arg_cnt > 0)
  {
    device_settings.cutdown_pressure_hpa = cmdStr2Num(args[1], 10);
    EEPROM.put(EEPROM_BASE_ADDRESS, device_settings);
    EEPROM.get(EEPROM_BASE_ADDRESS, device_settings);
  }
}

void SetTime(int arg_cnt, char **args)
{
  if (arg_cnt > 0)
  {
    device_settings.cutdown_time_minutes = cmdStr2Num(args[1], 10);
    EEPROM.put(EEPROM_BASE_ADDRESS, device_settings);
    EEPROM.get(EEPROM_BASE_ADDRESS, device_settings);
  }
}

uint16_t GetPressure()
{
  return device_settings.cutdown_pressure_hpa;
}

uint8_t GetTime()
{
  return device_settings.cutdown_time_minutes;
}

uint8_t GetDuration()
{
  return device_settings.cutdown_duration_seconds;
}

uint8_t GetId()
{
  return device_settings.unit_id;
}

uint16_t GetArm()
{
  return device_settings.cutdown_arm_hpa;
}

void Show(int arg_cnt, char **args)
{
  Serial.print(F("Firmware Version: "));
  Serial.print(device_settings.firmware_major_version);
  Serial.print(F("."));
  Serial.println(device_settings.firmware_minor_version);
  Serial.println(F("\nCutdown Settings"));
  Serial.println(F("--------------------------"));
  Serial.print(F("Cutdown ID: "));
  Serial.println(GetId());
  Serial.print(F("Cutdown Pressure (hPa): "));
  Serial.println(GetPressure());
  Serial.print(F("Cutdown Arming (hPa): "));
  Serial.println(GetArm());
  Serial.print(F("Cutdown Time (minutes): "));
  Serial.println(GetTime());
  Serial.print(F("Cutdown Duration (seconds): "));
  Serial.println(GetDuration());
  Serial.print(F("System Boot Value: "));
  Serial.println(device_settings.system_first_boot);

}

uint8_t xbee_cutdown()
{
  // Receive characters from the XBee until we hit a newline character. Once we find one we
  // return a 1 if it is valid and addressed to this unit or a 0 otherwise. We reset the
  // buffer on a newline since it's the start of a new packet.

  char target_chars[50];
  memset(&target_chars[0], 0, sizeof(target_chars));
  sprintf(target_chars, "NSSL CUTDOWN%d", GetId());
  char rx_chars[150];
  memset(&rx_chars[0], 0, sizeof(rx_chars));
  uint8_t rx_idx = 0;
  uint8_t strcmp_res = 1;
  while(xbeeSerial.available())
  {
    char c = xbeeSerial.read();
    if (c == 0x0A)
    {
      strcmp_res = strcmp(target_chars, rx_chars);
      rx_idx = 0;
      memset(rx_chars, 0, sizeof rx_chars);
    }
    else
    {
      rx_chars[rx_idx] = c;
      rx_idx += 1;
    }    
  }
  // If the strings match (result from strcmp of 0), return 1 to do the cutdown
  if (strcmp_res == 0)
  {
    return 1;
  }
  // Otherwise return 0
  return 0;
}

uint8_t external_cutdown()
{
  // Checks the external cutdown signal pin 10 times and if all 10 are low, then we return
  // a positive value to do the cutdown.
  uint8_t low_counter = 0;
  for (uint8_t i=0; i<10; i++)
  {
    if (digitalRead(PIN_EXT_CUTDOWN) == LOW)
    {
      low_counter += 1;
      delay(5);
    }
  }

  if (low_counter == 10)
  {
    return 1;
  }
  return 0;
}

uint8_t check_conditions()
{
  // * Reads the pressure into the current pressure varaible
  // * Sets flight state to complete if the pressure has increased from last time
  // * Checks if we have reached the time limit of the flight
  // * Checks if we have reached the pressure limit of the flight
  // * Checks is we have received a valid cutdown command via XBee
  // * Checks for external cutdown command
  // * Returns 0 if we don't need to cutdown, positive value otherwise
  uint8_t do_cutdown = 0;

  // Read pressure and see if flight is complete add some pressure for hystersis!
  float pressure = GetPressureReading();
  if (pressure < lowest_pressure)
  {
    lowest_pressure = pressure;
  }
  if (lowest_pressure < (current_pressure - GetArm() * 2))
  {
    #ifdef ENABLE_DEBUG
    xbeeSerial.println("FLIGHT NATURALLY TERMINATED");
    #endif
    Serial.println("FLIGHT NATURALLY TERMINATED");
    current_flight_state = FLIGHT_COMPLETE;
  }
  current_pressure = pressure;

  // Check if the time limit is reached
  if ((flight_duration_seconds / 60) >= GetTime())
  {
    Serial.println("Flight Duration Limit Reached");
    #ifdef ENABLE_DEBUG
    xbeeSerial.println("Flight Duration Limit Reached");
    #endif
    do_cutdown += 1;
  }

  // Check if the pressure limit is reached
  if (current_pressure < GetPressure())
  {
    Serial.println("Flight Pressure Limit Reached");
    #ifdef ENABLE_DEBUG
    xbeeSerial.println("Flight Pressure Limit Reached");
    #endif
    do_cutdown += 1;
  }

  // Check for XBee cutdown signal
  if (xbee_cutdown())
  {
    Serial.println("XBee Cutdown Received");
    #ifdef ENABLE_DEBUG
    xbeeSerial.println("XBee Cutdown Received");
    #endif
    do_cutdown += 1;
  }

  // Check for external cutdown signal
  if (external_cutdown())
  {
    Serial.println("External Cutdown Received");
    #ifdef ENABLE_DEBUG
    xbeeSerial.println("External Cutdown Received");
    #endif
    do_cutdown += 1;
  }

  return do_cutdown;
}

void setup()
{
  // Serial
  Serial.begin(9600);
  xbeeSerial.begin(9600);

  // Show the greeting
  Serial.println(F("Cutdown Module"));
  Serial.println(F("Leeman Geophysical LLC"));

  // Reads the settings from the EEPROM into our structure
  EEPROM.get(EEPROM_BASE_ADDRESS, device_settings);

  // Check and see if it's valid or if we need to restore defaults
  if (CheckFirstBoot() == 1)
  {
    Serial.println(F("Restoring Defaults!"));
    SetDefaults();
  }

  // Pin Modes
  pinMode(PIN_LED_ERROR, OUTPUT);
  pinMode(PIN_EXT_CUTDOWN, INPUT);
  pinMode(PIN_HOTWIRE, OUTPUT);

  // Pressure sensor
  bmp.begin_I2C(0x76);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  
  // Setup the command structure
  cmdInit(&Serial);
  cmdAdd("SETPRES", SetPressure);
  cmdAdd("SETTIME", SetTime);
  cmdAdd("SETDUR", SetDuration);
  cmdAdd("SETID", SetId);
  cmdAdd("SETARM", SetArm);
  cmdAdd("HELP", Help);
  cmdAdd("SHOW", Show);

  // Get the startup pressure
  for (int i=0; i<10; i++)
  {
    starting_pressure += GetPressureReading();
  }
  starting_pressure /= 10;
  current_pressure = starting_pressure;
  
  #ifdef FORCE_ARM
  Serial.println("TESTING PRESSURE OFFSET APPLIED - NOT FOR FLIGHT!");
  starting_pressure += 12 + GetArm();
  #endif
  

  Serial.print("Starting pressure (hPa): ");
  Serial.println(starting_pressure);
}

void execute_cutdown()
{
  // Turn on the hot wire for duration seconds
  digitalWrite(PIN_HOTWIRE, HIGH);
  delay(GetDuration() * 1000);
  digitalWrite(PIN_HOTWIRE, LOW);

  // Set the flight state to complete
  current_flight_state = FLIGHT_COMPLETE;
}

void serial_update()
{
  switch (current_flight_state)
  {
    case FLIGHT_START:
      Serial.print("DISARMED");
      #ifdef ENABLE_DEBUG
      xbeeSerial.print("DISARMED");
      #endif
      break;
    case FLIGHT_ASCENDING:
      Serial.print("ARMED");
      #ifdef ENABLE_DEBUG
      xbeeSerial.print("ARMED");
      #endif
      break;
    case FLIGHT_COMPLETE:
      Serial.print("COMPLETE");
      #ifdef ENABLE_DEBUG
      xbeeSerial.print("COMPLETE");
      #endif
      break;
    default:
      Serial.print("ERROR");
      #ifdef ENABLE_DEBUG
      xbeeSerial.print("ERROR");
      #endif
      break;
  }
  Serial.print("\t");
  Serial.print(flight_duration_seconds);
  Serial.print("\t");
  Serial.println(GetPressureReading());

  #ifdef ENABLE_DEBUG
  xbeeSerial.print("\t");
  xbeeSerial.print(flight_duration_seconds);
  xbeeSerial.print("\t");
  xbeeSerial.println(GetPressureReading());
  #endif
}

void loop()
{

  static uint32_t last_check_millis = millis();
  static uint32_t last_serial_millis = millis();

  // Send the serial status of the device and update flight duration every 10 seconds
  if ((millis() - last_serial_millis) > 5000)
  {
    flight_duration_seconds += (millis() - last_serial_millis) / 1000;

    // If the state is not armed (start of flight only) then we keep duration at zero
    // We keep the clock running when armed (in flight) and through the descent and landing.
    if (current_flight_state == FLIGHT_START)
    {
      flight_duration_seconds = 0;
    }

    last_serial_millis = millis();
    serial_update();
  }

  // Check for serial commands and handle them
  cmdPoll();

  switch (current_flight_state)
  {
    // If we're not armed yet, we check if we need to arm
    case FLIGHT_START:
      // Check the pressure - if it's more than threshold different from the starting value
      // then we are ascending and we arm the system.
      if ((starting_pressure - GetPressureReading()) > GetArm())
      {
        current_flight_state = FLIGHT_ASCENDING;
      }
      break;

    // If we are armed then we run the normal procedure
    case FLIGHT_ASCENDING:
      // Every 10 seconds
      if ((millis() - last_check_millis) > 10000)
      {
        last_check_millis = millis();

        // Check the conditions
        uint8_t do_cutdown = check_conditions();

        // If needed, do a cutdown
        if (do_cutdown)
        {
          execute_cutdown();
        }
      }
      break;
  
    // If the flight has been terminated then we are just along for the ride - turn on the
    // error so we are not accidently relaunched!
    case FLIGHT_COMPLETE:
      digitalWrite(PIN_LED_ERROR, HIGH);
      break;

    default:
      break;
  }
}
