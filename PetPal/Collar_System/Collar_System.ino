#include <stdlib.h>
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
// Bluetooth module
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"
// Heart rate/temp module
#include "MAX30105.h"
#include "heartRate.h"
// Speaker
#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
// Hardware serial port
#define GPSSerial Serial

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

// Bluetooth module
// ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST 
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Heart rate/temp module
MAX30105 particleSensor;

// Timer for the GPS
uint32_t timer = millis();

// Code needed for the heart rate
const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;        // Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);
// the packet buffer
extern uint8_t packetbuffer[];

/* These variables used to debug GPS
// variables to save GPS data
char coordinates[50];
char latt[25];
char longi[25];
// Short versions
char smallLatt[25];
char smallLongi[25];
char tempCoord[25];
*/

// A small helper for catching errors
void error(const __FlashStringHelper*err)
{
  Serial.println(err);
  // infinite loop
  while (1);
}

// Setup all sensors and the serial monitor
void setup(void)
{
  delay(500);
 
  // Use baud rate at 115200 to capture UART data
  // This is also needed to read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  Serial.println(F("Initializing heart/temperature..."));

  // Initialize heart rate/temperature sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    // If the sensor cannot be initialized
    Serial.println(F("Heart/Temperature Sensor was not found. Please check wiring/power. "));
    while (1);
  }
 
  // Turn off the LEDs so they won't affect the temperature reading
  particleSensor.setup(0);
  // Enable the temp ready interrupt. This is required
  particleSensor.enableDIETEMPRDY();

  // NOW LET'S INITIALIZE THE GPS MODULE
  // 9600 NMEA is the default baud rate for Adafruit GPS
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
 
  // NOW LET'S INITIALIZE THE BLUEFRUIT MODULE
  Serial.println(F("Initializing the Bluefruit LE module: "));

  if (!ble.begin(VERBOSE_MODE))
  {
    error(F("Couldn't find Bluefruit, make sure it's in Command mode & check wiring"));
  }

  if (FACTORYRESET_ENABLE)
  {
    // Perform a factory reset to make sure everything is in a known state 
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  // Disable command echo from Bluefruit 
  ble.echo(false);

  Serial.println(F("Requesting Bluefruit info:"));
  // Print Bluefruit information 
  ble.info();
  ble.verbose(false);  // debug info is a little annoying after this point!

  // Wait for connection 
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }
}

// Constantly poll for new command or response data
void loop(void)
{
 
  // Variable to save temperature data
  float temperatureF;

  /*
  // Check state of BLE
  if (ble.isConnected())
  {
    Serial.println("Bluetooth is connected, continue as normal!");
  }
  else
  {
    Serial.println("Bluetooth error");
  }
  */
 
  // Always return back to data mode
  ble.setMode(BLUEFRUIT_MODE_DATA);
  // Serial.println(F("Ready to read data!"));

  // Always turn off IR led after button presses so heat does not accumulate
  particleSensor.setup(0);
 
  // Wait for new data to arrive -- DATA MODE!!! 
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0)
  {
    Serial.println(F("Packet Length is 0"));
    return;
  }

  // Got a packet! 
  printHex(packetbuffer, len);

  // BUTTONS
  if (packetbuffer[1] == 'B')
  {
    // Parse the packetbuffer to find button information
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';

    // Button 1 is reserved for tempearture readings
    if (buttnum == 1)
    {
      /*
      for (int i = 0; i < len; i++)
      {
         Serial.println(packetbuffer[i]);
      }
      */
      if(pressed)
      {
        //Serial.println(F("Button 1 pressed"));
        temperatureF = particleSensor.readTemperatureF();
       
        // Now switch to command mode to send data
        ble.setMode(BLUEFRUIT_MODE_COMMAND);
        //Serial.println("Switched to command mode - sending data");

        ble.print("AT+BLEUARTTX=");
        ble.println(temperatureF);

        // check response stastus
        if (!ble.waitForOK())
        {
          Serial.println(F("Failed to send?"));
        }

        Serial.print(F("[Send] "));
        Serial.println(temperatureF);
      }
    }

    // Button 2 is reserved for heart rate readings
    if (buttnum == 2)
    {
      if(pressed)
      {
        // Turn on red led
        particleSensor.setup();
        //Serial.println(F("Button 2 pressed"));

        // now switch to command mode to send data
        ble.setMode(BLUEFRUIT_MODE_COMMAND);
       
        Serial.println(F("Calculating heart rate... this will take 10 seconds"));
        long starttime = millis();
        long endtime = starttime;
         
        // Check heart rate for 10 seconds then spit out the data
        while ((endtime - starttime) <= 10000)
        {
          // Run BPM loop in the background
          long irValue = particleSensor.getIR();

          if (checkForBeat(irValue) == true)
          {
            // We sensed a beat!
            long delta = millis() - lastBeat;
            lastBeat = millis();

            beatsPerMinute = 60 / (delta / 1000.0);

            if (beatsPerMinute < 255 && beatsPerMinute > 20)
            {
              rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
              rateSpot %= RATE_SIZE; // Wrap variable

              // Take average of readings
              beatAvg = 0;
              for (byte x = 0; x < RATE_SIZE; x++)
                beatAvg += rates[x];
              beatAvg /= RATE_SIZE;
            }
          }
          endtime = millis();
        }
        Serial.print(F("Average BPM="));
        Serial.print(beatAvg);
        Serial.println();

        ble.print("AT+BLEUARTTX=");
        ble.println(beatAvg);

        // check response status
        if (!ble.waitForOK())
        {
          Serial.println(F("Failed to send?"));
        }
      }
    }
   
    // Button3 is reserved for GPS data
    if (buttnum == 3)
    {
      if(pressed)
      {
        // Loop until we have received a GPS fix and hence valid location data
        while (1)
        {
          // Read data from the GPS in the 'main loop'
          char c = GPS.read();
          // if you want to debug, this is a good time to do it!
          if (GPSECHO)
          {
            if (c) Serial.print(c);    
          }
          // If a sentence is received, we can check the checksum, parse it...
          if (GPS.newNMEAreceived())
          {
            // A tricky thing here is if we print the NMEA sentence, or data
            // we end up not listening and catching other sentences!
            // so be very wary if using OUTPUT_ALLDATA and trying to print out data
            Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
           
            if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
              continue; // we can fail to parse a sentence in which case we should just wait for another
           
          }
          // Check this statement!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
          // Approximately every 2 seconds or so, print out the current stats
          if (millis() - timer > 2000)
          {
            timer = millis(); // reset the timer
            Serial.print("\nTime: ");
            if (GPS.hour < 10) { Serial.print('0'); }
            Serial.print(GPS.hour, DEC); Serial.print(':');
            if (GPS.minute < 10) { Serial.print('0'); }
            Serial.print(GPS.minute, DEC); Serial.print(':');
            if (GPS.seconds < 10) { Serial.print('0'); }
            Serial.print(GPS.seconds, DEC); Serial.print('.');
            if (GPS.milliseconds < 10) {
              Serial.print("00");
            } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
              Serial.print("0");
            }
            Serial.println(GPS.milliseconds);
            Serial.print("Date: ");
            Serial.print(GPS.day, DEC); Serial.print('/');
            Serial.print(GPS.month, DEC); Serial.print("/20");
            Serial.println(GPS.year, DEC);
            Serial.print("Fix: "); Serial.print((int)GPS.fix);
            Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
            if(GPS.fix)
            {
              Serial.println("We have a fix");
              Serial.print("Location: ");
              Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
              Serial.print(", ");
              Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

              // Concatenating GPS data to send everything all at once
              /*
              sprintf(latt, "%f", GPS.latitude);
              coordinates[0] = latt[0];
              coordinates[1] = latt[1];
              coordinates[2] = latt[2];
              coordinates[3] = latt[3];

              sprintf(smallLatt, "%f", GPS.lat);
             
              strcat(coordinates, smallLatt);
              strcat(coordinates, ", ");

              sprintf(longi, "%f", GPS.longitude);

              tempCoord[0] = longi[0];
              tempCoord[1] = longi[1];
              tempCoord[2] = longi[2];
              tempCoord[3] = longi[3];

              strcat(coordinates, tempCoord);
              sprintf(smallLongi, "%f", GPS.lon);
              strcat(coordinates, smallLongi);

              Serial.print("We got: ");
              Serial.println(coordinates);
              */

              // Send lat and long data to BLE
              //ble.print("AT+BLEUARTTX=");
              //ble.println(coordinates);
              ble.print(GPS.latitude, 4); ble.print(GPS.lat);
              ble.print(", ");
              ble.print(GPS.longitude, 4); ble.println(GPS.lon);
             
              Serial.print("Speed (knots): "); Serial.println(GPS.speed);
              Serial.print("Angle: "); Serial.println(GPS.angle);
              Serial.print("Altitude: "); Serial.println(GPS.altitude);
              Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
              break;
            }
          }
        }
      }
    }
   
    // Button 4 is reserved for speaker commands
    // The speaker is operating off of digitial pin 2
    if (buttnum == 4)
    {
      if(pressed)
      {
        tone(2, 800, (1000/2));
        delay(900);
        noTone(2);
        tone(2, 800, (1000/2));
        delay(900);
        noTone(2);
        tone(2, 800, (1000/2));
        delay(900);
        noTone(2);

        delay(1500);
        tone(2, 900, (1000/4));
        delay(500);
        noTone(2);
 
        tone(2, 800, (1000/2));
        delay(1000);
        noTone(2);
        tone(2, 800, (1000/2));
        delay(1000);
        noTone(2);
        tone(2, 800, (1000/2));
        delay(1000);
        noTone(2);
      }
    }
  }
}
