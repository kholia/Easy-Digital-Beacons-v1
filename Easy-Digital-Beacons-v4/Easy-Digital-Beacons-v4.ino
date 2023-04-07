// Si5351 powered digital radio beacon. WeMos D1 Mini.
//
// Author: Dhiru Kholia (VU3CER), 21th-October-2021.
//
// Borrowed from:
// - https://github.com/iw5ejm/multibandWSPR_nodeMCU/blob/master/multibandWSPR_nodeMCU.ino
// - https://randomnerdtutorials.com/esp8266-web-server-spiffs-nodemcu/
// - https://randomnerdtutorials.com/esp8266-relay-module-ac-web-server/
// - https://randomnerdtutorials.com/esp8266-0-96-inch-oled-display-with-arduino-ide/
// - https://www.bakke.online/index.php/2017/06/02/self-updating-ota-firmware-for-esp8266/
// - https://github.com/anupamsaikia
//
// Required Libraries
// ------------------
// Etherkit Si5351 (Library Manager)
// Etherkit JTEncode (Library Manager)
// Time (Library Manager)
// Wire (Arduino Standard Library)
// RTClib
//
// https://www.arduinoslovakia.eu/application/timer-calculator

/*
  HTTP server stuff is borrowed from:

  Rui Santos
  Complete project details at https://RandomNerdTutorials.com

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/


#include <Arduino.h>

#include <JTEncode.h>
#include <FS.h>
#include <Wire.h>
#include <RTClib.h>
#include "si5351.h"
#include <TimeLib.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <uEEPROMLib.h>
#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <rs_common.h>
#include <int.h>
#include <string.h>
#include "Wire.h"
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include "AsyncJson.h"
#include "ArduinoJson.h"
#include "SSD1306Wire.h"
#include "FT8.h"
#include "MyFont.h"

// WiFi credentials
#include "credentials.h"

// #define ENABLE_WSTJX_MODE 1  // FT8, FT4, and WSPR all work without any problems in WSJT-X mode!

// https://www.qsl.net/yo4hfu/SI5351.html says,
//
// If it is necessary, frequency correction must to be applied. My Si5351
// frequency was too high, correction used 1.787KHz at 10MHz. Open again
// Si5351Example sketch, set CLK_0 to 1000000000 (10MHz). Upload. Connect a
// accurate frequency counter to CLK_0 output pin 10. Correction factor:
// (Output frequency Hz - 10000000Hz) x 100. Example: (10001787Hz - 10000000Hz)
// x 100 = 178700 Note: If output frequency is less than 10MHz, use negative
// value of correction, example -178700.


// https://www.esp8266.com/wiki/doku.php?id=esp8266_gpio_pin_allocations
// The built-in led is on pin D4, and it is inverted.

// Global defines
#define PTT_PIN 14 // D5
// #define TX_LED_PIN 12 // D6
#define BUTTON_PIN D7
#define EEPROM_MODE_OFFSET 0
#define EEPROM_DELTA_OFFSET 4
#define EEPROM_TIME_DELTA_HACK_ENABLED_OFFSET 12
#define FREQUENCY_OFFSET 16
#define DELTA_DEFAULT 400


// Digital mode properties
#pragma region DigitalModeProperties
#define WSPR_TONE_SPACING 146 // ~1.46 Hz
#define FT8_TONE_SPACING 625  // ~6.25 Hz

#define WSPR_DELAY 683    // Delay value for WSPR
#define FT8_DELAY 159     // Delay value for FT8

#define WSPR_DEFAULT_FREQ 14097050UL
// #define FT8_DEFAULT_FREQ 14075000UL
#define FT8_DEFAULT_FREQ 21075000UL
#define FT4_DEFAULT_FREQ 14081000UL
#pragma endregion DigitalModeProperties

// Enumerations
#pragma region Enums
enum DeviceModes
{
  STANDALONE,
  WEBSERVER,
  WSJTX
};

const String deviceModeTexts[] = {
  "Standalone",
  "Webserver",
  "WSJT-X",
};

enum OperatingModes
{
  MODE_WSPR,
  MODE_FT8,
  MODE_FT4,
};

const String operatingModeTexts[] = {
  "WSPR",
  "FT8",
  "FT4",
};
#pragma endregion Enums

// Class instantiations
int buttonState = 0;
RTC_DS3231 rtc;
Si5351 si5351;
DateTime dt;
JTEncode jtencode;
uEEPROMLib eeprom(0x57); // I2C EEPROM - incorporated on DS3231 RTC modules
char message[] = "VU3CER VU3FOE MK68";
char call[] = "VU3FOE";
char loc[] = "MK68";
uint8_t dbm = 27;
uint8_t tx_buffer[255];  // NOTE
int rtc_lost_power = 0;
int vfo_ok = 1;
double delta = DELTA_DEFAULT;
uint32_t time_delta_hack_enabled = 0;
int beacon_enabled = 0;
const int ledPin = LED_BUILTIN;
enum codes
{
  NOP,
  LED_FLASH,
  NTP_SYNC,
  TUNE,
  TUNE_LONG,
  TUNE_POWER,
  MODE_SYNC,
  DELTA_SYNC,
  TIME_DELTA_HACK_SYNC,
  FREQUENCY_SYNC
};
enum modes
{
  FT8,
  FT4,
  WSPR,
};
enum modes mode = FT8; // Note
enum codes action = NOP;
#define str(x) #x
#define xstr(x) str(x)
const long utcOffsetInSeconds = (5.5 * 3600);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
AsyncWebServer server(80);
SSD1306Wire display(0x3c, SDA, SCL);
WiFiUDP Udp;

// common global states
#pragma region Common_Global_States
#ifdef ENABLE_WSTJX_MODE
DeviceModes deviceMode = WSJTX;      // default device mode is standalone
#else
DeviceModes deviceMode = STANDALONE;
#endif
OperatingModes operatingMode = MODE_FT8;   // default op mode is CW
uint64_t frequency = FT8_DEFAULT_FREQ;
int32_t si5351CalibrationFactor = 16999;  // si5351 calibration factor
boolean txEnabled = false;                // flag to denote if transmit is enabled or disabled
char txMessage[100] = "";                 // tx message
char myCallsign[10] = "VU3FOE";
char dxCallsign[10] = "VU3CER";
char myGridLocator[10] = "MK68";
uint8_t dBm = 30;
uint8_t txBuffer[255];
uint8_t symbolCount;
uint16_t toneDelay, toneSpacing;
char IP[16] = "0.0.0.0";
boolean refreshDisplay = false;

// Loop through the string, transmitting one character at a time.
void jtTransmitMessage()
{
  uint8_t i;

  Serial.println("TX!");
  txEnabled = true;
  // updateDisplay();
  // Reset the tone to the base frequency and turn on the output
  si5351.set_clock_pwr(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK0, 1);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PTT_PIN, HIGH); // Note

  for (i = 0; i < symbolCount; i++)
  {
    si5351.set_freq(frequency * 100ULL + (txBuffer[i] * toneSpacing), SI5351_CLK0);
    delay(toneDelay);
  }

  // Turn off the output
  si5351.set_clock_pwr(SI5351_CLK0, 0);
  si5351.output_enable(SI5351_CLK0, 0);
  digitalWrite(PTT_PIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
  txEnabled = false;
  // updateDisplay();
}

void setTxBuffer()
{
  // Clear out the transmit buffer
  memset(txBuffer, 0, sizeof(txBuffer));

  // Set the proper frequency and timer CTC depending on mode
  switch (operatingMode)
  {
    case MODE_WSPR:
      jtencode.wspr_encode(myCallsign, myGridLocator, dBm, txBuffer);
      break;
  }
}

String getTemperature()
{
  return String(rtc.getTemperature());
}

String getTime()
{
  char date[10] = "hh:mm:ss";
  rtc.now().toString(date);

  return date;
}

// Replaces placeholders in HTML code
String processor(const String &var)
{
  // Serial.println(var);
  if (var == "STATE")
  {
    if (digitalRead(ledPin))
    {
      return "OFF";
    }
    else
    {
      return "ON";
    }
  }
  else if (var == "RTC_LOST_POWER_STATE")
  {
    if (rtc_lost_power)
    {
      return "Yes";
    }
    else
    {
      return "No";
    }
  }
  else if (var == "VFO_OK")
  {
    if (vfo_ok)
    {
      return "Yes";
    }
    else
    {
      return "No";
    }
  }
  else if (var == "TIME_DELTA_HACK_ENABLED")
  {
    if (time_delta_hack_enabled)
    {
      return "Yes";
    }
    else
    {
      return "No";
    }
  }
  else if (var == "FT8_MESSAGE")
  {
    return String(message);
  }
  else if (var == "FREQUENCY")
  {
    return String(frequency);
  }
  else if (var == "DELTA")
  {
    return String(delta);
  }
  else if (var == "TEMPERATURE")
  {
    return getTemperature();
  }
  else if (var == "TIME")
  {
    return getTime();
  }
  else if (var == "MODE")
  {
    if (mode == FT8)
      return String(xstr(FT8));
    else if (mode == FT4)
      return String(xstr(FT4));
    else if (mode == WSPR)
      return String(xstr(WSPR));
  }

  return "FAIL";
}

void AnnounceServices()
{
  MDNS.announce();
}

// debug helper
void led_flash()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
  digitalWrite(LED_BUILTIN, HIGH);
}


// WSJTX UDP
#pragma region WSJTX
const unsigned int localUdpPort = 2237; // local port to listen on
uint8_t WSJTX_incomingByteArray[255];   // buffer for incoming packets
size_t WSJTX_currentIndex = 0;

// WSJTX helper functions
uint8 readuInt8()
{
  uint8 val;
  memcpy(&val, &WSJTX_incomingByteArray[WSJTX_currentIndex], sizeof(val));
  WSJTX_currentIndex += 1;
  return val;
}

uint32 readuInt32()
{
  uint32 bigEndianValue;
  memcpy(&bigEndianValue, &WSJTX_incomingByteArray[WSJTX_currentIndex], sizeof(bigEndianValue));
  WSJTX_currentIndex += 4;
  uint32 theUnpackedValue = ntohl(bigEndianValue);
  return theUnpackedValue;
}

int32 readInt32()
{
  int32 bigEndianValue;
  memcpy(&bigEndianValue, &WSJTX_incomingByteArray[WSJTX_currentIndex], sizeof(bigEndianValue));
  WSJTX_currentIndex += 4;
  int32 theUnpackedValue = ntohl(bigEndianValue);
  return theUnpackedValue;
}

uint64 readuInt64()
{
  uint64 bigEndianValue;
  memcpy(&bigEndianValue, &WSJTX_incomingByteArray[WSJTX_currentIndex], sizeof(bigEndianValue));
  WSJTX_currentIndex += 8;
  uint64 theUnpackedValue = __builtin_bswap64(bigEndianValue);
  return theUnpackedValue;
}

bool readBool()
{
  bool val;
  memcpy(&val, &WSJTX_incomingByteArray[WSJTX_currentIndex], sizeof(val));
  WSJTX_currentIndex += 1;
  return val;
}

#pragma endregion WSJTX

// Display functionality
#pragma region Display

// Primary frame. Shows the most important device states
void showScreen1()
{
  display.clear();

  display.setFont(Roboto_Mono_Thin_16);
  display.drawString(0, 0, String((double)frequency / 100000000, 6U) + "MHz");

  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 20, "Mode: " + deviceModeTexts[deviceMode]);
  display.drawString(0, 30, "OpMode: " + operatingModeTexts[operatingMode]);
  display.drawString(0, 40, "IP: " + String(IP));
  display.drawString(0, 50, "TxEnabled: " + String(txEnabled ? "true" : "false"));
  display.display();
}

void showScreenWSJTX()
{
  display.clear();

  display.setFont(Roboto_Mono_Thin_16);
  display.drawString(0, 0, String((double)frequency / 100000000, 6U) + "MHz");

  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 20, "DeviceMode: " + deviceModeTexts[deviceMode]);
  display.drawString(0, 30, "OpMode: " + operatingModeTexts[operatingMode]);
  if (operatingMode == MODE_WSPR)
  {
    display.drawString(0, 40, myCallsign + String(" ") + myGridLocator + String(" ") + String(dBm));
  }
  else
  {
    display.drawString(0, 40, String(txMessage));
  }
  display.drawString(0, 50, "TxEnabled: " + String(txEnabled ? "true" : "false"));

  display.display();
}

void updateDisplay()
{
  if (deviceMode == WSJTX)
  {
    showScreenWSJTX();
  }
  else
  {
    showScreen1();
  }
}

#pragma endregion Display

void set_mode()
{
  eeprom.eeprom_read(EEPROM_MODE_OFFSET, &mode);
  if (mode != FT8 && mode != FT4 && mode != WSPR) {
    Serial.println("EEPROM has corrupt mode setting, do a save ;)");
    mode = FT8;
  }
  eeprom.eeprom_read(EEPROM_DELTA_OFFSET, &delta);
  if (delta != delta) { // NaN
    delta = DELTA_DEFAULT;
  }
  eeprom.eeprom_read(EEPROM_TIME_DELTA_HACK_ENABLED_OFFSET, &time_delta_hack_enabled);
  if (time_delta_hack_enabled != 0 && time_delta_hack_enabled != 1) {
    time_delta_hack_enabled = 0;
  }

  if (mode == WSPR) {
    frequency = WSPR_DEFAULT_FREQ;;
    symbolCount = WSPR_SYMBOL_COUNT;
    toneSpacing = WSPR_TONE_SPACING;
    toneDelay = WSPR_DELAY;
    operatingMode = MODE_WSPR;
    setTxBuffer();
    Serial.println("Getting ready for WSPR...");
  } else if (mode == FT8) {
    frequency = FT8_DEFAULT_FREQ;
    symbolCount = FT8_SYMBOL_COUNT;
    toneSpacing = FT8_TONE_SPACING;
    toneDelay = FT8_DELAY;
    operatingMode = MODE_FT8;
    ftx_encode(message, txBuffer, false);
  } else if (mode == FT4) {
    frequency = FT4_DEFAULT_FREQ;
    symbolCount = 105;
    toneSpacing = 2083.3333; // ~20.83 Hz
    toneDelay = 47;
    operatingMode = MODE_FT4;
    ftx_encode(message, txBuffer, true);
    Serial.print("FSK tones: ");
    for (int j = 0; j < symbolCount; ++j) {
      Serial.print(tx_buffer[j]);
    }
    Serial.println("\n");
  }
}

void setup()
{
  int ret = 0;
  char date[10] = "hh:mm:ss";

  // Safety first!
  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);

  // Setup serial and IO pins
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  delay(2000);
  Serial.println("\n...");

  // Initialize the Si5351
  ret = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, si5351CalibrationFactor);
  if (ret != true) {
    Serial.flush();
    vfo_ok = 0;
  }
  si5351.set_clock_pwr(SI5351_CLK0, 0); // safety first
  Serial.print("Si5351 init status (should be 1 always) = ");
  Serial.println(ret);

  // Initialize the rtc
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC!");
    Serial.flush();
    // abort();
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, abort!?");
    Serial.flush();
    rtc_lost_power = 1;
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // hack!
    // abort();  // NOTE
  }
  rtc.disable32K();
  rtc.now().toString(date);

  // Print status
  Serial.print("Current time is = ");
  Serial.println(date);
  Serial.print("Temperature is: ");
  Serial.print(rtc.getTemperature());
  Serial.println(" C");

  // Set CLK0 output
  si5351.set_freq(frequency * 100ULL, SI5351_CLK0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for maximum power
  // delay(10000); // Keep TX on for 5 seconds for tunining purposes.
  si5351.set_clock_pwr(SI5351_CLK0, 0); // Disable the clock initially

  // Note
  set_mode();

  // Read saved frequency
  eeprom.eeprom_read(FREQUENCY_OFFSET, &frequency);
  if (frequency < 3000000UL or frequency > 60000000UL) {
    Serial.println("EEPROM has corrupt 'frequency' setting, do a save ;)");
    if (mode == WSPR) {
      frequency = WSPR_DEFAULT_FREQ;
    } else if (mode == FT8) {
      frequency = FT8_DEFAULT_FREQ;
    } else if (mode == FT4) {
      frequency = FT4_DEFAULT_FREQ;
    }
  }

  // Figure out the mode to run
  buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == HIGH) {
    // Connect to Wi-Fi
    WiFi.hostname("beacon");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(1000);
      Serial.println("Connecting to WiFi...");
    }
    Serial.println(WiFi.localIP());
    if (MDNS.begin("beacon"))
    {
      Serial.println("MDNS started");
    }
    timeClient.begin();
  }


  // Sanity checks
  if (!vfo_ok) {
    Serial.println("Check VFO connections!");
    led_flash();
    delay(50);
  }
  if (rtc_lost_power) {
    Serial.println("Check and set RTC time!");
    led_flash();
    delay(50);
  }

  if (buttonState == HIGH || rtc_lost_power || !vfo_ok) {
    beacon_enabled = 0;


    // Initialize SPIFFS
    if (!SPIFFS.begin())
    {
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }

    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      request->send(SPIFFS, "/index.html", String(), false, processor);
    });

    // Route to load style.css file
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      request->send(SPIFFS, "/style.css", "text/css");
    });

    // Route to load favicon.ico file
    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      request->send(SPIFFS, "/favicon.ico", "image/x-icon");
    });

    // Route to set GPIO to LOW (LED ON)
    server.on("/on", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      digitalWrite(ledPin, LOW);
      // request->send_P(200, "text/plain", "OK");
      // request->send(SPIFFS, "/index.html", String(), false, processor);
      request->redirect("/");
    });
    // Route to set GPIO to HIGH (LED OFF)
    server.on("/off", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      digitalWrite(ledPin, HIGH);
      // request->send_P(200, "text/plain", "OK");
      // request->send(SPIFFS, "/index.html", String(), false, processor);
      request->redirect("/");
    });
    server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      request->send_P(200, "text/plain", getTemperature().c_str());
    });

    // Custom
    server.on("/led_flash", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      action = LED_FLASH;
      request->send(SPIFFS, "/index.html", String(), false, processor);
    });
    server.on("/nop", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      action = NOP;
      request->send(SPIFFS, "/index.html", String(), false, processor);
    });

    server.on("/tx_on", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      // action = TX_ON;
      beacon_enabled = 1;
      request->send(SPIFFS, "/index.html", String(), false, processor);
    });
    server.on("/tx_off", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      action = NOP;
      beacon_enabled = 0;
      request->send(SPIFFS, "/index.html", String(), false, processor);
    });

    server.on("/ntp_sync", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      action = NTP_SYNC;
      request->redirect("/");
    });

    server.on("/ptt_on", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      digitalWrite(PTT_PIN, HIGH);
      request->redirect("/");
    });
    server.on("/ptt_off", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      digitalWrite(PTT_PIN, LOW);
      request->redirect("/");
    });

    server.on("/time", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      // DateTime timestamp = rtc.now();
      char date[10] = "hh:mm:ss";
      rtc.now().toString(date);
      request->send_P(200, "text/plain", date);
    });

    server.on("/delta_plus", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      delta = delta + 25;
      action = DELTA_SYNC;
      request->redirect("/");
    });
    server.on("/delta_minus", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      delta = delta - 25;
      if (delta < 0)
        delta = 0;
      action = DELTA_SYNC;
      request->send(SPIFFS, "/index.html", String(), false, processor);
    });

    server.on("/tune", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      action = TUNE;
      request->redirect("/");
    });

    server.on("/tune_long", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      action = TUNE_LONG;
      request->redirect("/");
    });

    server.on("/tune_power", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      action = TUNE_POWER;
      request->redirect("/");
    });

    server.on("/mode_ft8", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      mode = FT8;
      action = MODE_SYNC;
      request->redirect("/");
    });
    server.on("/mode_ft4", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      mode = FT4;
      action = MODE_SYNC;
      request->redirect("/");
    });
    server.on("/mode_wspr", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      mode = WSPR;
      action = MODE_SYNC;
      request->redirect("/");
    });
    server.on("/time_delta_hack_enable", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      time_delta_hack_enabled = 1;
      action = TIME_DELTA_HACK_SYNC;
      request->redirect("/");
    });
    server.on("/time_delta_hack_disable", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      time_delta_hack_enabled = 0;
      action = TIME_DELTA_HACK_SYNC;
      request->redirect("/");
    });
    server.on("/set_frequency", HTTP_GET, [](AsyncWebServerRequest * request)
    {
      String new_frequency;
      if (request->hasParam("frequency")) {
        new_frequency = request->getParam("frequency")->value();
        frequency = strtoul(new_frequency.c_str(), NULL, 10);
        action = FREQUENCY_SYNC;
      }
      request->redirect("/");
    });

    // timer.setInterval(1 * 1000, AnnounceServices);
    server.begin();
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Working in Webserver + WSJT-X mode...");
  }
  else {
    beacon_enabled = 1;
    delay(3000);
    Serial.print("Working in beacon mode -> ");
    // Serial.println(processor("MODE"));
    led_flash();
  }

  Serial.print("IP Address: ");
  strcpy(IP, WiFi.localIP().toString().c_str());
  Serial.println(IP);
  Serial.printf("TX frequency is %" PRId64 "\n", frequency);

#ifdef ENABLE_WSTJX_MODE
  // Begin UDP Listener
  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", IP, localUdpPort);
#endif

  // Initialising the UI
  /* display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    updateDisplay(); */
}

// main loop
void loop()
{
  // updateDisplay();
  delay(100);

#ifdef ENABLE_WSTJX_MODE
  // logic for device mode WSJTX
  // WSJTX message type: https://sourceforge.net/p/wsjt/wsjtx/ci/master/tree/Network/NetworkMessage.hpp#l141

  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    unsigned long now = millis();

    // receive incoming UDP packets
    int len = Udp.read(WSJTX_incomingByteArray, 255);
    if (len > 0)
    {
      WSJTX_currentIndex = 8; // skip packet header

      // Packet Type
      uint32 WSJTX_packetType = readuInt32();
      if (WSJTX_packetType == 1)
      {
        //--------------------------------------------------------------------//
        // Client id
        int32 WSJTX_clientIdLength = readInt32();
        char WSJTX_clientId[WSJTX_clientIdLength + 1];
        for (int32 i = 0; i < WSJTX_clientIdLength; i++)
        {
          WSJTX_clientId[i] = WSJTX_incomingByteArray[WSJTX_currentIndex];
          WSJTX_currentIndex += 1;
        }
        WSJTX_clientId[WSJTX_clientIdLength] = 0;

        //--------------------------------------------------------------------//
        // Dial Frequency
        uint64 WSJTX_dialFrequency = readuInt64();

        //--------------------------------------------------------------------//
        // Mode
        int32 WSJTX_modeLength = readInt32();
        char WSJTX_mode[WSJTX_modeLength + 1];
        for (int32 i = 0; i < WSJTX_modeLength; i++)
        {
          WSJTX_mode[i] = WSJTX_incomingByteArray[WSJTX_currentIndex];
          WSJTX_currentIndex += 1;
        }
        WSJTX_mode[WSJTX_modeLength] = 0;

        //--------------------------------------------------------------------//
        // DX Call
        int32 WSJTX_dxCallLength = readInt32();
        char WSJTX_dxCall[WSJTX_dxCallLength + 1];
        for (int32 i = 0; i < WSJTX_dxCallLength; i++)
        {
          WSJTX_dxCall[i] = WSJTX_incomingByteArray[WSJTX_currentIndex];
          WSJTX_currentIndex += 1;
        }
        WSJTX_dxCall[WSJTX_dxCallLength] = 0;

        //--------------------------------------------------------------------//
        // Report
        int32 WSJTX_reportLength = readInt32();
        char WSJTX_report[WSJTX_reportLength + 1];
        for (int32 i = 0; i < WSJTX_reportLength; i++)
        {
          WSJTX_report[i] = WSJTX_incomingByteArray[WSJTX_currentIndex];
          WSJTX_currentIndex += 1;
        }
        WSJTX_report[WSJTX_reportLength] = 0;

        //--------------------------------------------------------------------//
        // Tx mode
        int32 WSJTX_txModeLength = readInt32();
        char WSJTX_txMode[WSJTX_txModeLength + 1];
        for (int32 i = 0; i < WSJTX_txModeLength; i++)
        {
          WSJTX_txMode[i] = WSJTX_incomingByteArray[WSJTX_currentIndex];
          WSJTX_currentIndex += 1;
        }
        WSJTX_txMode[WSJTX_txModeLength] = 0;

        //--------------------------------------------------------------------//
        // Tx Enabled
        bool WSJTX_txEnabled = readBool();

        //--------------------------------------------------------------------//
        // Transmitting
        bool WSJTX_transmitting = readBool();

        //--------------------------------------------------------------------//
        // Decoding
        bool WSJTX_decoding = readBool();

        //--------------------------------------------------------------------//
        // Rx DF
        uint32 WSJTX_rxDF = readuInt32();

        //--------------------------------------------------------------------//
        // Tx DF
        uint32 WSJTX_txDF = readuInt32();

        //--------------------------------------------------------------------//
        // DE call
        int32 WSJTX_deCallLength = readInt32();
        char WSJTX_deCall[WSJTX_deCallLength + 1];
        for (int32 i = 0; i < WSJTX_deCallLength; i++)
        {
          WSJTX_deCall[i] = WSJTX_incomingByteArray[WSJTX_currentIndex];
          WSJTX_currentIndex += 1;
        }
        WSJTX_deCall[WSJTX_deCallLength] = 0;

        //--------------------------------------------------------------------//
        // DE grid
        int32 WSJTX_deGridLength = readInt32();
        char WSJTX_deGrid[WSJTX_deGridLength + 1];
        for (int32 i = 0; i < WSJTX_deGridLength; i++)
        {
          WSJTX_deGrid[i] = WSJTX_incomingByteArray[WSJTX_currentIndex];
          WSJTX_currentIndex += 1;
        }
        WSJTX_deGrid[WSJTX_deGridLength] = 0;

        //--------------------------------------------------------------------//
        // DX grid
        int32 WSJTX_dxGridLength = readInt32();
        char WSJTX_dxGrid[WSJTX_dxGridLength + 1];
        for (int32 i = 0; i < WSJTX_dxGridLength; i++)
        {
          WSJTX_dxGrid[i] = WSJTX_incomingByteArray[WSJTX_currentIndex];
          WSJTX_currentIndex += 1;
        }
        WSJTX_dxGrid[WSJTX_dxGridLength] = 0;

        //--------------------------------------------------------------------//
        // Tx Watchdog
        bool WSJTX_txWatchdog = readBool();

        //--------------------------------------------------------------------//
        // Sub-mode
        int32 WSJTX_subModeLength = readInt32();
        char WSJTX_subMode[WSJTX_subModeLength + 1];
        for (int32 i = 0; i < WSJTX_subModeLength; i++)
        {
          WSJTX_subMode[i] = WSJTX_incomingByteArray[WSJTX_currentIndex];
          WSJTX_currentIndex += 1;
        }
        WSJTX_subMode[WSJTX_subModeLength] = 0;

        //--------------------------------------------------------------------//
        // Fast mode
        bool WSJTX_fastMode = readBool();

        //--------------------------------------------------------------------//
        // Special Operation Mode
        uint8 WSJTX_specialOpMode = readuInt8();

        // Frequency Tolerance
        uint32 WSJTX_frequencyTolerance = readuInt32();

        //--------------------------------------------------------------------//
        // T/R Period
        uint32 WSJTX_txrxPeriod = readuInt32();

        //--------------------------------------------------------------------//
        // Configuration Name
        int32 WSJTX_configNameLength = readInt32();
        char WSJTX_configName[WSJTX_configNameLength + 1];
        for (int32 i = 0; i < WSJTX_configNameLength; i++)
        {
          WSJTX_configName[i] = WSJTX_incomingByteArray[WSJTX_currentIndex];
          WSJTX_currentIndex += 1;
        }
        WSJTX_configName[WSJTX_configNameLength] = 0;

        //--------------------------------------------------------------------//
        // Tx Message
        int32 WSJTX_txMessageLength = readInt32();
        // Funfact: While testing, I got WSJTX_txMessageLength=37 regardless of the actual message length.
        // Therefore, WSJTX_txMessage needs to be trimmed.
        char WSJTX_txMessage[WSJTX_txMessageLength + 1];
        for (int32 i = 0; i < WSJTX_txMessageLength; i++)
        {
          WSJTX_txMessage[i] = WSJTX_incomingByteArray[WSJTX_currentIndex];
          WSJTX_currentIndex += 1;
        }
        WSJTX_txMessage[WSJTX_txMessageLength] = 0;

        // set frequency
        frequency = (WSJTX_dialFrequency + WSJTX_txDF);

        // trim tx message
        String newTxMessage = String(WSJTX_txMessage);
        newTxMessage.trim();

        if (strcmp(WSJTX_mode, "FT8") == 0)
        {
          symbolCount = FT8_SYMBOL_COUNT;
          toneSpacing = FT8_TONE_SPACING;
          toneDelay = FT8_DELAY;
          operatingMode = MODE_FT8;
          txEnabled = WSJTX_txEnabled;
          strcpy(txMessage, newTxMessage.c_str());
        }
        else if (strcmp(WSJTX_mode, "FT4") == 0)
        {
          symbolCount = 105;
          toneSpacing = 2083.3333; // ~20.83 Hz
          toneDelay = 47;
          operatingMode = MODE_FT4;
          txEnabled = WSJTX_txEnabled;
          strcpy(txMessage, newTxMessage.c_str());
        }
        else if (strcmp(WSJTX_mode, "WSPR") == 0)
        {
          symbolCount = 160;
          toneSpacing = WSPR_TONE_SPACING;
          toneDelay = WSPR_DELAY;
          operatingMode = MODE_WSPR;
          txEnabled = WSJTX_txEnabled;
          strcpy(myCallsign, WSJTX_deCall);
          strcpy(myGridLocator, WSJTX_deGrid);
          // dbm value is not taken from WSJTX.
          // Enter correct dbm value from the web interface
        }
        else
        {
          txEnabled = false;
        }

        // update display
        updateDisplay();

        // transmit Message
        if (txEnabled && WSJTX_transmitting)
        {
          if (operatingMode == MODE_FT8)
          {
            ftx_encode(txMessage, txBuffer, false);
          }
          else if (operatingMode == MODE_FT4)
          {
            ftx_encode(txMessage, txBuffer, true);
          }
          else
          {
            setTxBuffer();
          }
          jtTransmitMessage();
          txEnabled = false;
        }
      }
    }
  }
#else  // STANDALONE
  if (beacon_enabled) {
    dt = rtc.now();
    if (time_delta_hack_enabled == 1 && (dt.second() % 15 == 14) && mode == FT8) { // Check for 14th, 29th, 44th, 59th second
      // if (time_delta_hack_enabled == 1 && (dt.second() == 14 || dt.second() == 44) && mode == FT8) { // TX 2 times every minute
      // if (time_delta_hack_enabled == 1 && (dt.second() == 14) && mode == FT8) { // TX once every minute
      delay(delta);
      jtTransmitMessage();
    }
    // FT4 -> 0, 7.5, 15, 22.5, 30, 37.5, 45, 52.5, 60
    if (time_delta_hack_enabled == 1 && (dt.second() % 7 == 0) && mode == FT4) { // Dirty hack but at least gives some decodes!
      delay(delta);
      jtTransmitMessage();
    } else if (time_delta_hack_enabled == 0 && (dt.second() % 15 == 0) && mode == FT8) {
      jtTransmitMessage();
    } else if (dt.second() == 1 && (dt.minute() % 2) == 0 && mode == WSPR) {
      jtTransmitMessage();
    }
  } else {
    if (action == LED_FLASH)
    {
      led_flash();
      action = NOP;
    }
    else if (action == NTP_SYNC)
    {
      timeClient.update();
      unsigned long now = timeClient.getEpochTime();
      rtc.adjust(now);
      led_flash();
      action = NOP;
    }
    else if (action == MODE_SYNC)
    {
      eeprom.eeprom_write(EEPROM_MODE_OFFSET, mode);
      led_flash();
      action = NOP;
    }
    else if (action == DELTA_SYNC)
    {
      eeprom.eeprom_write(EEPROM_DELTA_OFFSET, delta);
      led_flash();
      action = NOP;
    }
    else if (action == TIME_DELTA_HACK_SYNC)
    {
      eeprom.eeprom_write(EEPROM_TIME_DELTA_HACK_ENABLED_OFFSET, time_delta_hack_enabled);
      led_flash();
      action = NOP;
    }
    else if (action == FREQUENCY_SYNC)
    {
      eeprom.eeprom_write(FREQUENCY_OFFSET, frequency);
      led_flash();
      action = NOP;
    }
    else if (action == TUNE)
    {
      // Set CLK0 output
      si5351.set_freq(frequency * 100ULL, SI5351_CLK0);
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA); // Set for minimum power
      digitalWrite(LED_BUILTIN, LOW);
      si5351.set_clock_pwr(SI5351_CLK0, 1); // Enable the clock
      // si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power
      delay(7000); // Keep TX on for 10 seconds for tuning purposes.
      si5351.set_clock_pwr(SI5351_CLK0, 0); // Disable the clock
      digitalWrite(LED_BUILTIN, HIGH);
      action = NOP;
    }
    else if (action == TUNE_LONG)
    {
      si5351.set_freq(frequency * 100, SI5351_CLK0);
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(PTT_PIN, HIGH);
      si5351.set_clock_pwr(SI5351_CLK0, 1);
      delay(30000);
      si5351.set_clock_pwr(SI5351_CLK0, 0);
      digitalWrite(PTT_PIN, LOW);
      digitalWrite(LED_BUILTIN, HIGH);
      action = NOP;
    }
    else if (action == TUNE_POWER)
    {
      si5351.set_freq(frequency * 100, SI5351_CLK0);
      si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(PTT_PIN, HIGH);
      si5351.set_clock_pwr(SI5351_CLK0, 1);
      delay(7000);
      si5351.set_clock_pwr(SI5351_CLK0, 0);
      digitalWrite(PTT_PIN, LOW);
      digitalWrite(LED_BUILTIN, HIGH);
      action = NOP;
    }

    MDNS.update();
  }

  delay(100);
#endif
}

// end of loop
