// Si5351 powered digital radio beacon. WeMos D1 Mini.
//
// Author: Dhiru Kholia (VU3CER), 07th-April-2023.
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

/*
  - Start in AP mode and stay there.
  - Allow time sync to happen before TX can take place.
  - Time is pushed to the beacon (from an Android phone perhaps).
*/


#include <Arduino.h>

#include <JTEncode.h>
#include <FS.h>
#include <Wire.h>
#include <si5351.h>
#include <TimeLib.h>
#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <rs_common.h>
#include <int.h>
#include <string.h>
#include "Wire.h"
#include <ESPAsyncWebServer.h>
#include "AsyncJson.h"
#include "ArduinoJson.h"
#include <JTEncode.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

// WiFi credentials
#include "credentials.h"

int time_is_synchronized = 0;

// https://www.qsl.net/yo4hfu/SI5351.html says,
//
// If it is necessary, frequency correction must to be applied. My Si5351
// frequency was too high, correction used 1.787KHz at 10MHz. Open again
// Si5351Example sketch, set CLK_0 to 1000000000 (10MHz). Upload. Connect a
// accurate frequency counter to CLK_0 output pin 10. Correction factor:
// (Output frequency Hz - 10000000Hz) x 100. Example: (10001787Hz - 10000000Hz)
// x 100 = 178700 Note: If output frequency is less than 10MHz, use negative
// value of correction, example -178700.
int32_t si5351CalibrationFactor = 42000;  // si5351 calibration factor

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
#define WSPR_TONE_SPACING 146 // ~1.46 Hz
#define WSPR_DELAY 683    // Delay value for WSPR
#define WSPR_DEFAULT_FREQ 14097050UL

uint64_t frequency = WSPR_DEFAULT_FREQ;

enum OperatingModes
{
  MODE_WSPR,
  MODE_FT8,
  MODE_FT4,
};

Si5351 si5351;
JTEncode jtencode;
char message[] = "VU3CER VU3FOE MK68";
char call[] = "VU3FOE";
char loc[] = "MK68";
uint8_t dbm = 20;
uint8_t tx_buffer[255];  // NOTE
int vfo_ok = 1;
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
  WSPR,
};

int beacon_enabled = 1;
enum modes mode = WSPR; // Note
enum codes action = NOP;
#define str(x) #x
#define xstr(x) str(x)
const long utcOffsetInSeconds = (5.5 * 3600);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
AsyncWebServer server(80);
WiFiUDP Udp;

// common global states
char txMessage[100] = "";                 // tx message
char myCallsign[10] = "VU3FOE";
char dxCallsign[10] = "VU3CER";
char myGridLocator[10] = "MK68";
uint8_t txBuffer[255];
uint8_t symbolCount;
uint16_t toneDelay, toneSpacing;
char IP[16] = "0.0.0.0";

time_t current_time = 0;

void sync_time_with_ntp()
{
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  setTime(now);
  Serial.println(timeClient.getFormattedTime());
  led_flash();
}

time_t sync_time_callback()
{
  sync_time_with_ntp();
  return 0;
}

// Loop through the string, transmitting one character at a time.
void jtTransmitMessage()
{
  uint8_t i;

  Serial.println("TX!");
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
}

void setTxBuffer()
{
  // Clear out the transmit buffer
  memset(txBuffer, 0, sizeof(txBuffer));

  jtencode.wspr_encode(myCallsign, myGridLocator, dbm, txBuffer);
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
  else if (var == "FREQUENCY")
  {
    return String(frequency);
  }
  else if (var == "CALIBRATION")
  {
    return String(si5351CalibrationFactor);
  }
  else if (var == "SYNCHRONIZED")
  {
    if (time_is_synchronized) {
      return "Yes";
    } else {
      return "No";
    }
  }
  else if (var == "CURRENT_WSPR_MESSAGE")
  {
    // Power level, [Pwr] is taken as a value from 0 – 60. Although only
    // certain values will work with the WSJT / WSPR software (just those
    // ending in  0, 3 or 7).
    if (dbm == 20) {
      return "Default WSPR Message";
    } else if (dbm == 30) {
      return "I am OK";
    } else if (dbm == 33) {
      return "I am NOT OK";
    } else if (dbm == 37) {
      return "Please check on me";
    } else {
      return "Unknown!";
    }
  }
  else if (var == "TIME")
  {
    return getTime();
  }
  else if (var == "MODE")
  {
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

void set_mode()
{
  if (mode == WSPR) {
    frequency = WSPR_DEFAULT_FREQ;;
    symbolCount = WSPR_SYMBOL_COUNT;
    toneSpacing = WSPR_TONE_SPACING;
    toneDelay = WSPR_DELAY;
    setTxBuffer();
    Serial.println("Getting ready for WSPR...");
  }
}

String getTime()
{
  time_t time_now = now();
  return ctime(&time_now);
}

void setup()
{
  int ret = 0;

  // Safety first!
  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);

  // Setup serial and IO pins
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  // delay(5000);
  Serial.println("BeaconPlusAP booting up...");

  // Initialize the Si5351
  ret = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, si5351CalibrationFactor);
  if (ret != true) {
    Serial.flush();
    vfo_ok = 0;
  }
  si5351.set_clock_pwr(SI5351_CLK0, 0); // safety first
  Serial.print("Si5351 init status (should be 1 always) = ");
  Serial.println(ret);

  // AP stuff ("Captive Portal")
  const char* ap_ssid = "beacon";
  const char* ap_password = "password";
  uint8_t max_connections = 8;

  if (WiFi.softAP(ap_ssid, ap_password, 1, false, max_connections) == true)
  {
    Serial.print("Access Point is Created with SSID: ");
    Serial.println(ap_ssid);
    Serial.print("Max Connections Allowed: ");
    Serial.println(max_connections);
    Serial.print("Access Point IP: ");
    Serial.println(WiFi.softAPIP());
    if (MDNS.begin("beacon")) {
      Serial.println("MDNS started");
    }
  } else {
    Serial.println("Unable to Create Access Point!");
  }

  // Set CLK0 output
  si5351.set_freq(frequency * 100ULL, SI5351_CLK0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for maximum power
  // delay(10000); // Keep TX on for 5 seconds for tunining purposes.
  si5351.set_clock_pwr(SI5351_CLK0, 0); // Disable the clock initially

  // Note
  set_mode();

  // Sanity checks
  if (!vfo_ok) {
    Serial.println("Check VFO connections!");
    led_flash();
    delay(50);
  }

  // Initialize SPIFFS
  if (!SPIFFS.begin())
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    led_flash();
    delay(50);
  }

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/sync_time.cgi", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    if (request->hasArg("epoch")) {
      String epoch = request->arg("epoch");
      current_time = strtoul(epoch.c_str(), NULL, 0);
      setTime(current_time);
      printf(ctime(&current_time));
      time_is_synchronized = 1;
    }
    request->redirect("/");

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
    time_t time_now = now();
    request->send_P(200, "text/plain", ctime(&time_now));
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

  server.on("/mode_wspr", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    mode = WSPR;
    action = MODE_SYNC;
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
  server.on("/set_message", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    String new_message;
    if (request->hasParam("message")) {
      new_message = request->getParam("message")->value();
      dbm = strtoul(new_message.c_str(), NULL, 10);
      setTxBuffer();
    }
    request->redirect("/");
  });

  server.on("/set_calibration", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    String new_calibration;
    if (request->hasParam("calibration")) {
      new_calibration = request->getParam("calibration")->value();
      si5351CalibrationFactor = strtoul(new_calibration.c_str(), NULL, 10);
      si5351.set_correction(si5351CalibrationFactor, SI5351_PLL_INPUT_XO);
    }
    request->redirect("/");
  });

  server.begin();
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Working in Webserver mode...");

  Serial.print("IP Address: ");
  strcpy(IP, WiFi.localIP().toString().c_str());
  Serial.println(IP);
  Serial.printf("TX frequency is %" PRId64 "\n", frequency);
}

// main loop
void loop()
{
  delay(100);
  char c;
  if (Serial.available() > 0) {
    c = Serial.read();
    Serial.println(c);
    if (c == 't') {
      jtTransmitMessage();
    }
  }

  // https://github.com/PaulStoffregen/Time
  if (second() == 0 && (minute() % 2 == 0)) { // production
    if (time_is_synchronized) {
      jtTransmitMessage();
    }
    else {
      led_flash();
      Serial.println("NOT TX'ing. Time is NOT synchronized!");
    }
  }

  if (action == LED_FLASH)
  {
    led_flash();
    action = NOP;
  }
  else if (action == NTP_SYNC)
  {
    timeClient.update();
    unsigned long now = timeClient.getEpochTime();
    led_flash();
    action = NOP;
  }
  else if (action == FREQUENCY_SYNC)
  {
    // eeprom.eeprom_write(FREQUENCY_OFFSET, frequency);
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

  delay(100);
}
