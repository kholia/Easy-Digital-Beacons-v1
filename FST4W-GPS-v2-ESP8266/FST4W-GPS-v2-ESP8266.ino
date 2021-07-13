// Huge thanks to https://github.com/f4goh/ for the original FST4W work!
//
// Huge thanks to IK1HGI (Antonio) for introducing me to FST4W and the top band!
//
// Reference: https://physics.princeton.edu/pulsar/k1jt/FST4_Quick_Start.pdf

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <si5351.h>
#include <TimeLib.h>

// GPS stuff
#include "SoftwareSerial.h"
#include <TinyGPS++.h>
#define MYPORT_RX 13 // D7
TinyGPSPlus gps;
#define SW_SERIAL_UNUSED_PIN -1
SoftwareSerial myPort(MYPORT_RX, SW_SERIAL_UNUSED_PIN); // RX, TX -> GPS stuff FST4W timing

#define WITH_NTP_SUPPORT 1

#ifdef WITH_NTP_SUPPORT
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#endif

#define ENABLE_LCD_SUPPORT 1

#ifdef ENABLE_LCD_SUPPORT
#include "LiquidCrystal_I2C.h"
// https://randomnerdtutorials.com/esp32-esp8266-i2c-lcd-arduino-ide/
LiquidCrystal_I2C lcd(0x27, 16, 2);
#endif

#ifdef ENABLE_OLED_SUPPORT
#include <SSD1306Wire.h>
#endif
#include "MyFont.h"

#include "credentials.h"

// Global defines
#define PTT_PIN 14 // D5
#define IS_GPS_SYNCED_PIN 12 // D6

// Use https://github.com/etherkit/Si5351Arduino/blob/master/examples/si5351_calibration/si5351_calibration.ino
// to derive calibration value below!
int32_t si5351CalibrationFactor = 16999;  // si5351 calibration factor

// #define FST4W_DEFAULT_FREQ  473450UL // 472 KHz band (630m band)! Tune receiver to 472.000 KHz USB!
#define FST4W_DEFAULT_FREQ  137450UL // Antonio's frequency
// #define FST4W_DEFAULT_FREQ  14097050UL  // 20 meter band for testing
#define FST4W_SYMBOL_COUNT  160

// fst4sim "IK1HGI JN45 20" 120 1500 0.0 0.1 1.0 10 -15 F
const uint8_t  FST4Wsymbols[FST4W_SYMBOL_COUNT] = { 0, 1, 3, 2, 1, 0, 2, 3, 3, 0, 3, 3, 0, 3, 1, 0, 0, 2, 2, 1, 3, 1, 3, 0, 3, 1, 0, 1, 0, 3, 1, 3, 0, 0, 3, 0, 1, 1, 2, 3, 1, 0, 3, 2, 0, 1, 0, 3, 1, 2, 1, 2, 3, 2, 1, 1, 0, 0, 1, 3, 3, 1, 3, 0, 0, 2, 3, 1, 2, 0, 3, 0, 2, 2, 1, 2, 0, 1, 3, 2, 1, 0, 2, 3, 0, 3, 0, 3, 2, 2, 0, 1, 0, 2, 0, 1, 0, 0, 2, 2, 3, 1, 3, 0, 2, 1, 1, 3, 0, 1, 0, 3, 2, 2, 2, 3, 1, 0, 3, 2, 0, 1, 0, 0, 3, 2, 1, 2, 0, 2, 2, 0, 3, 0, 0, 1, 3, 3, 3, 0, 0, 0, 1, 3, 2, 1, 1, 0, 0, 2, 0, 2, 0, 1, 3, 2, 1, 0, 2, 3 };

// Note - Change me for using a different FST4W mode!
int FST4W_MODE = 0;   // FST4W 120
// 0 = FST4W 120
// 1 = FST4W 300
// 2 = FST4W 900
// 3 = FST4W 1800

float toneSpacing[4] = {
  1.464,   // 120
  0.558,   // 300
  0.1803,  // 900
  0.08929  // 1800
};


// Load WSPR/FST4W symbol length
unsigned int symbolLength[4] = {
  683,    // 120
  1792,   // 300
  5547,   // 900
  11200   // 1800
};

enum OperatingModes
{
  MODE_FST4W,
};

const String operatingModeTexts[] = {
  "FST4W",
};

// Class instantiations
boolean txEnabled = false;

#ifdef ENABLE_OLED_SUPPORT
SSD1306Wire display(0x3c, SDA, SCL);
#endif
RTC_DS3231 rtc;
Si5351 si5351;
DateTime dt;
unsigned long freq;
int rtc_lost_power = 0;
int vfo_ok = 1;
const int ledPin = LED_BUILTIN;
enum modes
{
  FST4W,
};
enum modes mode = FST4W;
OperatingModes operatingMode = MODE_FST4W;
uint64_t frequency = FST4W_DEFAULT_FREQ * 100ULL;
uint8_t symbolCount;

void updateDisplay()
{
  char sub_mode[2];
#ifdef ENABLE_OLED_SUPPORT
  display.clear();
  display.setFont(Roboto_Mono_Thin_16);
  display.drawString(0, 0, String((double)frequency / 100000000, 6U) + "MHz");
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 20, "Mode: Standalone Beaon");
  display.drawString(0, 30, "OpMode: FST4W");
  display.drawString(0, 50, "TxEnabled: " + String(txEnabled ? "true" : "false"));
  display.display();
#endif

#ifdef ENABLE_LCD_SUPPORT
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(String((double)frequency / 100000000, 6U) + "MHz");
  lcd.setCursor(0, 1);
  sprintf(sub_mode, "%d", FST4W_MODE);
  lcd.print("FST4W (" + String(sub_mode) + ") TX: " + String(txEnabled ? "T" : "F"));
#endif
}

// Loop through the string, transmitting one character at a time.
void jtTransmitMessage()
{
  uint8_t i;

  Serial.printf("TX! FST4W Mode is (%d)\n", FST4W_MODE);
  // Reset the tone to the base frequency and turn on the output
  si5351.set_clock_pwr(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  txEnabled = true;
  updateDisplay();
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PTT_PIN, HIGH);// Note

  for (i = 0; i < symbolCount; i++)
  {
    si5351.set_freq(frequency + (FST4Wsymbols[i] * (toneSpacing[FST4W_MODE] * 100)), SI5351_CLK0);
    delay(symbolLength[FST4W_MODE]);
  }

  // Turn off the output
  si5351.set_clock_pwr(SI5351_CLK0, 0);
  si5351.output_enable(SI5351_CLK0, 0);
  digitalWrite(PTT_PIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
  txEnabled = false;
  updateDisplay();
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
  frequency = FST4W_DEFAULT_FREQ * 100ULL;;
  symbolCount = FST4W_SYMBOL_COUNT;
  operatingMode = MODE_FST4W;
  // Serial.println("Getting ready for FST4W...");
}

#ifdef WITH_NTP_SUPPORT
void sync_time()
{
  // const long utcOffsetInSeconds = (5.5 * 3600);
  const long utcOffsetInSeconds = 0; // UTC. Note. Attention!
  WiFiUDP ntpUDP;
  NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

  WiFi.hostname("beacon");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println(WiFi.localIP());
  timeClient.begin();
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  rtc.adjust(now);
  Serial.println("Time set on RTC!");
  led_flash();
}
#endif

void sync_time_with_gps()
{
  bool updated = 0;
  Serial.println("Finding GPS data...");
  delay(500);
  digitalWrite(IS_GPS_SYNCED_PIN, LOW);

  myPort.begin(9600);
  if (!myPort) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid SoftwareSerial pin configuration, check config");
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  }
  // Do the GPS thing until success.
  do {
    while (myPort.available() > 0)
      gps.encode(myPort.read());
    if (gps.time.isUpdated() && gps.date.isUpdated()) {
      byte Year = gps.date.year();
      byte Month = gps.date.month();
      byte Day = gps.date.day();
      byte Hour = gps.time.hour();
      byte Minute = gps.time.minute();
      byte Second = gps.time.second();
      rtc.adjust(DateTime(Year, Month, Day, Hour, Minute, Second));
      // Serial.println(("[+] Time updated from GPS, w00t!"));
      updated = 1;
      led_flash();
      digitalWrite(IS_GPS_SYNCED_PIN, HIGH);
    } else {
      Serial.println(("[!] No GPS fix yet. Can't set RTC yet. Please wait..."));
      delay(500);
    }
  } while (!updated);
}

void sync_time_with_gps_with_timeout()
{
  int tries = 0;
  digitalWrite(IS_GPS_SYNCED_PIN, LOW);
  bool updated = 0;

  myPort.begin(9600);
  do {
    while (myPort.available() > 0)
      gps.encode(myPort.read());
    if (gps.time.isUpdated() && gps.date.isUpdated() && (gps.location.isValid() && gps.location.age() < 2000)) {
      byte Year = gps.date.year();
      byte Month = gps.date.month();
      byte Day = gps.date.day();
      byte Hour = gps.time.hour();
      byte Minute = gps.time.minute();
      byte Second = gps.time.second();
      rtc.adjust(DateTime(Year, Month, Day, Hour, Minute, Second));
      digitalWrite(IS_GPS_SYNCED_PIN, HIGH);
      led_flash();
      updated = 1;
    } else {
      tries = tries + 1;
#ifdef ENABLE_LCD_SUPPORT
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.printf("GPS wait (%d)", tries);
#endif
      delay(500);
    }
    if (tries > 120) // keep trying gps sync for a ~minute only
      break;
  } while (!updated);
}

void setup()
{
  int ret = 0;
  char date[10] = "hh:mm:ss";

  mode = FST4W;
  freq = FST4W_DEFAULT_FREQ;

  // Safety first!
  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);
  pinMode(IS_GPS_SYNCED_PIN, OUTPUT);
  digitalWrite(IS_GPS_SYNCED_PIN, LOW);

  // Setup serial and IO pins
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH); // OFF
  delay(2000);
  Serial.println("\n...");

  // Initialize the Si5351
  ret = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, si5351CalibrationFactor);
  if (ret != true) {
    Serial.flush();
    vfo_ok = 0;
  }
  si5351.set_clock_pwr(SI5351_CLK0, 0); // safety first
  if (ret != 1) {
    Serial.print("Si5351 init status (should be 1 always) = ");
    Serial.println(ret);
  }

  // Initialize the rtc
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC!");
    Serial.flush();
    abort();
  }
  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power, continue!?"));
    Serial.flush();
    rtc_lost_power = 1;
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // hack!
    // abort();  // NOTE
  }
  rtc.disable32K();
  rtc.now().toString(date);

  // Print status
  /* Serial.print("Current UTC time is = ");
    Serial.println(date);
    Serial.print("Temperature is: ");
    Serial.print(rtc.getTemperature());
    Serial.println(" C"); */

  // Set CLK0 output
  si5351.set_freq(freq * 100, SI5351_CLK0);
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
  if (rtc_lost_power) {
    Serial.println("Check and set RTC time!");
    led_flash();
    delay(50);
  }

  // Initialising the UI
#ifdef ENABLE_OLED_SUPPORT
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
#endif

#ifdef ENABLE_LCD_SUPPORT
  lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();
#endif

  updateDisplay();
  digitalWrite(ledPin, HIGH); // OFF

  // do automatic gps sync with timeout at startup
  sync_time_with_gps_with_timeout();
  updateDisplay();
}

// main loop
void loop()
{
  char c;

  if (Serial.available() > 0) {
    c = Serial.read();
    Serial.println(c);
    if (c == 't') {
      jtTransmitMessage();
    }
    if (c == 's') {
#ifdef WITH_NTP_SUPPORT
      sync_time();
#endif
    }
    if (c == 'w') {
#ifdef WITH_NTP_SUPPORT
      char date[10] = "hh:mm:ss";
      rtc.now().toString(date);
      Serial.print("Current UTC time is = ");
      Serial.println(date);
#endif
    }
    if (c == 'g') {
      sync_time_with_gps();
      char date[10] = "hh:mm:ss";
      rtc.now().toString(date);
      Serial.print("Current UTC time is = ");
      Serial.println(date);
    }
    else if (c == '0') {
      Serial.println("Setting FST4W_MODE to 0 (FST4W-120)...");
      FST4W_MODE = 0;
      updateDisplay();
    }
    else if (c == '1') {
      Serial.println("Setting FST4W_MODE to 1 (FST4W-300)...");
      FST4W_MODE = 1;
      updateDisplay();
    }
    else if (c == '2') {
      Serial.println("Setting FST4W_MODE to 2 (FST4W-900)...");
      FST4W_MODE = 2;
      updateDisplay();
    }
    else if (c == '3') {
      Serial.println("Setting FST4W_MODE to 3 (FST4W-1800)...");
      FST4W_MODE = 3;
      updateDisplay();
    }
  }

  dt = rtc.now();
  // FST4W-120
  // if (dt.second() == 0 && (dt.minute() % 6 == 0) && mode == FST4W) {  // production
  if (dt.second() == 0 && (dt.minute() % 2 == 0) && mode == FST4W && FST4W_MODE == 0) {  // testing
    // if (dt.second() == 0 && (dt.minute() % 4 == 0) && mode == FST4W && FST4W_MODE == 0) {  // testing
    jtTransmitMessage();
  }
  // FST4W-300
  if (dt.second() == 0 && (dt.minute() % 5 == 0) && mode == FST4W && FST4W_MODE == 1) {
    jtTransmitMessage();
  }
  // FST4W-900
  if (dt.second() == 0 && (dt.minute() % 15 == 0) && mode == FST4W && FST4W_MODE == 2) {
    jtTransmitMessage();
  }
  // FST4W-1800
  if (dt.second() == 0 && (dt.minute() % 30 == 0) && mode == FST4W && FST4W_MODE == 3) {
    jtTransmitMessage();
  }

  delay(10);
}

// end of loop
