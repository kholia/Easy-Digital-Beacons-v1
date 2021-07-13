/*
  WSPR with encode

  Anthony LE CREN F4GOH@orange.fr
  Created 16/7/2021
  the program send wspr sequence every 2 minutes
  In serial Monitor
  key 'h' to set up RTC
  key 'w' to send wspr sequence 162 symbols

  GPS support added by Dhiru (VU3CER) - April 2022

  Upstream code: https://github.com/f4goh/WSPR/blob/master/src/WSPR_TinyGps_encode/WSPR_TinyGps_encode.ino
*/

// GPS stuff
#include <TinyGPS++.h>
TinyGPSPlus gps;
// #include <AltSoftSerial.h>
// AltSoftSerial altSerial;  // Uses pin 8 (D8) for RX on Arduino Nano -> a LED is on pin 8, so we can't use AltSoftSerial
#include <SoftwareSerial.h>
SoftwareSerial altSerial(4, 5); // RX, TX (GPS) -> D4, D5
// RTC stuff
#include <RTClib.h>
#include <si5351.h>
#include <TimeLib.h>
RTC_DS3231 rtc;
DateTime dt;

// CHANGE ME PLEASE, Use https://kholia.github.io/wspr_encoder.html to customize this value
// The following tx_buffer is for "IK1HGI JN45 23" message
uint8_t wsprSymb[] = { 1, 1, 0, 0, 2, 2, 0, 2, 3, 0, 0, 2, 1, 1, 1, 2, 2, 0, 3, 0, 2, 1, 0, 3, 1, 1, 1, 2, 2, 2, 0, 2, 2, 2, 3, 2, 2, 3, 0, 3, 2, 0, 2, 2, 0, 2, 1, 0, 3, 3, 0, 0, 1, 3, 0, 1, 2, 2, 0, 1, 1, 2, 3, 2, 2, 2, 2, 3, 1, 0, 3, 2, 1, 0, 1, 2, 3, 0, 0, 3, 2, 2, 1, 2, 1, 1, 2, 2, 0, 3, 3, 0, 3, 0, 1, 0, 0, 0, 1, 0, 0, 0, 2, 0, 3, 2, 0, 1, 2, 0, 3, 3, 1, 0, 1, 1, 0, 2, 1, 1, 0, 1, 2, 0, 2, 3, 1, 3, 0, 0, 0, 2, 0, 3, 2, 1, 2, 0, 3, 1, 2, 0, 0, 0, 2, 2, 2, 1, 3, 2, 1, 2, 1, 3, 0, 0, 2, 3, 3, 0, 2, 0 };
#define IS_GPS_SYNCED_PIN 3 // D3

#include <SPI.h>
#include <EEPROM.h>
#include <Wire.h>
#include <JTEncode.h>

#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C
#define RST_PIN -1
SSD1306AsciiWire oled;  //afficheur oled;

// JTEncode jtencode; // is super buggy and causes crashes - avoid!

#define LED 8
#define W_CLK 13
#define FQ_UD 10
#define RESET 9
#define GAIN 6  // pwm output pinout to adjust gain (mos polarization)
uint64_t FREQUENCY =    14097050UL; // 20m band, CHANGE ME, if required
// uint64_t FREQUENCY = 28124600UL; // 10m band, CHANGE ME, if required
// uint64_t FREQUENCY = 24924600UL; // 24m band, CHANGE ME, if required
// uint64_t FREQUENCY = 18104600UL; // 18m band, CHANGE ME, if required
// uint64_t FREQUENCY = 21094600UL; // 15m band, CHANGE ME, if required
// uint64_t FREQUENCY = 10138700UL; // 30m band, CHANGE ME, if required
// uint64_t FREQUENCY =  7038600UL; // 40m band, CHANGE ME, if required
// uint64_t FREQUENCY =  7040100UL; // 40m band, CHANGE ME, if required
// uint64_t FREQUENCY =  5364700UL; // 60m band, CHANGE ME, if required
// uint64_t FREQUENCY =  5287200UL; // 60m band, CHANGE ME, if required
// uint64_t FREQUENCY =  3568600UL; // 80m band, CHANGE ME, if required
// uint64_t FREQUENCY =  1836600UL; // 160m band, CHANGE ME, if required
// uint64_t FREQUENCY =   472750UL; // 600m band, CHANGE ME, if required
// uint64_t FREQUENCY =   135700UL; // 2200m band, CHANGE ME, if required
#define CALL "IK1HGI"
#define LOCATOR "JN45"
#define DBM 20
#define WSPR_TONE_SPACING       1.4548
#define WSPR_DELAY              683
#define PWM_GAIN 170      // continous voltage ctrl for mosfet R3=1K and R4=4.7K

long factor = -1500;	  // adjust frequency to wspr band
int secPrec = 0;
int rtc_lost_power = 0;

int gain = PWM_GAIN;
tmElements_t tm;

void sync_time_with_gps_with_timeout()
{
  digitalWrite(IS_GPS_SYNCED_PIN, LOW);
  bool newData = false;

  oled.clear();
  oled.setCursor(0, 0);
  oled.print(F("GPS Sync Wait..."));
  delay(500);

  altSerial.begin(9600);

  for (unsigned long start = millis(); millis() - start < 32000;)
  {
    while (altSerial.available())
    {
      char c = altSerial.read();
#ifdef debugGPS
      Serial.write(c);
#endif
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
    if (newData && gps.time.isUpdated() && gps.date.isUpdated() && (gps.location.isValid() && gps.location.age() < 2000)) {
      byte Year = gps.date.year();
      byte Month = gps.date.month();
      byte Day = gps.date.day();
      byte Hour = gps.time.hour();
      byte Minute = gps.time.minute();
      byte Second = gps.time.second();
      rtc.adjust(DateTime(Year, Month, Day, Hour, Minute, Second));
      digitalWrite(IS_GPS_SYNCED_PIN, HIGH);
      oled.clear();
      oled.setCursor(0, 0);
      Serial.println(F("GPS Sync Done!"));
      delay(2000);
      return;
    }
  }
  Serial.println(F("GPS Sync Failed!"));
  oled.clear();
}

void setup()
{
  // Safety first!
  pinMode(IS_GPS_SYNCED_PIN, OUTPUT);
  digitalWrite(IS_GPS_SYNCED_PIN, LOW);

  Wire.begin();
  Wire.setClock(400000L);
  Serial.begin(9600);
  Serial.print("Hello! :-)");
  pinMode(LED, OUTPUT);
  pinMode(GAIN, OUTPUT);
  analogWrite(GAIN, 0);
#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0
  oled.setFont(TimesNewRoman16_bold);
  oled.clear();
  oled.setCursor(5, 0);
  oled.println(F("WSPR"));
  oled.setCursor(5, 5); // x, y
  oled.print(F("IK1HGI 2022"));
  oled.setFont(fixednums15x31); // Change this to change the clock font size
  delay(2000);
  oled.clear();

  // DDS
  initDds();
  setfreq(0, 0);
  setfreq(0, 0);

  // RTC
  // Initialize the rtc
  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC!"));
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

  // jtencode.wspr_encode(CALL, LOCATOR, DBM, wsprSymb); // encode WSPR, super buggy
  int n, lf;
  lf = 0;
  for (n = 0; n < WSPR_SYMBOL_COUNT; n++) {   // print symbols on serial monitor
    if (lf % 16 == 0) {
      Serial.println();
      Serial.print(n);
      Serial.print(": ");
    }
    lf++;
    Serial.print(wsprSymb[n]);
    Serial.print(',');
  }
  Serial.println();
  sync_time_with_gps_with_timeout();
  Serial.println("Ready to roll...");
}

void loop() {
  char c;
  if (Serial.available() > 0) {
    c = Serial.read();
    Serial.println(c);
    if (c == 't') {
      sendWspr(FREQUENCY);
    }
    if (c == 'v') {
      Serial.println("v");
    }
    if (c == 'w') {
      char date[10] = "hh:mm:ss";
      rtc.now().toString(date);
      Serial.print(F("Current UTC time is = "));
      Serial.println(date);
    }
    if (c == 'g') {
      sync_time_with_gps_with_timeout();
      char date[10] = "hh:mm:ss";
      rtc.now().toString(date);
      Serial.print(F("Current UTC time is = "));
      Serial.println(date);
    }
  }
  dt = rtc.now();

  if (dt.second() == 0 && (dt.minute() % 6 == 0)) {
    sendWspr(FREQUENCY);
  }

  if (secPrec != dt.second()) {
    /* Serial.print(dt.hour());
      Serial.print(":");
      Serial.print(dt.minute());
      Serial.print(":");
      Serial.println(dt.second()); */
    char heure[10];
    sprintf(heure, "%02d:%02d:%02d", dt.hour(), dt.minute(), dt.second());
    oled.setCursor(0, 0);
    oled.print(heure);
    secPrec = tm.Second;
  }
  delay(10);
}

/********************************************************
  AD9850 routines
 ********************************************************/

void initDds()
{
  SPI.begin();
  pinMode(W_CLK, OUTPUT);
  pinMode(FQ_UD, OUTPUT);
  pinMode(RESET, OUTPUT);

  SPI.setBitOrder(LSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  pulse(RESET);
  pulse(W_CLK);
  pulse(FQ_UD);
}

void pulse(int pin) {
  digitalWrite(pin, HIGH);
  // delay(1);
  digitalWrite(pin, LOW);
}

void setfreq(double f, uint16_t p) {
  uint32_t deltaphase;

  deltaphase = f * 4294967296.0 / (125000000 + factor);
  for (int i = 0; i < 4; i++, deltaphase >>= 8) {
    SPI.transfer(deltaphase & 0xFF);
  }
  SPI.transfer((p << 3) & 0xFF) ;
  pulse(FQ_UD);
}

void sendWspr(long freqWspr)
{
  Serial.println("TX!");
  digitalWrite(LED, HIGH);
  oled.clear();
  oled.setCursor(0, 0);
  oled.print("1");
  analogWrite(GAIN, PWM_GAIN);
  int a = 0;
  for (int element = 0; element < 162; element++) {
    a = int(wsprSymb[element]); //   get the numerical ASCII Code
    unsigned long timer = millis();
    setfreq((double) freqWspr + (double) a * WSPR_TONE_SPACING, 0);
    while ((millis() - timer) <= WSPR_DELAY) {
      __asm__("nop\n\t");
    }
    // delay(WSPR_DELAY);
  }
  setfreq(0, 0);
  digitalWrite(LED, LOW);
  analogWrite(GAIN, 0);
  Serial.println("EOT");
}

