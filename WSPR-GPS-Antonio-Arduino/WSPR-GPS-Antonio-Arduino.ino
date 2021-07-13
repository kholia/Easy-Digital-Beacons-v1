// Huge thanks to https://github.com/f4goh/WSPR for the original WSPR work!

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <si5351.h>
#include <TimeLib.h>
#include <JTEncode.h>

// GPS stuff
#include <TinyGPS++.h>
TinyGPSPlus gps;
#include <AltSoftSerial.h>
AltSoftSerial altSerial;  // Uses pin 8 (D8) for RX on Arduino Nano

#include <LiquidCrystal_I2C.h>
// https://randomnerdtutorials.com/esp32-esp8266-i2c-lcd-arduino-ide/
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Global defines
#define PTT_PIN 5 // D5
#define IS_GPS_SYNCED_PIN 6 // D6

// Use https://github.com/etherkit/Si5351Arduino/blob/master/examples/si5351_calibration/si5351_calibration.ino
// to derive calibration value below!
int32_t si5351CalibrationFactor = 16999;  // si5351 calibration factor, CHANGE ME!

// CHANGE ME PLEASE, Use https://kholia.github.io/wspr_encoder.html to customize this value
// The following tx_buffer is for "IK1HGI JN45 23" message
uint8_t tx_buffer[] = { 1, 1, 0, 0, 2, 2, 0, 2, 3, 0, 0, 2, 1, 1, 1, 2, 2, 0, 3, 0, 2, 1, 0, 3, 1, 1, 1, 2, 2, 2, 0, 2, 2, 2, 3, 2, 2, 3, 0, 3, 2, 0, 2, 2, 0, 2, 1, 0, 3, 3, 0, 0, 1, 3, 0, 1, 2, 2, 0, 1, 1, 2, 3, 2, 2, 2, 2, 3, 1, 0, 3, 2, 1, 0, 1, 2, 3, 0, 0, 3, 2, 2, 1, 2, 1, 1, 2, 2, 0, 3, 3, 0, 3, 0, 1, 0, 0, 0, 1, 0, 0, 0, 2, 0, 3, 2, 0, 1, 2, 0, 3, 3, 1, 0, 1, 1, 0, 2, 1, 1, 0, 1, 2, 0, 2, 3, 1, 3, 0, 0, 0, 2, 0, 3, 2, 1, 2, 0, 3, 1, 2, 0, 0, 0, 2, 2, 2, 1, 3, 2, 1, 2, 1, 3, 0, 0, 2, 3, 3, 0, 2, 0 };
uint64_t frequency =    14097050UL; // 20m band, CHANGE ME, if required

// uint64_t frequency = 28124600UL; // 10m band, CHANGE ME, if required
// uint64_t frequency = 24924600UL; // 24m band, CHANGE ME, if required
// uint64_t frequency = 18104600UL; // 18m band, CHANGE ME, if required
// uint64_t frequency = 21094600UL; // 15m band, CHANGE ME, if required
// uint64_t frequency = 10138700UL; // 30m band, CHANGE ME, if required
// uint64_t frequency =  7038600UL; // 40m band, CHANGE ME, if required
// uint64_t frequency =  5364700UL; // 60m band, CHANGE ME, if required
// uint64_t frequency =  5287200UL; // 60m band, CHANGE ME, if required
// uint64_t frequency =  3568600UL; // 80m band, CHANGE ME, if required
// uint64_t frequency =  1836600UL; // 160m band, CHANGE ME, if required
// uint64_t frequency =   472750UL; // 600m band, CHANGE ME, if required
// uint64_t frequency =   135700UL; // 2200m band, CHANGE ME, if required

// WSPR properties
#define TONE_SPACING            146           // ~1.46 Hz
#define WSPR_DELAY              683          // Delay value for WSPR
#define SYMBOL_COUNT            WSPR_SYMBOL_COUNT

boolean txEnabled = false;
// JTEncode jtencode; // is super buggy and causes crashes - avoid!
RTC_DS3231 rtc;
Si5351 si5351;
DateTime dt;
unsigned long freq;
int rtc_lost_power = 0;
int vfo_ok = 1;
const int ledPin = LED_BUILTIN;
uint8_t symbolCount;

void updateDisplay()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(String((double)frequency / 1000000, 6U) + " MHz");
  lcd.setCursor(0, 1);
  lcd.print("WSPR TX: " + String(txEnabled ? "T" : "F"));
}

// Loop through the string, transmitting one character at a time.
void jtTransmitMessage()
{
  uint8_t i;

  // Serial.printf("TX! FST4W Mode is (%d)\n", FST4W_MODE);
  // Reset the tone to the base frequency and turn on the output
  si5351.set_clock_pwr(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  txEnabled = true;
  updateDisplay();
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(PTT_PIN, HIGH);

  for (i = 0; i < symbolCount; i++)
  {
    // Thanks to https://github.com/W3PM/Auto-Calibrated-GPS-RTC-Si5351A-FST4W-and-WSPR-MEPT/blob/main/w3pm_GPS_FST4W_WSPR_V1_1a.ino
    unsigned long timer = millis();
    si5351.set_freq((frequency * 100) + (tx_buffer[i] * TONE_SPACING), SI5351_CLK0);
    while ((millis() - timer) <= WSPR_DELAY) {
      __asm__("nop\n\t");
    };
  }

  // Turn off the output
  si5351.set_clock_pwr(SI5351_CLK0, 0);
  si5351.output_enable(SI5351_CLK0, 0);
  digitalWrite(PTT_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  txEnabled = false;
  updateDisplay();
}

String getTime()
{
  char date[10] = "hh:mm:ss";
  rtc.now().toString(date);

  return date;
}

void set_mode()
{
  symbolCount = SYMBOL_COUNT;
  Serial.println("Getting ready for WSPR...");
}

void sync_time_with_gps()
{
  bool updated = 0;
  Serial.println(F("Finding GPS data..."));
  delay(500);
  digitalWrite(IS_GPS_SYNCED_PIN, LOW);

  altSerial.begin(9600);
  // Do the GPS thing until success.
  do {
    while (altSerial.available() > 0)
      gps.encode(altSerial.read());
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
      digitalWrite(IS_GPS_SYNCED_PIN, HIGH);
    } else {
      Serial.println(F("[!] No GPS fix yet. Can't set RTC yet. Please wait..."));
      delay(500);
    }
  } while (!updated);
}

void sync_time_with_gps_with_timeout()
{
  int tries = 0;
  digitalWrite(IS_GPS_SYNCED_PIN, LOW);
  bool updated = 0;

  altSerial.begin(9600);
  do {
    while (altSerial.available() > 0)
      gps.encode(altSerial.read());
    if (gps.time.isUpdated() && gps.date.isUpdated() && (gps.location.isValid() && gps.location.age() < 2000)) {
      byte Year = gps.date.year();
      byte Month = gps.date.month();
      byte Day = gps.date.day();
      byte Hour = gps.time.hour();
      byte Minute = gps.time.minute();
      byte Second = gps.time.second();
      rtc.adjust(DateTime(Year, Month, Day, Hour, Minute, Second));
      digitalWrite(IS_GPS_SYNCED_PIN, HIGH);
      updated = 1;
    } else {
      tries = tries + 1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("GPS wait "));
      lcd.print(tries);
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

  // Safety first!
  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);
  pinMode(IS_GPS_SYNCED_PIN, OUTPUT);
  digitalWrite(IS_GPS_SYNCED_PIN, LOW);

  // Setup serial and IO pins
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
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
    Serial.print(F("Si5351 init status (should be 1 always) = "));
    Serial.println(ret);
  }

  // Initialize the rtc
  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC!"));
  }
  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power, continue!?"));
    rtc_lost_power = 1;
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // hack!
    // abort();  // NOTE
  }

  // Set CLK0 output
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for maximum power
  si5351.set_clock_pwr(SI5351_CLK0, 0); // Disable the clock initially

  // Note
  set_mode();

  // Sanity checks
  if (!vfo_ok) {
    Serial.println(F("Check VFO connections!"));
  }
  if (rtc_lost_power) {
    Serial.println(F("Check and set RTC time!"));
  }

  // Initialising the UI
  lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();

  updateDisplay();
  digitalWrite(ledPin, LOW); // Turn off the inbuilt LED -> All good so far

  // do automatic gps sync with timeout at startup
  Serial.println(F("Waiting for GPS, check LCD for status..."));
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
    if (c == 'w') {
      char date[10] = "hh:mm:ss";
      rtc.now().toString(date);
      Serial.print(F("Current UTC time is = "));
      Serial.println(date);
    }
    if (c == 'g') {
      sync_time_with_gps();
      char date[10] = "hh:mm:ss";
      rtc.now().toString(date);
      Serial.print(F("Current UTC time is = "));
      Serial.println(date);
    }
  }
  dt = rtc.now();
  // WSPR TX LOOP
  if (dt.second() == 0 && (dt.minute() % 2 == 0)) {
    jtTransmitMessage();
  }

  delay(10);
}
