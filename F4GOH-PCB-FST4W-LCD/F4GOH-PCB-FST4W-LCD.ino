// Huge thanks to https://github.com/f4goh/ for the original FST4W work!
//
// Huge thanks to IK1HGI (Antonio) for introducing me to FST4W and the top band!
//
// Reference: https://physics.princeton.edu/pulsar/k1jt/FST4_Quick_Start.pdf

// GPS stuff
#include <TinyGPS++.h>
TinyGPSPlus gps;
// #include <AltSoftSerial.h>
// AltSoftSerial altSerial;  // Uses pin 8 (D8) for RX on Arduino Nano -> a LED is on pin 8, so we can't use AltSoftSerial
#include <SoftwareSerial.h>
SoftwareSerial altSerial(4, 5); // RX, TX (GPS) -> D4, D5 -> GPS stuff for timing
// RTC stuff
#include <RTClib.h>
#include <si5351.h>
#include <TimeLib.h>
RTC_DS3231 rtc;
DateTime dt;

#define FST4W_SYMBOL_COUNT  160

// Select frequency below
#define FST4W_DEFAULT_FREQ  475700UL // 475700 - 1500 (factor) = 474200 KHz band (630m band)! Tune receiver to 474.200 KHz USB!
// #define FST4W_DEFAULT_FREQ  137450UL // Antonio's frequency (2200m band)
// #define FST4W_DEFAULT_FREQ  14097050UL  // 20 meter band for testing
long factor = -1500; // adjust frequency to wspr band

// fst4sim "IK1HGI JN45 20" 120 1500 0.0 0.1 1.0 10 -15 F
const uint8_t FST4Wsymbols[FST4W_SYMBOL_COUNT] = { 0, 1, 3, 2, 1, 0, 2, 3, 3, 0, 3, 3, 0, 3, 1, 0, 0, 2, 2, 1, 3, 1, 3, 0, 3, 1, 0, 1, 0, 3, 1, 3, 0, 0, 3, 0, 1, 1, 2, 3, 1, 0, 3, 2, 0, 1, 0, 3, 1, 2, 1, 2, 3, 2, 1, 1, 0, 0, 1, 3, 3, 1, 3, 0, 0, 2, 3, 1, 2, 0, 3, 0, 2, 2, 1, 2, 0, 1, 3, 2, 1, 0, 2, 3, 0, 3, 0, 3, 2, 2, 0, 1, 0, 2, 0, 1, 0, 0, 2, 2, 3, 1, 3, 0, 2, 1, 1, 3, 0, 1, 0, 3, 2, 2, 2, 3, 1, 0, 3, 2, 0, 1, 0, 0, 3, 2, 1, 2, 0, 2, 2, 0, 3, 0, 0, 1, 3, 3, 3, 0, 0, 0, 1, 3, 2, 1, 1, 0, 0, 2, 0, 2, 0, 1, 3, 2, 1, 0, 2, 3 };

// Note - Change me for using a different FST4W mode!
uint8_t FST4W_MODE = 0; // Use 'Serial Port Commands' to change mode
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

enum modes
{
  FST4W,
};
enum modes mode = FST4W; // Note

#define IS_GPS_SYNCED_PIN 3 // D3

#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>

// LCD display
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define LED 8
#define W_CLK 13
#define FQ_UD 10
#define RESET 9
#define GAIN 6  // pwm output pinout to adjust gain (mos polarization)
uint64_t FREQUENCY = FST4W_DEFAULT_FREQ;


#define PWM_GAIN 170      // continous voltage ctrl for mosfet R3=1K and R4=4.7K

int secPrec = 0;
int rtc_lost_power = 0;
boolean txEnabled = false;

int gain = PWM_GAIN;
tmElements_t tm;

void updateDisplay()
{
  char sub_mode[2];
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(String((double)FREQUENCY / 100000000, 6U) + "MHz");
  lcd.setCursor(0, 1);
  sprintf(sub_mode, "%d", FST4W_MODE);
  lcd.print("FST4W (" + String(sub_mode) + ") TX: " + String(txEnabled ? "T" : "F"));
}

void sync_time_with_gps_with_timeout()
{
  digitalWrite(IS_GPS_SYNCED_PIN, LOW);
  bool newData = false;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("GPS wait ..."));
  delay(1000);

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
      Serial.println(F("GPS Sync Done!"));
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("GPS Sync Done!"));
      delay(2000);
      return;
    }
  }
  Serial.println(F("GPS Sync Failed!"));
  delay(2000);
  lcd.clear();
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

  int n, lf;
  lf = 0;
  for (n = 0; n < FST4W_SYMBOL_COUNT; n++) {   // print symbols on serial monitor
    if (lf % 16 == 0) {
      Serial.println();
      Serial.print(n);
      Serial.print(": ");
    }
    lf++;
    Serial.print(FST4Wsymbols[n]);
    Serial.print(',');
  }

  lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();

  Serial.println();
  sync_time_with_gps_with_timeout();
  Serial.println("Ready to roll...");
  updateDisplay();
}

void loop() {
  char c;
  if (Serial.available() > 0) {
    c = Serial.read();
    Serial.println(c);
    if (c == 't') {
      tx(FREQUENCY);
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
    if (c == '0') {
      Serial.println(F("Setting FST4W_MODE to 0 (FST4W-120)..."));
      FST4W_MODE = 0;
      updateDisplay();
    }
    if (c == '1') {
      Serial.println(F("Setting FST4W_MODE to 1 (FST4W-300)..."));
      FST4W_MODE = 1;
      updateDisplay();
    }
    if (c == '2') {
      Serial.println(F("Setting FST4W_MODE to 2 (FST4W-900)..."));
      FST4W_MODE = 2;
      updateDisplay();
    }
    if (c == '3') {
      Serial.println(F("Setting FST4W_MODE to 3 (FST4W-1800)..."));
      FST4W_MODE = 3;
      updateDisplay();
    }
  }

  dt = rtc.now();
  // FST4W-120
  if (dt.second() == 0 && (dt.minute() % 6 == 0) && mode == FST4W) {  // production
    // if (dt.second() == 0 && (dt.minute() % 2 == 0) && mode == FST4W && FST4W_MODE == 0) {  // testing
    tx(FREQUENCY);
  }
  // FST4W-300
  if (dt.second() == 0 && (dt.minute() % 5 == 0) && mode == FST4W && FST4W_MODE == 1) {
    tx(FREQUENCY);
  }
  // FST4W-900
  if (dt.second() == 0 && (dt.minute() % 15 == 0) && mode == FST4W && FST4W_MODE == 2) {
    tx(FREQUENCY);
  }
  // FST4W-1800
  if (dt.second() == 0 && (dt.minute() % 30 == 0) && mode == FST4W && FST4W_MODE == 3) {
    tx(FREQUENCY);
  }

  if (secPrec != dt.second()) {
    /* Serial.print(dt.hour());
      Serial.print(":");
      Serial.print(dt.minute());
      Serial.print(":");
      Serial.println(dt.second()); */
    char heure[10];
    sprintf(heure, "%02d:%02d:%02d", dt.hour(), dt.minute(), dt.second());
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(heure);
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

void tx(long freq)
{
  Serial.println("TX!");
  digitalWrite(LED, HIGH);
  txEnabled = true;
  updateDisplay();
  analogWrite(GAIN, PWM_GAIN);
  int a = 0;
  for (int element = 0; element < 162; element++) {
    a = int(FST4Wsymbols[element]); //   get the numerical ASCII Code
    unsigned long timer = millis();
    setfreq((double) freq + (double) a * toneSpacing[FST4W_MODE], 0);
    while ((millis() - timer) <= symbolLength[FST4W_MODE]) {
      __asm__("nop\n\t");
    }
    // delay(WSPR_DELAY);
  }
  setfreq(0, 0);
  digitalWrite(LED, LOW);
  analogWrite(GAIN, 0);
  Serial.println("EOT");
  txEnabled = false;
  updateDisplay();
}

