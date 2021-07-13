// Huge thanks to https://github.com/f4goh/ for the original FST4W work!
//
// Huge thanks to IK1HGI (Antonio) for introducing me to FST4W and the top band!
//
// Reference: https://physics.princeton.edu/pulsar/k1jt/FST4_Quick_Start.pdf

#include <Arduino.h>
#include <Wire.h>
#include <si5351.h>
#include <TimeLib.h>
#include <JTEncode.h>

// GPS stuff
#include "SoftwareSerial.h"
#include <TinyGPS++.h>
#define MYPORT_RX 13 // D7
TinyGPSPlus gps;
#define SW_SERIAL_UNUSED_PIN -1
SoftwareSerial myPort(MYPORT_RX, SW_SERIAL_UNUSED_PIN); // RX, TX -> GPS stuff FST4W timing

#define ENABLE_LCD_SUPPORT 1

#ifdef ENABLE_LCD_SUPPORT
#include "LiquidCrystal_I2C.h"
// https://randomnerdtutorials.com/esp32-esp8266-i2c-lcd-arduino-ide/
LiquidCrystal_I2C lcd(0x27, 16, 2);
#endif

// Global defines
#define PTT_PIN 14 // D5
#define IS_GPS_SYNCED_PIN 12 // D6
#define RELAY_PIN 2 // D4

// Use https://github.com/etherkit/Si5351Arduino/blob/master/examples/si5351_calibration/si5351_calibration.ino
// to derive calibration value below!
int32_t si5351CalibrationFactor = 16999;  // si5351 calibration factor

#define TONE_SPACING            146          // ~1.46 Hz
#define WSPR_DELAY              683          // Delay value for WSPR
#define SYMBOL_COUNT            WSPR_SYMBOL_COUNT

// Change this
char call[7] = "IK1HGI";                        // Change this
char loc[5] = "JN45";                           // Change this
uint8_t dbm = 23;                               // Change this
#define WSPR_DEFAULT_FREQ 7038600

boolean txEnabled = false;
uint8_t tx_buffer[SYMBOL_COUNT];

Si5351 si5351;
JTEncode jtencode;
unsigned long freq;
int vfo_ok = 1;
const int ledPin = LED_BUILTIN;
uint64_t frequency = WSPR_DEFAULT_FREQ * 100ULL;
uint8_t symbolCount;
int secPrec = 255;

void updateDisplay()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(String((double)frequency / 100000000, 6U) + "MHz");
  lcd.setCursor(0, 1);
  if (txEnabled)
    lcd.print("WSPR TX: T");
  else
    lcd.print("WSPR RX: F");
}

// Loop through the string, transmitting one character at a time.
void jtTransmitMessage()
{
  uint8_t i;

  Serial.printf("TX!\n");
  // Reset the tone to the base frequency and turn on the output
  si5351.set_clock_pwr(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  txEnabled = true;
  updateDisplay();
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PTT_PIN, HIGH); // Note
  digitalWrite(RELAY_PIN, HIGH); // Relay module is 'Active LOW', turn off the relay during TX
  delay(50); // Safe enough!?

  for (i = 0; i < SYMBOL_COUNT; i++)
  {
    si5351.set_freq(frequency + (tx_buffer[i] * TONE_SPACING), SI5351_CLK0);
    delay(WSPR_DELAY);
  }

  // Turn off the output
  si5351.set_clock_pwr(SI5351_CLK0, 0);
  si5351.output_enable(SI5351_CLK0, 0);
  digitalWrite(PTT_PIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(RELAY_PIN, LOW); // RX mode
  delay(50);
  txEnabled = false;
  updateDisplay();
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
  jtencode.wspr_encode(call, loc, dbm, tx_buffer);
  // Serial.println("Getting ready for FST4W...");
}

void sync_time_with_gps_with_timeout()
{
  digitalWrite(IS_GPS_SYNCED_PIN, LOW);
  bool newData = false;

  Serial.println("GPS Sync Wait...");
#ifdef ENABLE_LCD_SUPPORT
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.printf("GPS Sync Wait...");
#endif
  digitalWrite(LED_BUILTIN, LOW);
  myPort.begin(9600); // https://github.com/earlephilhower/arduino-pico/blob/master/variants/rpipico/pins_arduino.h#L11-L15

  for (unsigned long start = millis(); millis() - start < 32000;)
  {
    while (myPort.available())
    {
      char c = myPort.read();
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
      setTime(Hour, Minute, Second, Day, Month, Year);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      Serial.println("GPS Sync Done!");
      digitalWrite(IS_GPS_SYNCED_PIN, HIGH);
#ifdef ENABLE_LCD_SUPPORT
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.printf("GPS Sync Done!");
#endif
      delay(2000);
      return;
    }
  }
  Serial.println("GPS sync failed!");
#ifdef ENABLE_LCD_SUPPORT
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.printf("GPS Sync Failed!");
#endif
}

time_t sync_time_callback()
{
  sync_time_with_gps_with_timeout();
  return 0;
}

void setup()
{
  int ret = 0;
  char date[10] = "hh:mm:ss";

  // Safety first!
  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Note: Turn on relay during RX
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

  // Set CLK0 output
  si5351.set_freq(frequency, SI5351_CLK0);
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

  // Initialising the UI
#ifdef ENABLE_LCD_SUPPORT
  lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();
#endif

  updateDisplay();
  digitalWrite(ledPin, HIGH); // OFF

  // do automatic gps sync with timeout at startup
  // sync_time_with_gps_with_timeout();
  updateDisplay();
  // Set time sync provider
  setSyncInterval(600); // Note: GPS sync will be done every 10 minutes (600 seconds!)
  setSyncProvider(sync_time_callback);
}

// main loop
void loop()
{
  char c;
  int secNow;

  if (Serial.available() > 0) {
    c = Serial.read();
    Serial.println(c);
    if (c == 't') {
      jtTransmitMessage();
    }
  }

  // https://github.com/PaulStoffregen/Time
  if (second() == 0 && (minute() % 6 == 0)) {  // production
    jtTransmitMessage();
  }

  secNow = second();
  if (secPrec != secNow) {
    char heure[16];
    sprintf(heure, "RX: %02d:%02d:%02d", hour(), minute(), second());
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(String((double)frequency / 100000000, 6U) + "MHz");
    lcd.setCursor(0, 1);
    lcd.print(heure);
    secPrec = secNow;
  }
  delay(10);
}
