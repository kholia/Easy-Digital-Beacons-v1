#include <Arduino.h>
#include <Wire.h>
#include <si5351.h>
#include <TimeLib.h>
#include <JTEncode.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

#include <ESP8266WiFi.h>

// WiFi credentials
#include "credentials.h"

// Global defines
#define PTT_PIN 14 // D5

// Use https://github.com/etherkit/Si5351Arduino/blob/master/examples/si5351_calibration/si5351_calibration.ino
// to derive calibration value below!
int32_t si5351CalibrationFactor = 141900;  // si5351 calibration factor

#define TONE_SPACING            146          // ~1.46 Hz
#define WSPR_DELAY              683          // Delay value for WSPR
#define SYMBOL_COUNT            WSPR_SYMBOL_COUNT

// Change this
char call[7] = "VU3CER";                        // Change this
char loc[5] = "MK68";                           // Change this
uint8_t dbm = 10;                               // Change this
// #define WSPR_DEFAULT_FREQ 14097150UL
#define WSPR_DEFAULT_FREQ 21096150UL

boolean txEnabled = false;
uint8_t tx_buffer[SYMBOL_COUNT];

Si5351 si5351;
JTEncode jtencode;
unsigned long freq;
int vfo_ok = 1;
const int ledPin = LED_BUILTIN;
uint64_t frequency = WSPR_DEFAULT_FREQ * 100ULL;
uint8_t symbolCount;
const long utcOffsetInSeconds = (5.5 * 3600);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

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
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PTT_PIN, HIGH); // Note

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
  txEnabled = false;
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
  Serial.println("Getting ready for WSPR...");
}

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

void setup()
{
  int ret = 0;
  char date[10] = "hh:mm:ss";

  // Safety first!
  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);

  // Setup serial and IO pins
  Serial.begin(115200);
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

  digitalWrite(ledPin, HIGH); // OFF

  WiFi.hostname("beacon");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println(WiFi.localIP());
  timeClient.begin();

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
  if (second() == 0 && (minute() % 2 == 0)) {  // production
    jtTransmitMessage();
  }

  delay(10);
}
