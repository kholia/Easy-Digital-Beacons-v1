// Runs on Raspberry Pi Pico

#include <Arduino.h>
#include <Wire.h>
#include <si5351.h>
#include <TimeLib.h>

// GPS stuff
#include <TinyGPS++.h>
TinyGPSPlus gps;

Si5351 si5351;

int symbolCount = 79;
int toneSpacing = 625;
int frequency = 21073500 * 100UL; // CHANGE THIS PLEASE
#define FT8_TONE_SPACING 625  // ~6.25 Hz
#define FT8_DELAY 159  // Delay value for FT8
#define FT8_SYMBOL_COUNT 79
char message[32] = "VU3CER VU3FOE MK68"; // CHANGE THIS PLEASE! ATTENTION: Only proper FT8 messages are allowed!
uint8_t tones[79];

int toneDelay = 159;

// Use https://github.com/etherkit/Si5351Arduino/blob/master/examples/si5351_calibration/si5351_calibration.ino
// to derive calibration value below!
int32_t si5351CalibrationFactor = 16999;  // CHANGE THIS PLEASE, si5351 calibration factor

#define EXTERNAL_LED 6 // GP6 -> Pin 9 on the Pico board

int encoder(char *message, uint8_t *tones, int is_ft4);

void tx(uint8_t *tones)
{
  uint8_t i;

  Serial.println("TX!");
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(EXTERNAL_LED, HIGH);
  si5351.set_clock_pwr(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK0, 1);

  for (i = 0; i < symbolCount; i++)
  {
    si5351.set_freq(frequency + (tones[i] * toneSpacing), SI5351_CLK0);
    delay(toneDelay);
  }

  // Turn off the output
  si5351.set_clock_pwr(SI5351_CLK0, 0);
  si5351.output_enable(SI5351_CLK0, 0);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(EXTERNAL_LED, LOW);
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

void sync_time_with_gps_with_timeout()
{
  // digitalWrite(IS_GPS_SYNCED_PIN, LOW);
  bool newData = false;

  Serial.println("GPS Sync Wait...");
  digitalWrite(LED_BUILTIN, LOW);
  Serial2.begin(9600); // RX, TX -> 8, 9

  for (unsigned long start = millis(); millis() - start < 64000;)
  {
    while (Serial2.available())
    {
      char c = Serial2.read();
#define debugGPS 1
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
      return;
    }
  }
  Serial.println("GPS Sync Failed!");
}

void setup() {
  int ret = 0;

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(EXTERNAL_LED, OUTPUT);
  Serial.begin(115200);
  delay(5000);

  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();

  // Initialize the Si5351
  ret = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, si5351CalibrationFactor);
  if (ret != true) {
    led_flash();
    watchdog_reboot(0, 0, 1000);
  }
  si5351.set_clock_pwr(SI5351_CLK0, 0); // safety first
  if (ret != 1) {
    Serial.print(F("Si5351 init status (should be 1 always) = "));
    Serial.println(ret);
  }

  Serial.print(F("Si5351 init status (should be 1 always) = "));
  Serial.println(ret);

  // Set CLK0 output
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for maximum power
  si5351.set_clock_pwr(SI5351_CLK0, 0); // Disable the clock initially

  encoder(message, tones, 0);

  sync_time_with_gps_with_timeout();
}

void loop() {
  if (second() % 15 == 0) {
    tx(tones);
  }
}
