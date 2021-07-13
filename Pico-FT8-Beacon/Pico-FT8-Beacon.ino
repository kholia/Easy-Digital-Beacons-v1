// Runs on Raspberry Pi Pico

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <si5351.h>
#include <TimeLib.h>

// GPS stuff
#include <TinyGPS++.h>
TinyGPSPlus gps;

RTC_DS3231 rtc;
Si5351 si5351;
DateTime dt;

int symbolCount = 79;
int toneSpacing = 625;
int frequency = 14075500 * 100UL;
#define FT8_TONE_SPACING 625  // ~6.25 Hz
#define FT8_DELAY 159  // Delay value for FT8
#define FT8_DEFAULT_FREQ 14075000UL
#define FT8_SYMBOL_COUNT 79
char message[] = "VU3CER VU3FOE MK68";

uint8_t tones[FT8_SYMBOL_COUNT];
int toneDelay = 159;

int rtc_lost_power = 0;
int vfo_ok = 1;

// Use https://github.com/etherkit/Si5351Arduino/blob/master/examples/si5351_calibration/si5351_calibration.ino
// to derive calibration value below!
int32_t si5351CalibrationFactor = 16999;  // si5351 calibration factor

int encoder(char *message, uint8_t *tones, int is_ft4);

void tx(uint8_t *tones)
{
  uint8_t i;

  Serial.println("TX!");
  digitalWrite(LED_BUILTIN, HIGH);
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
}

String getTime()
{
  char date[10] = "hh:mm:ss";
  rtc.now().toString(date);

  return date;
}

void sync_time_with_gps_with_timeout()
{
  // digitalWrite(IS_GPS_SYNCED_PIN, LOW);
  bool newData = false;

  Serial.println("GPS Sync Wait...");
  digitalWrite(LED_BUILTIN, LOW);
  Serial2.begin(9600); // https://github.com/earlephilhower/arduino-pico/blob/master/variants/rpipico/pins_arduino.h#L11-L15

  for (unsigned long start = millis(); millis() - start < 32000;)
  {
    while (Serial2.available())
    {
      char c = Serial2.read();
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
  Serial.begin(115200);
  delay(5000);

  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();

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

  Serial.print(F("Si5351 init status (should be 1 always) = "));
  Serial.println(ret);

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

  // Set CLK0 output
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for maximum power
  si5351.set_clock_pwr(SI5351_CLK0, 0); // Disable the clock initially

  encoder(message, tones, 0);
  sync_time_with_gps_with_timeout();
}

void loop() {
  dt = rtc.now();
  if (dt.second() % 15 == 0) {
    tx(tones);
  }
}
