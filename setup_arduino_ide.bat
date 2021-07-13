REM https://downloads.arduino.cc/arduino-cli.exe/arduino-cli_latest_Windows_64bit.zip

arduino-cli.exe lib install "AltSoftSerial"
arduino-cli.exe lib install "SSD1306Ascii"
arduino-cli.exe config init
arduino-cli.exe core update-index
arduino-cli.exe core install esp8266:esp8266
arduino-cli.exe core install rp2040:rp2040

arduino-cli.exe lib install "TinyGPSPlus"
arduino-cli.exe lib install "ESP8266 and ESP32 OLED driver for SSD1306 displays"
arduino-cli.exe lib install "Etherkit JTEncode"
arduino-cli.exe lib install "Etherkit Si5351"
arduino-cli.exe lib install "RTClib"
arduino-cli.exe lib install "Time"
arduino-cli.exe lib install "NTPClient"
arduino-cli.exe lib install "uEEPROMLib"
arduino-cli.exe lib install "Adafruit BusIO"
arduino-cli.exe lib install "LiquidCrystal"

arduino-cli.exe config init
arduino-cli.exe core update-index
arduino-cli.exe core install arduino:avr
arduino-cli.exe lib install "Etherkit JTEncode"
arduino-cli.exe lib install "Etherkit Si5351"
arduino-cli.exe lib install "RTClib"
arduino-cli.exe lib install "LiquidCrystal I2C"

arduino-cli compile --fqbn=esp8266:esp8266:d1_mini_clone Easy-Digital-Beacons-v4

@REM arduino-cli compile --fqbn=esp8266:esp8266:d1_mini_clone FST4W-Antonio
@REM arduino-cli.exe -v upload -p "COM3" --fqbn=esp8266:esp8266:d1_mini_clone FST4W-Antonio
