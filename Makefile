# https://github.com/arduino/arduino-cli/releases

port := $(shell python3 board_detect.py)

default:
	arduino-cli compile --fqbn=arduino:avr:nano:cpu=atmega328old FST4W-GPS-v3-Arduino
	arduino-cli compile --fqbn=esp8266:esp8266:d1_mini_clone Easy-Digital-Beacons-v4

upload:
	@# echo $(port)
	arduino-cli compile --fqbn=rp2040:rp2040:rpipico Pico-FT8-Beacon-OnlyGPS
	arduino-cli compile --fqbn=rp2040:rp2040:rpipico Pico-FT8-Beacon
	arduino-cli compile --fqbn=esp8266:esp8266:d1_mini_clone Pixie-WSPR-ESP8266
	arduino-cli compile --fqbn=arduino:avr:nano:cpu=atmega328old F4GOH-PCB-FST4W-LCD
	arduino-cli compile --fqbn=arduino:avr:nano:cpu=atmega328old F4GOH-PCB-FST4W-OLED
	# arduino-cli -v upload -p "${port}" --fqbn=arduino:avr:nano:cpu=atmega328old F4GOH-PCB-FST4W-OLED
	arduino-cli compile --fqbn=arduino:avr:nano:cpu=atmega328old F4GOH-PCB-WSPR
	# arduino-cli -v upload -p "${port}" --fqbn=arduino:avr:nano:cpu=atmega328old F4GOH-PCB-WSPR
	# arduino-cli -v upload -p "${port}" --fqbn=rp2040:rp2040:rpipico Pico-FT8-Beacon
	# arduino-cli -v upload -p "${port}" --fqbn=arduino:avr:nano:cpu=atmega328old F4GOH-PCB-WSPR
	# arduino-cli compile --fqbn=arduino:avr:nano:cpu=atmega328old WSPR-GPS-Antonio
	# arduino-cli -v upload -p "${port}" --fqbn=arduino:avr:nano:cpu=atmega328old WSPR-GPS-Antonio
	# arduino-cli compile --fqbn=arduino:avr:nano:cpu=atmega328old FST4W-GPS-v3-Arduino
	# arduino-cli -v upload -p "${port}" --fqbn=arduino:avr:nano:cpu=atmega328old FST4W-GPS-v3-Arduino
	# arduino-cli compile --fqbn=esp8266:esp8266:d1_mini_clone WSPR-Antonio
	arduino-cli compile --fqbn=esp8266:esp8266:d1_mini_clone FST4W-GPS-v2-ESP8266
	# arduino-cli -v upload -p "${port}" --fqbn=esp8266:esp8266:d1_mini_clone FST4W-GPS-v2
	# arduino-cli compile --fqbn=esp8266:esp8266:d1_mini_clone Easy-Digital-Beacons-v4
	# arduino-cli -v upload -p "${port}" --fqbn=esp8266:esp8266:d1_mini_clone Easy-Digital-Beacons-v4

install_platform:
	arduino-cli config init --overwrite
	arduino-cli core update-index
	arduino-cli core install rp2040:rp2040
	arduino-cli core install esp8266:esp8266
	arduino-cli core install arduino:avr

deps:
	arduino-cli lib install "Adafruit SSD1306"
	arduino-cli lib install "Adafruit GFX Library"
	arduino-cli lib install "SSD1306Ascii"
	arduino-cli lib install "DS3232RTC"
	arduino-cli lib install "AltSoftSerial"
	arduino-cli lib install "RotaryEncoder"
	arduino-cli lib install "TinyGPSPlus"
	arduino-cli lib install "EspSoftwareSerial"
	arduino-cli lib install "ESP8266 and ESP32 OLED driver for SSD1306 displays"
	arduino-cli lib install "Etherkit JTEncode"
	arduino-cli lib install "Etherkit Si5351"
	arduino-cli lib install "RTClib"
	arduino-cli lib install "Time"
	arduino-cli lib install "NTPClient"
	arduino-cli lib install "uEEPROMLib"
	arduino-cli lib install "Adafruit BusIO"
	arduino-cli lib install "LiquidCrystal"
	# arduino-cli lib install "ESPAsyncWebServer"
	# arduino-cli lib install "ESPAsyncTCP"
	arduino-cli lib install "Etherkit JTEncode"
	arduino-cli lib install "Etherkit Si5351"
	arduino-cli lib install "RTClib"
	arduino-cli lib install "LiquidCrystal I2C"
	arduino-cli lib install "ArduinoJson"

install_arduino_cli:
	curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh

go:
	GOOS=windows GOARCH=amd64 go build -o message2array.exe message2array.go
