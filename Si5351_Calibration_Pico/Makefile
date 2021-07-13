# https://github.com/arduino/arduino-cli/releases

port := $(shell python3 board_detect.py)

default:
	@# echo $(port)
	arduino-cli compile --fqbn=rp2040:rp2040:rpipico si5351_calibration
	arduino-cli -v upload -p "${port}" --fqbn=rp2040:rp2040:rpipico si5351_calibration

install_platform:
	arduino-cli config init --overwrite
	arduino-cli core update-index
	arduino-cli core install rp2040:rp2040

deps:
	arduino-cli lib install "Etherkit Si5351"

install_arduino_cli:
	curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
