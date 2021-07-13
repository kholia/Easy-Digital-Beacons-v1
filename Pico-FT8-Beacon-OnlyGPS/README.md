#### Connections

Pin 1 (GP0) is SDA and Pin 2 (GP1) is SCL - connect Si5351 module to these
pins. Connect GPS's TX pin to pin 12 (GP9) of Pi Pico.


#### Install the firmware on Pi Pico

- Hold down the BOOTSEL button on your Pico (keep holding it down) and plug it
  into your computer's USB port.

  Now release the BOOTSEL button.

- Open Explorer, and open the RPI-RP2 directory like you would any other hard
  drive.

- Drag and drop the UF2 file (`firmware.uf2`) from this folder into the RPI-RP2
  directory.

- Done ;)


#### Build from source

Use the latest Arduino IDE along with https://github.com/earlephilhower/arduino-pico
to build this source code.


#### Misc. Notes

https://github.com/mikalhart/TinyGPSPlus/issues/30 can affect NEO-6M GPS
modules - take care!


#### Credits

- https://github.com/earlephilhower/arduino-pico

- https://github.com/etherkit/Si5351Arduino/blob/master/src/si5351.cpp

- https://github.com/kgoba/ft8_lib/
