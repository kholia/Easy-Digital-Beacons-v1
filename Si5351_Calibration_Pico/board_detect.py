#!/usr/bin/env python

# pip3 install pyserial

import serial.tools.list_ports
import serial

a = serial.tools.list_ports.comports()
for w in a:
    # https://devicehunt.com/view/type/usb/vendor/1A86/device/7523
    # https://linux-hardware.org/?id=usb:2e8a-000a
    if "1a86" in w.hwid.lower() or "2e8a" in w.hwid.lower():
        print(w.device)
