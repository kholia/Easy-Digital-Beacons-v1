This project is from Antonio (https://www.facebook.com/tony.musumeci.50).

Connect TX pin of the NEO-6M GPS module to pin `D7` of WeMos D1 mini.

Connect `D5` pin to an external LED (via a resistor) for TX indication.

Connect `D6` pin to an external LED (via a resistor) for `GPS SYNC` indication.

On ESP8266 startup, it will automatically try to do a GPS time sync for a
~minute.

Please power the `I2C LCD Display (16x2)` using 5v. It works poorly with
3.3v.

Launch the `Serial Monitor` (at a baud rate of 9600) and send the `g` command
to trigger a GPS update manually.

Note: RTC is NOT needed by this code.
