Note: A DS3231 RTC module is required by this code!

SDA - A4. SCL - A5.

Connect TX pin of the NEO-6M GPS module to pin `D8` of Arduino Nano.

Connect `D5` pin to an external LED (via a resistor) for TX indication.

Connect `D6` pin to an external LED (via a resistor) for `GPS SYNC` indication.

On Arduino startup, it will automatically try to do a GPS time sync for a
~minute.

Please power the `I2C LCD Display (16x2)` using 5v. It works poorly with
3.3v.

Launch the `Serial Monitor` (at a baud rate of 9600) and send the `g` command
to trigger a GPS update manually.
