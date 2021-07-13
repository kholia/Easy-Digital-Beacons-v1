This program adds GPS functionality to the following code:

https://github.com/f4goh/WSPR/tree/master/src/IK1HGI/wsprEncode

Note: Connect TX pin of the NEO-6M GPS module to pin `D4` of Arduino Nano or to
the `TXGPS` pin on the F4GOH PCB board.

Connect `D3` pin to an external LED (via a resistor) for `GPS SYNC` indication.

On Arduino startup, it will automatically try to do a GPS time sync.

Launch the `Serial Monitor` (at a baud rate of 9600) and send the `g` command
to trigger a GPS update manually.
