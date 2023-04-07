```
[dhiru@zippy BeaconPlusAP]$ ./wspr_monitor.py
[2023-04-07 21:06:00] VU3FOE ==> I am OK
[2023-04-07 21:04:00] VU3FOE ==> I am OK
[2023-04-07 21:02:00] VU3FOE ==> I am OK
[2023-04-07 21:00:00] VU3FOE ==> I am OK
[2023-04-07 20:58:00] VU3FOE ==> Please check on me!
[2023-04-07 20:56:00] VU3FOE ==> Please check on me!
[2023-04-07 20:54:00] VU3FOE ==> Please check on me!
[2023-04-07 20:48:00] VU3FOE ==> I am OK
[2023-04-07 20:46:00] VU3FOE ==> I am OK
[2023-04-07 19:54:00] VU3FOE ==> I am OK
[2023-04-07 19:52:00] VU3FOE ==> I am OK
```

Note: Use Arduino IDE 1.8.19 with this project!

https://www.cpcstech.com/dbm-to-watt-conversion-information.htm

https://swharden.com/software/FSKview/wspr/

Note 2: Use https://github.com/kholia/BeaconHelper to push time to the beacon.

Power level, [Pwr] is taken as a value from 0 - 60. Although only certain
values will work with the WSJT / WSPR software (just those ending in 0, 3 or 7)
any of the possible 61 values will be encoded; Illegal values are labelled when
decoded.

The total source data has now been reduced to 50 bits of data: 28 for the
callsign in N, and 15 for the locator and 7 for the power in M.

...

Personal disaster management system :)

Run WSPR beacon in watchdog mode. Send message "Please check  on me!". If the
beacon is left unattended for a long time, it sends this message out
automatically.
