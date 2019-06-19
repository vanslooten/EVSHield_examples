# EVShield examples
Example Arduino sketches for the EVShield.
Some of these are described in [articles on my website](https://home.et.utwente.nl/slootenvanf/tag/evshield/).

# Issues
If you get "__vector_7" compile errors, edit NewPing.h (Arduino\libraries\NewPing\src): find "#define TIMER_ENABLED true" and change it to false

If you get further timer related errors, you might consider:

Pins | Controlled by timer
-----|--------------------
5,6  | timer0
9,10 | timer1
11,3 | timer2

This means that sensors or internal functions (eg. tone(), millis(), which use timer0) should not use the same timer.
Practically this means that the ultrsonic sensor is best off using eg. pins 9 and 10,
and the color senosr for instance 3,11 (and more).
