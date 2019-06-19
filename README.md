# EVShield examples
Example Arduino sketches for the EVShield.
Some of these are described in [articles on my website](https://home.et.utwente.nl/slootenvanf/tag/evshield/).

# Issues
If you get "__vector_7" compile errors, edit NewPing.h (Arduino\libraries\NewPing\src): find "#define TIMER_ENABLED true" and change it to false

If you get further timer related errors, you might consider:

Pins 5 and 6 are controlled by timer0
Pins 9 and 10 are controlled by timer1
Pins 11 and 3 are controlled by timer2

this means that sensors or internal functions (eg. tone(), millis(), which use timer0) should not use the same timer.
Practically this means that the ultrsonic sensor is best off using eg. pins 9 and 10,
and the color senosr for instance 3,11 (and more).
