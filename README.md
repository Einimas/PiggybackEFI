# PiggybackEFI
Piggyback version 1
Derived of MAP_EFI_V2.1.ino very crude... 
OP's written code might work where temperature is constant whole year round (way better than anything I could have written from scratch).



Main changes 1.1:

1 changed psi to bar (less conversions when adapting sensor)

2 added a crude IAT implementation (could increase the resolution)

3 listed more IO

Main changes 1.2:

1 added steinheart heart to calculate temperature precisely (stole some code from:https://forum.arduino.cc/t/bosch-automotive-pressure-temperature-sensor-0281006059-0281006060/980118); calculated the values precisely: https://docs.google.com/spreadsheets/d/1g7HUZ1HyK0B082gxlFn5IR-zISHINthUldbfXhKSpno/edit#gid=0

2 simplified map selection (pressure side)

Still untested.

Will test it on Mazda 121 with mono injection and a Z5 from 323 (thats why 4 injector outputs), on bikes after mazda gets speeduino with fresh wireing harness.

Plans:  additional sensors: coolant, lambda, throttle position; AFR correction using lambda reading; temperature dependant enritchment etc...


Use at your own risk or as inspiration. Experimental!!!!!
