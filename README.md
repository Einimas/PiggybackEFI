# PiggybackEFI
Piggyback version 1
Derived of MAP_EFI_V2.1.ino very crude... 
OP's written code might work where temperature is constant whole year round (way better than anything I could have written from scratch).


Main changes:
A changed psi to bar (less conversions when adapting sensor)
B added a crude IAT implementation (could increase the resolution)
C listed more IO


Will test it on Mazda 121 with mono injection and a Z5 from 323 .

Plans:  additional sensors: coolant, lambda, throttle position; AFR correction using lambda reading; temperature dependant enritchment etc...
