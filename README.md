# GPS Spoofing Detection

![electronics hardware](pi_zero_gps.jpg)

This project provides a way to detect GPS spoofing without additional hardware
that isn't already present on most GPS-enabled devices (eg. an accelerometer/gyroscope).
The way it does this is by integrating the acceleration of the device to constantly 
provide an estimated velocity and position based on the previous information. If at 
any point, the time, date, or location doesn't match what it should, then spoofing is 
highly likely.


