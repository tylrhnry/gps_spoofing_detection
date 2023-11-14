# GPS Spoofing Detection

![electronics hardware](./md_img/pi_zero_gps.jpg)

This project provides a way to detect GPS spoofing without additional hardware
that isn't already present on most GPS-enabled devices (eg. an accelerometer/gyroscope).
The way it does this is by integrating the acceleration of the device to constantly 
provide an estimated velocity and position based on the previous information. If at 
any point, the time, date, or location doesn't match what it should, then spoofing is 
highly likely.

This program runs on a Raspberry Pi Zero, with a Neo-6M GPS module and a GY-521 
accelerometer/gyroscope. The GPS module is connected to the board via the serial 
connection on UART pins (GPIO pins 14 and 15). The accelerometer/gyro is connected to 
the Pi on the I2C pins (GPIO pins 2 and 3). 

Because this project is made to run on specific hardware and compiled for the 
aarch64-unknown-linux-gnu platform, I have not been able to build or run this project 
on my laptop. I am, however, able to `cargo check` and verfiy the syntax. I will be 
adding tests to better verify my program logic, although, because I am recieving a 
constant stream of data from the sensors, I can't pass in data the same way for all 
of the functions (the main ones testing benefits from).