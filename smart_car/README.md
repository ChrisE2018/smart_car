# Summary

This software controls a smart car robot using an Arduino Mega 2560 microcontroller. The car is
equipped with separate motors and controls for each of the four wheels, speed sensors for
each wheel, a clock, MPU, bluetooth, ultrasound and micro SD card. 

The overall goal is to implement a smart car that can navigate and solve simple
problems in a known space. Ideally, the project will incorporate a vision system
for better localization.

The software is organized as a set of plugins each dedicated to a particular function.
The main Arduino loop function ends up calling the cycle method for each plugin that
requires it. The Plugin base class provides a standard interface and collects some
statistics. Each motor has its own low level and PID controller. Navigation is implemented
using a Kalman filter to perform sensor fusion of data provided by odometry, the MPU and
the ultrasound.

The car can be tested when it is tethered to a host computer by a USB cable or running
independently on battery power with bluetooth communication to a smart phone.
There is a powerful command processor that support both serial communication via USB
or bluetooth. A powerful logging facility, inspired by log4j, was developed as part
of this project and later split off into its own MIT licensed project available at
https://github.com/ChrisE2018/logging.

This is a personal project implemented with no external support.

# Status

The general architecture has been implemented and most low-level functions are in place.
Integration of the odometry, PID control and navigation is under development. The Kalman
filter is currently ignored, and navigation only uses odometry. When odometry is sufficiently
validated, the plan is to re-implement sensor fusion and combine MPU data with odometry.
Fusion of ultrasound data has not yet been attempted.

# Future Work

A Beaglebone Black single board computer will be added to the car to expand
the processing capabilities. This may replace the Arduino or supplement it.
Several architectures are under consideration. The Mega might be kept or
replaced with 1-5 Seeed Studio XIAO ESP32C3 microcontrollers to implement
high speed PID control and MPU support, using the Beagle for high level
problem solving and navigation. Alternatively, the Beagle might be fast
enough to perform all required processing, its capabilities have not been
fully explored yet.

Vision capabilities, if possible, would open up many development paths.
The first goal for vision is to recognize known locations to support
localization. Mapping of unknown spaces could be implemented along with
response to changing state in known spaces (open or closed doors for example).

# Credits

Jeroen F.J. Laros reviewed the logging code and provided useful feedback.

The heap.cpp source has been reviewed by several people on the Arduino forum,
including in no particular order, DrDiettrich, Coding Badly, WestfW and oqibidpo.