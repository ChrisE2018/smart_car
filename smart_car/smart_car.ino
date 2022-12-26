#include "Arduino.h"
#include "Car.hpp"
#include "Motor.hpp"

// Program for robot car

Car car;

/* Control program. */

void setup ()
{
    Serial.begin(9600);
    Serial1.begin(9600);
    Serial.println("Smart car");
    car.setup();
    car.demo_drive_leds();
    Serial.println("Ready");
}

void loop ()
{
    car.cycle();
}

