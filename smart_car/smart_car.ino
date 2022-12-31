#include "Arduino.h"
#include "Car.hpp"
#include "smart_car.hpp"

/* Program for robot car. */

std::ohserialstream cout(Serial);

Car *car;


/* Control program. */

void setup ()
{
    Serial.begin(9600);
    Serial.println("Smart car");
    Serial1.begin(9600);
    car = new Car();
    car->setup();
    car->demo_drive_leds();
    Serial.println("Ready");
}

void loop ()
{
    car->cycle();
}

