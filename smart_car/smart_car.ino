#include "Arduino.h"
#include "Car.hpp"
#include "smart_car.hpp"

/* Program for robot car. */

// For some reason this does not resolve.
extern HardwareSerial Serial;

std::ohserialstream cout(Serial);
std::ohserialstream cout1(Serial1);

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

