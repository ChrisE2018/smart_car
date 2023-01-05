#include "Arduino.h"
#include "src/robot/Car.hpp"
#include "smart_car.hpp"
#include "src/logging/Logger.hpp"
#include "src/logging/RobotAppender.hpp"

/* Program for robot car. */

// For some reason this does not resolve.
extern HardwareSerial Serial;

std::ohserialstream cout(Serial);

// Connections for HC05 bluetooth:
// rx1 19 yellow to 23 (far side)
// tx1 18 blue to 22 (near side)
std::ohserialstream cout1(Serial1);

Car *car;

/* Control program. */

void setup ()
{
    Serial.begin(9600);
    Serial.println("Smart car");
    Serial1.begin(9600);
    car = new Car();
    Logger::ROOT->add_appender(new RobotAppender(*car));
    car->setup();
    car->demo_drive_leds();
    Serial.println("Ready");
}

void loop ()
{
    car->cycle();
}

