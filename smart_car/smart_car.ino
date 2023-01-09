#include "Arduino.h"
#include "src/robot/Car.hpp"
#include "smart_car.hpp"
#include "src/robot/speed_counter.hpp"
#include "src/logging/Logger.hpp"
#include "src/logging/RobotAppender.hpp"

/* Program for robot car. */

// For some reason this does not resolve.
extern HardwareSerial Serial;

std::ohserialstream cout(Serial);

// Connections for HC05 bluetooth:
// rx3 15 yellow to 23 (far side)
// tx3 14 blue to 22 (near side)
std::ohserialstream cout1(Serial3);

Car *car;

/* Control program. */

void setup ()
{
    Serial.begin(9600);
    Serial.println("Smart car");
    Serial3.begin(9600);
    car = new Car();
    Logger::ROOT->add_appender(new RobotAppender(*car));
    car->setup();
    car->demo_drive_leds();
    setup_speed_counter();
    Serial.println("Ready");
    Serial.println("");
}

// @see https://forum.arduino.cc/t/constant-run-time-of-a-loop/568829/2
void loop ()
{
    car->cycle();
}

