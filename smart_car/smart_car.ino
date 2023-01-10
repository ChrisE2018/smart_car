#include "Arduino.h"
#include "src/robot/Car.hpp"
#include "smart_car.hpp"
#include "src/robot/speed_counter.hpp"
#include "src/robot/heap.hpp"
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

static Car *car;
static RobotAppender *robot_appender = nullptr;
static unsigned long cycle_time = 0;

/* Control program. */

void setup ()
{
    Serial.begin(9600);
    Serial.println(F("Smart car"));
    Serial3.begin(9600);
    car = new Car();
    robot_appender = new RobotAppender(*car);
    Logger::ROOT->add_appender(robot_appender);
    car->setup();
//    robot_appender->open_logfile();
    car->demo_drive_leds();
    setup_speed_counter();
    print_heap_state ();
    Serial.println(F("Ready"));
    Serial.println();
}

// @see https://forum.arduino.cc/t/constant-run-time-of-a-loop/568829/2
void loop ()
{
    const unsigned long now = millis();
    const unsigned long cycle = now / 10;
    if (now != cycle_time)
    {
        cycle_time = now;
        car->cycle();
    }
    else
    {
        // We have part of a cycle idle time
        robot_appender->flush();
    }
}

/* The rule of thumb is: Your FLASH usage is fine as long as it does not go
 * above 100% but your SRAM usage has to leave room for your stack and heap
 * which will depend on your execution path. If your sketch goes wonky
 * try reducing your SRAM usage.
 * If you have any .print("String constant"); calls the easiest fix is to change them
 * to .print(F("String constant")); to keep your string constants from being stored
 * in both FLASH and SRAM.
 *
 * Board            Microcontroller  Family  Architecture    Flash   SRAM    EEPROM
 * Mega 2560 Rev3   ATmega2560      AVR      Harvard         256kB   8kB     4kB
 */

