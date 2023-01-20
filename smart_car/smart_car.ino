#include "Arduino.h"
#include "src/robot/Car.hpp"
#include "smart_car.hpp"
#include "src/robot/speed_counter.hpp"
#include "src/robot/heap.hpp"
#include "src/logging/Logger.hpp"
#include "src/logging/RobotAppender.hpp"
#include "src/logging/SerialAppender.hpp"
#include "src/logging/StandardFormatter.hpp"
#include "src/logging/TimeSource.hpp"
#include "src/plugins/ClockPlugin.hpp"

/* Program for robot car. */

// For some reason this does not resolve.
extern HardwareSerial Serial;

std::ohserialstream cout(Serial);

// Connections for HC05 bluetooth:
// rx3 15 yellow to 23 (far side)
// tx3 14 blue to 22 (near side)
std::ohserialstream cout1(Serial3);

static Car *car;
static unsigned long cycle_count = 0;

logging::RobotAppender *robot_appender = nullptr;
logging::SerialAppender *usb_appender = nullptr;

/* Control program. */

void setup ()
{
    Serial.begin(115200);
    Serial.println(F("Smart car"));
    Serial3.begin(115200);
    car = new Car();
    logging::TimeSource &time_source = *car->get_clock_plugin();
    logging::StandardFormatter *formatter = new logging::StandardFormatter(time_source);
    robot_appender = new logging::RobotAppender(logging::Level::info, *formatter, time_source);
    usb_appender = new logging::SerialAppender(logging::Level::info, *formatter);
    logging::Logger::ROOT->add_appender(robot_appender);
    robot_appender->open_logfile();
    car->setup();
    car->demo_drive_leds();
    setup_speed_counter();
    const int heap_buffer_size = 100;
    char heap_buffer[heap_buffer_size];
    get_heap_state(heap_buffer, heap_buffer_size);
    robot_appender->log_data("/HISTORY", "STARTUP.TXT", heap_buffer);
    Serial.println(heap_buffer);
    Serial.print(F("C++ version "));
    Serial.println(__cplusplus);
    Serial.println(F("Ready"));
    Serial.println();
    // Don't send to bluetooth unless it is active
    robot_appender->enable_bluetooth_logger(false);
}

// @see https://forum.arduino.cc/t/constant-run-time-of-a-loop/568829/2
void loop ()
{
    const unsigned long cycle = millis();
    if (cycle != cycle_count)
    {
        cycle_count = cycle;
        car->cycle();
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

