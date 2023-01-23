#include "Arduino.h"

#include "Logger.hpp"
#include "src/logging/SdAppender.hpp"
#include "SerialAppender.hpp"
#include "TimeSource.hpp"
#include "TimestampFormatter.hpp"

#include "src/robot/Car.hpp"
#include "src/robot/speed_counter.hpp"
#include "src/robot/heap.hpp"
#include "src/plugins/Plug.hpp"
#include "src/plugins/ClockPlugin.hpp"
#include "src/plugins/DrivePlugin.hpp"
#include "src/plugins/GoalPlugin.hpp"
#include "src/plugins/KalmanPlugin.hpp"
#include "src/plugins/MpuPlugin.hpp"
#include "src/plugins/OdomPlugin.hpp"
#include "src/plugins/UltrasoundPlugin.hpp"
#include "src/plugins/WallPlugin.hpp"

#include "smart_car.hpp"

/* Program for robot car. */

// For some reason this does not resolve.
extern HardwareSerial Serial;

std::ohserialstream cout(Serial);

// Connections for HC05 bluetooth:
// rx3 15 yellow to 23 (far side)
// tx3 14 blue to 22 (near side)
std::ohserialstream cout1(Serial3);

Plug plug;

ClockPlugin clock_plugin;
DrivePlugin forward_plugin(PluginId::FORWARD_PLUGIN, 500, MotorDirection::FORWARD, MotorDirection::FORWARD);
DrivePlugin reverse_plugin(PluginId::REVERSE_PLUGIN, 500, MotorDirection::REVERSE, MotorDirection::REVERSE);
GoalPlugin goal_plugin;
MpuPlugin mpu_plugin;

KalmanPlugin kalman_plugin;
OdomPlugin odom_plugin;
UltrasoundPlugin ultrasound_plugin;
WallPlugin wall_plugin;

Car car;
static unsigned long cycle_count = 0;

logging::TimestampFormatter *formatter = nullptr;
logging::SdAppender *sd_appender = nullptr;
logging::SerialAppender *usb_appender = nullptr;
logging::SerialAppender *bluetooth_appender = nullptr;

/* Control program. */

void setup ()
{
    Serial.begin(115200);
    Serial.println(F("Smart car"));
    Serial3.begin(115200);
    plug.add_plugin(&clock_plugin);
    plug.add_plugin(&forward_plugin);
    plug.add_plugin(&reverse_plugin);
    plug.add_plugin(&goal_plugin);
    plug.add_plugin(&mpu_plugin);
    plug.add_plugin(&kalman_plugin);
    plug.add_plugin(&odom_plugin);
    plug.add_plugin(&ultrasound_plugin);
    plug.add_plugin(&wall_plugin);

    formatter = new logging::TimestampFormatter(clock_plugin);
    usb_appender = new logging::SerialAppender(Serial, logging::Level::info, *formatter);
    bluetooth_appender = new logging::SerialAppender(Serial3, logging::Level::info, *formatter);
    sd_appender = new logging::SdAppender(PIN_53_SS, logging::Level::info, *formatter, clock_plugin);
    logging::Logger::root->add_appender(usb_appender);
    logging::Logger::root->add_appender(bluetooth_appender);
    logging::Logger::root->add_appender(sd_appender);
    sd_appender->open_logfile();
    car.setup();
    car.demo_drive_leds();
    setup_speed_counter();
    const int heap_buffer_size = 100;
    char heap_buffer[heap_buffer_size];
    get_heap_state(heap_buffer, heap_buffer_size);
    //sd_appender->log_data("/HISTORY", "STARTUP.TXT", heap_buffer);
    Serial.println(heap_buffer);
    Serial.print(F("C++ version "));
    Serial.println(__cplusplus);
    Serial.println(F("Ready"));
    Serial.println();
    // Don't send to bluetooth unless it is active
    bluetooth_appender->set_level(logging::Level::none);
}

// @see https://forum.arduino.cc/t/constant-run-time-of-a-loop/568829/2
void loop ()
{
    const unsigned long cycle = millis();
    if (cycle != cycle_count)
    {
        cycle_count = cycle;
        car.cycle();
        plug.cycle();
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

