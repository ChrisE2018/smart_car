/*
 * RobotAppender.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#include "RobotAppender.hpp"
#include <SPI.h>
#include "../plugins/ClockPlugin.hpp"

RobotAppender::RobotAppender (Car &car) :
                car(car)
{
}

void RobotAppender::append (const Logger *logger, const Level level, const int line,
        const char *message)
{
    time_t t = 0;
    ClockPlugin *const clock_plugin = car.get_clock_plugin();
    if (clock_plugin != nullptr)
    {
        t = clock_plugin->get_unixtime();
    }
    struct tm *lt = localtime(&t);
    const int ms = millis() % 1000;
    char buf[buffer_size];
    snprintf(buf, buffer_size, "%s.%03d [%s %s:%d] %s", isotime(lt), ms, stringify(level),
            logger->get_short_name().c_str(), line, message);
    Serial.println(buf);
    Serial3.println(buf);
    if (log_file)
    {
        log_file.println(buf);
    }
}

String RobotAppender::get_logfile ()
{
    time_t t = 0;
    ClockPlugin *const clock_plugin = car.get_clock_plugin();
    if (clock_plugin != nullptr)
    {
        t = clock_plugin->get_unixtime();
    }
    struct tm *lt = localtime(&t);
    char buf[32];
    snprintf(buf, 32, "log%s.txt", isotime(lt));
    String result(buf);
    result.replace(' ', '_');
    return result;
}

void RobotAppender::open_logfile ()
{
    Serial.print(F("Initializing SD card"));

    if (!SD.begin(chipSelect))
    {
        Serial.println(F("SD card failed"));
    }
    else
    {
        Serial.println(F("SD card initialized"));
        String filename = get_logfile();
        Serial.print(F("Creating "));
        Serial.println(filename);
        log_file = SD.open(filename, FILE_WRITE);
    }
}
void RobotAppender::flush ()
{
    if (log_file)
    {
        log_file.flush();
    }
}

void RobotAppender::close ()
{
    if (log_file)
    {
        log_file.close();
    }
}
