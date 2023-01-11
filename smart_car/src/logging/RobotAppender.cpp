/*
 * RobotAppender.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#include "RobotAppender.hpp"
#include <stdio.h>
#include <SPI.h>
#include "../plugins/ClockPlugin.hpp"

RobotAppender::RobotAppender (Car &car) :
                car(car)
{
}

void RobotAppender::append (const Logger *const logger, const Level level, const int line,
        const char *const message)
{
    time_t t = 0;
    ClockPlugin *const clock_plugin = car.get_clock_plugin();
    if (clock_plugin != nullptr)
    {
        t = clock_plugin->get_unixtime();
    }
    struct tm *const lt = localtime(&t);
    const int ms = millis() % 1000;
    snprintf(buffer, buffer_size, "%s.%03d [%s %s:%d] %s", isotime(lt), ms, stringify(level),
            logger->get_short_name().c_str(), line, message);
    buffer[buffer_size - 1] = '\0';
    append(buffer);
}

void RobotAppender::append(const char *const message)
{
    Serial.println(message);
    Serial3.println(message);
    if (log_file)
    {
        log_file.println(message);
        log_file.flush();
    }
}

void RobotAppender::get_logfile ()
{
    time_t t = 0;
    ClockPlugin *const clock_plugin = car.get_clock_plugin();
    if (clock_plugin != nullptr)
    {
        t = clock_plugin->get_unixtime();
    }
    struct tm *const lt = localtime(&t);
    snprintf(log_filename, filename_size, "L%02d%03d.TXT", lt->tm_year, lt->tm_yday);
}

void RobotAppender::open_logfile ()
{
    Serial.println(F("Initializing SD card"));

    if (!SD.begin(chipSelect))
    {
        Serial.println(F("SD card failed"));
    }
    else
    {
        Serial.println(F("SD card initialized"));
        get_logfile();
        log_file = SD.open(log_filename, FILE_WRITE);
        if (log_file)
        {
            Serial.print(F("Created "));
            Serial.println(log_filename);
            log_file.println("[logfile open]");
        }
        else
        {
            Serial.print(F("Could not create logfile "));
            Serial.println(log_filename);
        }
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
