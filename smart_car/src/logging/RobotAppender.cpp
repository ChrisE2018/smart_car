/*
 * RobotAppender.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#include <stdio.h>
#include <SPI.h>
#include "RobotAppender.hpp"
#include "UnixTime.hpp"

RobotAppender::RobotAppender (Car &car) :
                car(car)
{
}

void RobotAppender::append (const Logger *const logger, const Level level, const int line,
        const char *const message)
{
    const time_t t = get_unixtime(car);
    struct tm *const lt = localtime(&t);
    const int ms = millis() % 1000;
    snprintf(buffer, buffer_size, "%s.%03d [%s %s:%d] %s", isotime(lt), ms, stringify(level),
            logger->get_short_name().c_str(), line, message);
    buffer[buffer_size - 1] = '\0';
    append(level, buffer);
}

void RobotAppender::append (const Level level, const char *const message)
{
    if (static_cast<int>(level) <= static_cast<int>(Level::info))
    {
        if (usb_logger)
        {
            Serial.println(message);
        }
        if (bluetooth_logger)
        {
            Serial3.println(message);
        }
    }
    if (file_logger && log_file)
    {
        log_file.println(message);
        log_file.flush();
    }
}

void RobotAppender::get_logfile ()
{
    const time_t t = get_unixtime(car);
    struct tm *const lt = localtime(&t);
    const int size = snprintf(log_filename, filename_size, "LOGS/Y%4d/M%02d/D%02d/",
            2000 + lt->tm_year, lt->tm_mon + 1, lt->tm_mday);
    SD.mkdir(log_filename);
    snprintf(log_filename + size, filename_size - size, "L%02d-%02d.TXT", lt->tm_hour, lt->tm_min);
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

void RobotAppender::enable_usb_logger (const bool enable)
{
    usb_logger = enable;
}

void RobotAppender::enable_bluetooth_logger (const bool enable)
{
    bluetooth_logger = enable;
}

void RobotAppender::enable_file_logger (const bool enable)
{
    file_logger = enable;
}

bool RobotAppender::is_usb_logger ()
{
    return usb_logger;
}

bool RobotAppender::is_bluetooth_logger ()
{
    return bluetooth_logger;
}

bool RobotAppender::is_file_logger ()
{
    return file_logger;
}
