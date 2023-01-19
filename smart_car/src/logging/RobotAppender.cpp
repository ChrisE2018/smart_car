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

RobotAppender *robot_appender = nullptr;

RobotAppender::RobotAppender (Car &car, const Level level) :
        car(car), level(level)
{
}

void RobotAppender::append (const Logger *const logger, const Level level, const int line, const char *const message)
{
    const time_t t = get_unixtime(car);
    struct tm *const lt = localtime(&t);
    const int ms = millis() % 1000;
    snprintf(buffer, buffer_size, "%s.%03d [%s %s:%d] %s", isotime(lt), ms, stringify(level),
            logger->get_short_name().c_str(), line, message);
    buffer[buffer_size - 1] = '\0';
    append(level, buffer);
}

void RobotAppender::append (const Level _level, const char *const message)
{
    if (static_cast<int>(_level) <= static_cast<int>(level))
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
    if (log_file)
    {
        if (file_logger)
        {
            log_file.println(message);
            //log_file.flush();
        }
    }
}

void RobotAppender::append_usb (const char *const message)
{
    Serial.println(message);
}

void RobotAppender::append_bluetooth (const char *const message)
{
    Serial3.println(message);
}

void RobotAppender::append_file (const char *const message, const bool flush = false)
{
    if (log_file)
    {
        log_file.println(message);
        if (flush)
        {
            log_file.flush();
        }
    }
}

void RobotAppender::log_data_p (const char *format, ...)
{
    va_list args;
    va_start(args, format);
    const time_t t = get_unixtime(car);
    struct tm *const lt = localtime(&t);
    const int ms = millis() % 1000;
    const int n = snprintf(buffer, buffer_size, "%s.%03d", isotime(lt), ms);
    vsnprintf_P(buffer + n, buffer_size - n, format, args);
    append_file(buffer, false);
    va_end(args);
}

void RobotAppender::get_logfile ()
{
    const time_t t = get_unixtime(car);
    struct tm *const lt = localtime(&t);
    const int size = snprintf(log_filename, filename_size, "LOGS/Y%4d/M%02d/D%02d/", 1900 + lt->tm_year, lt->tm_mon + 1,
            lt->tm_mday);
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

bool RobotAppender::log_data (String folder, String filename, const char *message)
{
    folder.toUpperCase();
    filename.toUpperCase();
    SD.mkdir(folder);
    char buf[32];
    snprintf(buf, 32, "%s/%s", folder.c_str(), filename.c_str());
    File stream = SD.open(buf, FILE_WRITE);
    if (stream)
    {
        Serial.print(F("Opened "));
        Serial.println(buf);
        stream.println(message);
        stream.flush();
        stream.close();
        return true;
    }
    else
    {
        Serial.print(F("Open failed "));
        Serial.println(buf);
        return false;
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

bool RobotAppender::get_logger_state (const String &mode) const
{
    if (mode == F("usb"))
    {
        return usb_logger;
    }
    else if (mode == F("bluetooth"))
    {
        return bluetooth_logger;
    }
    else if (mode == F("file"))
    {
        return file_logger;
    }
    return false;
}

void RobotAppender::set_logger_state (const String &mode, const String &state)
{
    bool enable = false;
    if (state == F("on") || state == F("true"))
    {
        enable = true;
    }
    if (mode == F("usb"))
    {
        usb_logger = enable;
    }
    else if (mode == F("bluetooth"))
    {
        bluetooth_logger = enable;
    }
    else if (mode == F("file"))
    {
        file_logger = enable;
    }
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
