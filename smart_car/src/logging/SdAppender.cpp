/*
 * SdAppender.cpp
 *
 *  Created on: Jan 20, 2023
 *      Author: cre
 */

#include "SdAppender.hpp"

namespace logging
{

SdAppender::SdAppender (const Level level, Formatter &formatter, TimeSource &time_source) :
        Appender(level, formatter), time_source(time_source)
{
}

void SdAppender::append (const Level _level, const char *const message)
{
    if (static_cast<int>(_level) <= static_cast<int>(level))
    {
        if (log_file)
        {
            log_file.println(message);
        }
    }
}

void SdAppender::open_logfile ()
{
    Serial.println(F("Initializing SD card"));

    if (!SD.begin(chipSelect))
    {
        Serial.println(F("SD card failed"));
    }
    else
    {
        Serial.println(F("SD card initialized"));
        set_log_pathname();
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

void SdAppender::set_log_pathname ()
{
    const time_t t = time_source.get_unixtime();
    struct tm *const lt = localtime(&t);
    const int size = snprintf(log_filename, filename_size, "LOGS/Y%4d/M%02d/D%02d/", 1900 + lt->tm_year, lt->tm_mon + 1,
            lt->tm_mday);
    SD.mkdir(log_filename);
    snprintf(log_filename + size, filename_size - size, "L%02d-%02d.TXT", lt->tm_hour, lt->tm_min);
}

void SdAppender::flush ()
{
    if (log_file)
    {
        log_file.flush();
    }
}

void SdAppender::close ()
{
    if (log_file)
    {
        log_file.close();
    }
}

bool SdAppender::log_data (String folder, String filename, const char *message)
{
    folder.toUpperCase();
    filename.toUpperCase();
    SD.mkdir(folder);
    constexpr int buf_size = 32;
    char buf[buf_size];
    snprintf(buf, buf_size, "%s/%s", folder.c_str(), filename.c_str());
    File stream = SD.open(buf, FILE_WRITE);
    if (stream)
    {
        Serial.print(F("Opened "));
        Serial.println(buf);

        const time_t t = time_source.get_unixtime();
        struct tm *const lt = localtime(&t);
        const int ms = millis() % 1000;
        const int n = snprintf(buf, buf_size, "%s.%03d ", isotime(lt), ms);
        stream.print(buf);
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

bool SdAppender::save_data (String folder, String filename, const char *message)
{
    folder.toUpperCase();
    filename.toUpperCase();
    SD.mkdir(folder);
    constexpr int buf_size = 32;
    char buf[buf_size];
    snprintf(buf, buf_size, "%s/%s", folder.c_str(), filename.c_str());
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
}
