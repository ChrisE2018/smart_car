/*
 * Copyright 2023 Christopher Eliot
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "SdAppender.hpp"

namespace logging
{

SdAppender::SdAppender (const int chip_select, const Level level, Formatter &formatter, TimeSource &time_source) :
        Appender(level, formatter), chip_select(chip_select), time_source(time_source)
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

    if (!SD.begin(chip_select))
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
    const time_t t = time_source.unixtime();
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

        const time_t t = time_source.unixtime();
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
